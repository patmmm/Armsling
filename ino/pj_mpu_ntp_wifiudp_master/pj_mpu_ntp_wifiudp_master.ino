#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

#include "MPU6050_6Axis_MotionApps20.h"
// #include "MPU6050.h" // not necessary if using MotionApps include file
int nodeNo = 0;
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

typedef union float2byte {
  float fval;
  char bval[4];
};
WiFiUDP Udp;
NTPClient timeClient(Udp);
const char* ssid     = "Meow";
const char* password = "88888888";
char incomingPacket[255];

const char *host = "128.199.101.16";
int localUdpPort = 5005;

typedef union long2byte {
  long lval;
  char bval[4];
};

long2byte epochTime, epochMillis;
void setup() {
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  delay(10);

  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Updating time");
  timeClient.begin();
  while (!timeClient.update())
    Serial.print(".");
  Serial.println();
  Serial.println("done");
  pinMode(D4, OUTPUT);
  syncMaster();
  //// after sync
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  // TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to ");
  Serial.println(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("done");

  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

  Serial.println(F("Initializing I2C devices..."));


  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-2492);
  mpu.setYAccelOffset(1517);
  mpu.setZAccelOffset(883);
  mpu.setXGyroOffset(67);
  mpu.setYGyroOffset(-58);
  mpu.setZGyroOffset(22);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(15, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}
int value = 0;

void loop() {// if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    char buff[100];
    float2byte w, x, y, z;
    w.fval = q.w;
    x.fval = q.x;
    y.fval = q.y;
    z.fval = q.z;
    long2byte mag;
    mag.lval = sqrt(pow(aaWorld.x, 2) + pow(aaWorld.y, 2) + pow(aaWorld.z, 2));
    epochTime.lval = timeClient.getEpochTime();
    epochMillis.lval = timeClient.getMicroSeconds();
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.print(q.z);
    Serial.print("\t");
    Serial.print(epochTime.lval);
    Serial.print(".");
    Serial.println(epochMillis.lval);
    //    dtostrf(q.w, 4, 2, w);
    //    dtostrf(q.x, 4, 2, x);
    //    dtostrf(q.y, 4, 2, y);
    //    dtostrf(q.z, 4, 2, z);
    //    sprintf(buff, "$%s\t%s\t%s\t%s\t%d", w.bval, x.bval, y.bval, z.bval, timeClient.getEpochTime());// w x y z time
    //    Serial.print(buff);
    Udp.beginPacket(host, localUdpPort);
    Udp.write(nodeNo);
    Udp.write(w.bval, 4);
    Udp.write(x.bval, 4);
    Udp.write(y.bval, 4);
    Udp.write(z.bval, 4);
    Udp.write(mag.bval, 4);
    Udp.write(epochTime.bval, 4);
    Udp.write(epochMillis.bval, 4);
    if (!Udp.endPacket()) {
      Serial.println("error");
    }
  }

  long t1 = timeClient.getEpochTime();
  digitalWrite(D4, t1 % 2);

}
int maxDeviceCount = 0;

void syncMaster() {
  const char *ssid = "ESPap";
  const char *password = "thereisnospoon";
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  //  while (WiFi.status() != WL_CONNECTED) {
  //    delay(500);
  //    Serial.print(".");
  //  }
  int deviceCount = 0;
  Udp.begin(5000);
  while (deviceCount != maxDeviceCount) {
    int packetSize = Udp.parsePacket();
    if (packetSize)
    {
      //      Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
      int len = Udp.read(incomingPacket, 255);
      if (len > 0)
      {
        incomingPacket[len] = 0;
      }
      if (len == 2) {
        deviceCount++;
        Serial.println("done");
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("ok");
        Udp.endPacket();
        continue;
      }
      epochTime.lval = timeClient.getEpochTime();
      epochMillis.lval = timeClient.getMicroSeconds();
      //      Serial.printf("UDP packet contents: %s\n", incomingPacket);
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(incomingPacket, 4);
      Udp.write(epochTime.bval, 4);
      Udp.write(epochMillis.bval, 4);
      Udp.write(incomingPacket[4]);
      Udp.endPacket();
    }
  }

}
