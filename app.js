var dgram = require('dgram');
var bufferpack = require('bufferpack');
var s = dgram.createSocket('udp4');
s.on('message', function(msg, rinfo) {
	var data = bufferpack.unpack('<BffffLL',msg,0);
	console.log('I got this message: ' + data);

});
s.bind(5005);