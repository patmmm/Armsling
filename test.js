const express = require('express')
const app = express()
var dgram = require('dgram');
var bufferpack = require('bufferpack');
var s = dgram.createSocket('udp4');


//set the template engine ejs
app.set('view engine', 'ejs')

//middlewares
app.use(express.static('public'))


//routes
app.get('/', (req, res) => {
	res.render('index')
})

app.get('/cube', (req, res) => {
    res.render('cube')
})

app.get('/cube2', (req, res) => {
    res.render('cube2')
})

//Listen on port 3000
server = app.listen(8080)



//socket.io instantiation
const io = require("socket.io")(server)


//listen on every connection
io.on('connection', (socket) => {
	console.log('New user connected')

	//default username
	socket.username = "Anonymous"

    //listen on change_username
    socket.on('change_username', (data) => {
        socket.username = data.username
    })

    //listen on new_message
    socket.on('new_message', (data) => {
        //broadcast the new message
        io.sockets.emit('new_message', {message : data.message, username : socket.username});
    })

    //listen on typing
    socket.on('typing', (data) => {
    	socket.broadcast.emit('typing', {username : socket.username})
    })
})

s.on('message', function(msg, rinfo) {
	var data = bufferpack.unpack('<BffffLL',msg,0);
	// console.log('I got this message: ' + data);
	io.emit('new_message', {message : data.slice(1,), username : 'Sensor'+data[0]});
});
s.bind(5005);