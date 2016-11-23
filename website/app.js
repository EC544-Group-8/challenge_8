var SerialPort = require("serialport");
var express = require('express');
var app = express();
var http = require('http').Server(app);

// portConfig = {
// 	baudRate: 9600,
// 	parser: SerialPort.parsers.readline("\n")
// };

// ---- Brought over from "/matlab/RSSI_SERVER.js" ---- //
var client = require('./client');
var xbee_api = require('xbee-api');
var C = xbee_api.constants;
var XBeeAPI = new xbee_api.XBeeAPI({
    api_mode: 2,      // [1, 2]; 1 is default, 2 is with escaping (set ATAP=2) 
});

var portName = process.argv[2];//******************************************************************************
//Note that with the XBeeAPI parser, the serialport's "data" event will not fire when messages are received!
portConfig = {
    baudRate: 9600,
    parser: XBeeAPI.rawParser()

};

var sp = new SerialPort.SerialPort(portName, portConfig);//*****************************************************

var host='localhost';
var port=5000;
var c = new client(host, port);

// Begin the function that handles the receipt of data from Matlab, and adds it to a queue
c.receive();

app.use(express.static(__dirname + '/public'));

app.get('/', function(req, res){
  res.sendfile('index.html');
});

// Setup the the web server
http.listen(3000, function(){
  console.log('listening on *:3000');
});


// ------------ BEGIN - SEND RSSI REQUEST FROM COORDINATOR ------------ //

//Create a packet to be sent to all other XBEE units on the PAN to request RSSI packets back to coordinator.
// The value of 'data' is meaningless, for now.
var RSSIRequestPacket = {
  type: C.FRAME_TYPE.ZIGBEE_TRANSMIT_REQUEST,
  destination64: "000000000000ffff",
  broadcastRadius: 0x01,
  options: 0x00,
  data: "test"
};

var requestRSSI = function(){
  sp.write(XBeeAPI.buildFrame(RSSIRequestPacket));//**********************************************************
};


var sampleDelay = 3000;
// Every "sampleDelay" seconds, gather new RSSI values********************************************************
sp.on("open", function () {
  console.log('open');
  requestRSSI();
  setInterval(requestRSSI, sampleDelay);
});

// ------------ END - SEND RSSI REQUEST FROM COORDINATOR ------------ //




// --------- BEGIN - HANDLE RSSI VALUES FROM THE NODES ---------- //

// Instantiate the beacon data array
var beacon_data = {};
var bd_length = Object.keys(beacon_data).length;

// Reset the beacon data after sent to matlab
var resetBeaconData = function() {
  beacon_data = {};
  bd_length = 0;
};

// When the Coordinator Xbee receives the RSSI values, gather them, and send them to matlab
XBeeAPI.on("frame_object", function(frame) {
  if (frame.type == 144){
    console.log("Beacon ID: " + frame.data[1] + ", RSSI: " + (frame.data[0]));
    beacon_data[frame.data[1]] = frame.data[0];
    bd_length = Object.keys(beacon_data).length;
    console.log(bd_length);

    if(bd_length >= 4){
      // Send to Matlab
      var data_to_send = beacon_data['1'] + ',' + beacon_data['2'] + ',' +beacon_data['3'] + ',' +beacon_data['4'];
      c.send(data_to_send);
      // Reset beacon data
      resetBeaconData();
    }
  }
});

//setInterval(function(){c.send('59,60,62,74'); /*console.log('sent');*/},3000);

// ------------ END - HANDLE RSSI VALUES FROM THE NODES ------------ //





// --------- DEFINE AJAX POST REQUESTS HERE --------- //

// For getting the most recent location of the moving device
app.get('/get_location', function(req, res){
	// Send the current bin_id back to the view
	res.send(c.queue[c.queue.length - 1]);
});

// setInterval(function(){c.send('[52,65,75,65]'); console.log('data sent');},5000);   // 7
// setInterval(function(){c.send('[30,40,50,60]'); console.log('data sent');},6000);   // 9
// setInterval(function(){c.send('[60,51,66,81]'); console.log('data sent');},7000);   // 13
// setInterval(function(){c.send('[60,51,66,81]'); console.log('data sent');},8000);   // 13
// setInterval(function(){c.send('[60,51,66,81]'); console.log('data sent');},9000);   // 13
