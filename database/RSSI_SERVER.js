var SerialPort = require("serialport");
var app = require('express')();
var xbee_api = require('xbee-api');
var fs = require('fs');

var C = xbee_api.constants;
var XBeeAPI = new xbee_api.XBeeAPI({
    api_mode: 2,      // [1, 2]; 1 is default, 2 is with escaping (set ATAP=2) 
});

var portName = process.argv[2];

var sampleDelay = 3000;
var counter = 0;

var BIN_NUMBER = '54';

//Note that with the XBeeAPI parser, the serialport's "data" event will not fire when messages are received!
portConfig = {
    baudRate: 9600,
    parser: XBeeAPI.rawParser()

};

var sp;
sp = new SerialPort.SerialPort(portName, portConfig);


//Create a packet to be sent to all other XBEE units on the PAN.
// The value of 'data' is meaningless, for now.
var RSSIRequestPacket = {
  type: C.FRAME_TYPE.ZIGBEE_TRANSMIT_REQUEST,
  destination64: "000000000000ffff",
  broadcastRadius: 0x01,
  options: 0x00,
  data: "test"
}

var requestRSSI = function(){
  sp.write(XBeeAPI.buildFrame(RSSIRequestPacket));
}

sp.on("open", function () {
  console.log('open');
  requestRSSI();
  // console.log("following request");
  // sp.on('data', function(data) {
  //   console.log('data received: ' + data);
  // });
  setInterval(requestRSSI, sampleDelay);
});



XBeeAPI.on("frame_object", function(frame) {
  if (frame.type == 144){
    counter++;
    console.log(counter + ": Beacon ID: " + frame.data[1] + ", RSSI: " + (frame.data[0]));
    fs.appendFile('beacon_rssi_data.txt', BIN_NUMBER + ',' + frame.data[1] + ',' + frame.data[0] + '\n', function(err) {
        if (err) {
          return console.error(err);
        }
    });
  }
});




