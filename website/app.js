var SerialPort = require("serialport");
var express = require('express');
var app = express();
var http = require('http').Server(app);

var xbee_api = require('xbee-api');
var C = xbee_api.constants;
var XBeeAPI = new xbee_api.XBeeAPI({
    api_mode: 2,      // [1, 2]; 1 is default, 2 is with escaping (set ATAP=2) 
});

var portName = process.argv[2];//******************************************************************************
//Note that with the XBeeAPI parser, the serialport's "data" event will not fire when messages are received!
portConfig = {
    baudRate: 57600,
    parser: XBeeAPI.rawParser()

};

var sp = new SerialPort.SerialPort(portName, portConfig);//*****************************************************

// raspberry PI GPIO Pins
var start_stop_pin_number = 23;
var safe_to_turn_pin_number = 18;

var Gpio = require('onoff').Gpio,
  start_stop_pin = new Gpio(start_stop_pin_number, 'out'),
  safe_to_turn_pin = new Gpio(safe_to_turn_pin_number, 'out');
 
// Make sure the led is turned off to start
start_stop_pin.writeSync(0);
safe_to_turn_pin.writeSync(0);

// Unexport GPIO and free resources on ctrl+c
process.on('SIGINT', function () {
 start_stop_pin.unexport();
 safe_to_turn_pin.unexport();
});

// // functions
function updateSafeTurn(status) {
  safe_to_turn_pin.writeSync(status);
  console.log('Safe to turn Status set to: ' + String(status));
}

function updateStartStop(status) {
  start_stop_pin.writeSync(status);
  console.log('Stop start Status set to: ' + String(status));
}


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


var sampleDelay = 2500;
// Every "sampleDelay" seconds, gather new RSSI values********************************************************
sp.on("open", function () {
  console.log('open');
  requestRSSI();
  setInterval(requestRSSI, sampleDelay);
});

// ------------ END - SEND RSSI REQUEST FROM COORDINATOR ------------ //

// global variables
var fs = require('fs');
var parse = require('csv-parse');
var csvData=[];
var bin_history = [];

// read the file and save to global variable
fs.createReadStream('DB_AVG.txt')
    .pipe(parse({delimiter: ','}))
    .on('data', function(csvrow) {
        csvData.push(csvrow);
        //console.log(csvrow);
    });


// predict the nth neighbors
function predict(sample) {
    var delta = {};
    var nbr = -1;
    // loop through the matrix
    for (i = 0; i < csvData.length; i++) {
        var sum = 0;
        for(j = 1; j < csvData[0].length; j++) {
            sum += Math.pow(parseFloat(csvData[i][j])-sample[j-1],2);
        }
        // push to data container
        delta[parseFloat(csvData[i][0])] = Math.sqrt(sum);
    }
    // find the minimum value
    var min = 1000;
    for(var key in delta) {
        if (delta[key] < min) {
            min = delta[key];
            nbr = key;
        }
    }
    console.log(nbr);
    return nbr;
}


// --------- BEGIN - HANDLE RSSI VALUES FROM THE NODES ---------- //

// Instantiate the beacon data array
var beacon_data = {};
var bd_length = Object.keys(beacon_data).length;

// Reset the beacon data after prediction made
var resetBeaconData = function() {
  beacon_data = {};
  bd_length = 0;
};

// When the Coordinator Xbee receives the RSSI values, gather them, and make the bin prediction
XBeeAPI.on("frame_object", function(frame) {
  if (frame.type == 144){
    console.log("Beacon ID: " + frame.data[1] + ", RSSI: " + (frame.data[0]));
    beacon_data[frame.data[1]] = frame.data[0];
    bd_length = Object.keys(beacon_data).length;
    console.log(bd_length);

    if(bd_length >= 4){
      var data_to_send = [beacon_data['1'], beacon_data['2'], beacon_data['3'], beacon_data['4']];
      // Predict the bin based off the data
      var pos_prediction = predict(data_to_send);
      bin_history.push(pos_prediction);

      // Reset beacon data
      resetBeaconData();
    }
  }
});


// ------------ END - HANDLE RSSI VALUES FROM THE NODES ------------ //





// --------- DEFINE AJAX POST REQUESTS HERE --------- //

// For getting the most recent location of the moving device
app.get('/get_location', function(req, res){
	// Send the current bin_id back to the view
  var position = bin_history[bin_history.length - 1];
  if (position < 53 && position > 48) {
    updateSafeTurn(0);
  }
  else {
    updateSafeTurn(1);
  }
	res.send(position);
});

// For starting/stopping
app.get('/start_stop_crawler', function(req, res){
  updateStartStop(start_stop_pin.readSync() === 0 ? 1 : 0); //TODO! 
	res.send("1");

});

// For going left
app.get('/turn_left', function(req, res){
	res.send("1");

});

// For going right
app.get('/turn_right', function(req, res){
	res.send("1");
});

