var express = require('express');
var app = express();
var http = require('http').Server(app);

// --------- DEFINE Particle API setup HERE --------- //
var Particle = require('particle-api-js');
var particle = new Particle();
var token;

// Login to account
particle.login({username: 'losborne@bu.edu', password: 'ec544group8'}).then(
  function(data){
    console.log('API login call COMPLETED on promise resolve: ', data.body.access_token);
    // Set the token to the global variable
    token = data.body.access_token;

    // List devices, optional
    // var devicesPr = particle.listDevices({ auth: token });
    // devicesPr.then(
    //   function(devices){
    //     console.log('Devices: ', devices);

    //   },
    //   function(err) {
    //     console.log('List devices call failed: ', err);
    //   }
    // );

  },
  function(err) {
    console.log('API login call - promise FAIL: ', err);
  }
);

// --------- END Particle API setup  --------- //


app.use(express.static(__dirname + '/public'));

app.get('/', function(req, res){
  res.sendfile('index.html');
});


http.listen(3000, function(){
  console.log('listening on *:3000');
});


// --------- DEFINE AJAX POST REQUESTS HERE --------- //
// For changing turning the particle 
app.get('/change_motion', function(req, res){
  // This motion_id is in the parameters from ajax call in main.js
	var motion_id = String(req.query.motion_id);

	// if(motion_id == '1') {
	// 	console.log('sending RUN command to particle');
	// } else if(motion_id == '0') {
	// 	console.log('sending STOP command to particle');
	// } else {
	// 	console.log('Motion ID error: '+motion_id);
	// }
	// Send to particle
	var fnPr = particle.callFunction({
		deviceId: '430034000947353235303037',	// This is the ID for "jester_turtle"
		name: 'new_motion',						// This is the name of the function that is in the jester_turtle on-board code
		argument: motion_id,					// 1 = RUN, 0 = STOP
		auth: token
	});
	
	// fnPr.then(
	// function(data) {
	// 	console.log('Function called succesfully:', data);
	// }, function(err) {
	// 	console.log('An error occurred:', err);
	// });
	
	// Let the main.js know that the AJAX worked
	res.send('1');
});

