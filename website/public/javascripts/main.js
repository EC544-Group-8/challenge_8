// Wait til page load before running JS
$(document).ready(function () {
    // declare the buttons as variables from the index.html page
    var floor_plan = $("#floor_plan");
    var updateInterval = 1000;
    var fake_bin_id = 0;
	var start_stop_btn = $("#start_stop");
	var left_btn = $("#left");
	var right_btn = $("#right");

    // Function to change the location
    function update_location(event) {
        // Go to the route on the server that is designed to change the motion on the particle
        // console.log('Trying to get current location');
        
        $.get('/get_location', function(bin_id) {
            console.log('bin id: '+bin_id);
            var new_image_path = "/images/bin"+bin_id+".png"; // This will be the bin id from matlab
            floor_plan.attr("src", new_image_path);
        });
    }

    function start_stop_crawler(event) {
        $.get('/start_stop_crawler', function(data) {
			console.log("Start or Stop");
        });
    }
	
    function turn_left(event) {
        $.get('/turn_left', function(data) {
			console.log("Turn Left");
        });
    }

    function turn_right(event) {
        $.get('/turn_right', function(data) {
			console.log("Turn Right");
        });
    }

    // update location after specified time. 
    setInterval(function(){update_location();}, updateInterval);

	start_stop_btn.click(start_stop_crawler);
	left_btn.click(turn_left);
	right_btn.click(turn_right);

});
