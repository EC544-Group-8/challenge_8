// Wait til page load before running JS
$(document).ready(function () {
    // declare the buttons as variables from the index.html page
    var floor_plan = $("#floor_plan");
    var updateInterval = 1000;
    var fake_bin_id = 0;

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

    // update location after specified time. 
    setInterval(function(){update_location();}, updateInterval);


});