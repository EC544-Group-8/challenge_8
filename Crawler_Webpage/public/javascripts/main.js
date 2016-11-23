// Wait til page load before running JS
$(document).ready(function () {
    // declare the buttons as variables from the index.html page
    var run = $("#run");
    var stop = $("#stop");

    // Function to change the status of the particle
    function change_motion(event) {
        // Go to the route on the server that is designed to change the motion on the particle
        console.log('Trying to tell the particle to run');
        var parameters = {motion_id : event.data.state};
        $.get('/change_motion', parameters, function(data) {
            // if(data > 0) {
            //     console.log('particle ' + parameters.motion_id + ' worked');
            // } else {
            //     console.log('particle ' + parameters.motion_id + ' did NOT work');
            // }
        });
    }

    // On buttons, when clicked, will run the change_motion function
    run.click({state: 1}, change_motion);
    stop.click({state: 0}, change_motion);


});