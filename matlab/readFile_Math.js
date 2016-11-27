/*  
    This can be used to read in a csv file containing our knn
    training set which we can use to predict a location from. In 
    this case we would have to write it from scratch (already done
    for K-1) as opposed to using matlabs model. 
*/

// global variables
var fs = require('fs'); 
var parse = require('csv-parse');
var csvData=[];
var historic = [];

// csv file reader
fs.createReadStream('DB_Small.txt')
    .pipe(parse({delimiter: ','}))
    .on('data', function(csvrow) {
        csvData.push(csvrow);
        //console.log(csvrow);
    })
    .on('end',function() {
      POS = predict([45,77,79,47]); // remove this for implementation 
      storeLocations(POS);
    });

// predict the nth neighbors
function predict(sample) {
    var delta = {};
    var nbr = -1;
    // loop through the matrix
    for (i = 0; i < csvData.length; i++) {
        var sum = 0;
        for(j = 1; j < csvData[0].length; j++) {
            sum += Math.abs(parseFloat(csvData[i][j])-sample[j-1]);
        }
        // push to data container
        delta[parseFloat(csvData[i][0])] = sum;
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


function storeLocations(current_pos) {
    // add to past locations
    var trusted_pos = -1;
    historic.push(current_pos);
    var pos_sum = 0;
    var delta = 0;
    for (i = (historic.length-5); i < historic.length; i++) {
        pos_sum += historic[i];
        if (i != (historic.length-1)) {
            delta += Math.abs(historic[i+1] - historic[i]);
        }
    }
    var avg_pos = Math.round(pos_sum/5);
    var avg_delta = Math.round(delta/4);

    // create algorithm to use this data and decide if we trust 
    // the current position estimation 
    return trusted_pos;
}



