% Indoor tracking script

% Generate the path to the file
%addpath(genpath('/Users/connormccann/Documents/BU/Fall_2016/EC544/EC544_demos/challenge_6/database'))
%addpath(genpath('../database'))

format shortg

% Read the CSV into a table
T = readtable('../database/beacon_rssi_data.txt','Delimiter',',','ReadVariableNames',false);

% Change the variable (column) names
T.Properties.VariableNames = {'Location','Beacon','RSSI'};

% Generate the avg table
uniqueLocations = unique(T.Location);
uniqueBeacons = unique(T.Beacon);
dataVec = zeros(1,4);

% Pre-allocate the table
avgDB = table();
avgDB.Location = zeros(length(uniqueLocations),1);
avgDB.B1 = zeros(length(uniqueLocations),1);
avgDB.B2 = zeros(length(uniqueLocations),1);
avgDB.B3 = zeros(length(uniqueLocations),1);
avgDB.B4 = zeros(length(uniqueLocations),1);

for i = 1:length(uniqueLocations)
    for j = 1:length(uniqueBeacons)
        dataVec(j) = floor(100*mean(T.RSSI(T.Location == i & T.Beacon == j)))/100;
    end
    avgDB.Location(i) = i';
    avgDB.B1(i) = dataVec(1);
    avgDB.B2(i) = dataVec(2);
    avgDB.B3(i) = dataVec(3);
    avgDB.B4(i) = dataVec(4);
end

% save table to excel sheet
writetable(avgDB,'avgDB.txt','Delimiter',' ')

% sample
sample = [69,37,53,82];

tic
% vectorized knn position from average database 
[min_val,our_position] = min(sum((table2array(avgDB(:,2:5)) - repmat(sample,height(avgDB),1)).^2,2)');

% display the result of average database
disp(our_position)

% knn through the RAW database
min_total_E_dist = 10000;
for i = 1:length(uniqueLocations)
    total_E_dist = 0;
    for j = 1:4
        RSSI_vec = T.RSSI(T.Location == i & T.Beacon == j);
        E_dist = (sample(j) - RSSI_vec).^2;
        [min_E_dist,I] = min(E_dist);
        total_E_dist = total_E_dist + min_E_dist;
    end
    if(total_E_dist < min_total_E_dist)
        min_total_E_dist = total_E_dist;
        our_position = i;
    end
end

% display the result of the RAW database
disp(our_position)
toc









% knn through the RAW database
%{
tic
min_total_E_dist = 10000;
for i = 1:length(uniqueLocations)
    total_E_dist = 0;
    for j = 1:4
        RSSI_vec = T.RSSI(T.Location == i & T.Beacon == j);
        min_E_dist = 10000;
        for k = 1:length(RSSI_vec)
            E_dist = (sample(j) - RSSI_vec(k))^2;
            if(E_dist < min_E_dist)
                min_E_dist = E_dist;
            end
        end
        total_E_dist = total_E_dist + min_E_dist;
    end
    if(total_E_dist < min_total_E_dist)
        min_total_E_dist = total_E_dist;
        our_position = i;
    end
end

% display the result of the RAW database
disp(our_position)



% k-nn look up within average database
sample = [56,59,66,77];
E_vec = zeros(1,height(avgDB));
storeAsVec = false;
min_dist = 10000;
for i = 1:height(avgDB)
    E_dist = 0;
    for j = 1:4
        var = sprintf('B%d',j);
        E_dist = E_dist + (sample(j) - avgDB.(var)(i))^2;
    end
    if(storeAsVec)
        E_vec(i) = E_dist;
    elseif(E_dist < min_dist)
        min_dist = E_dist;
        our_position = i;
    end
end

if(storeAsVec)
    [M,I] = min(E_vec);

    % determine the actual position
    our_position = avgDB.Location(I);
       
end

disp(our_position)
toc
%}       




