% measurements
% take T and make a list of measurments with 1 sample from each beacon

% species
% for each corresponding row in T, this lists the location

[r,c] = size(T);
meas = [0,0,0,0];
species = [0];
new_row = [0 0 0 0];

for i = 1:r
   row = T(i,:);
   location = row{:,1};
   beacon = row{:,2};
   RSSI = row{:,3};
   
   % add reading to corresponding column of new row
   new_row(:,beacon) = RSSI;
   if(new_row > 0) % if all elements are non-zero
       % push to measurements and and reset new row
       meas = [meas;new_row];
       species = [species;location]; % need to make sure all have 4? TODO
       new_row = [0 0 0 0];
   end

end

% clean up first rows
meas(1,:)=[]
species(1,:)=[]

% make knn model
Mdl = fitcknn(meas,species,'NumNeighbors',5)

% predict a value
m1 = 78;
m2 = 54;
m3 = 49;
m4 = 62;
m = [m1,m2,m3,m4];
bin = predict(Mdl,m)