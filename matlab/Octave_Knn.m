% Test script to run Knn in Octave
load('model.mat');
sample = [40,50,60,70];
our_position = predict(Mdl,sample);