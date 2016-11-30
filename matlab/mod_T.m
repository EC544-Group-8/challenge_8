% this script will modify the raw table T which is contained within
% indoor_localization for the purpose of re-defining our bins down to just
% 8 locations...this whill not modify the original script and it will not
% modify the original database T

clear;clc;

Indoor_Localization;
Idx = find(T.Location == 54);