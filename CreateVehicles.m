function [ Vehicles ] = CreateVehicles(  )
%CREATENET Summary of this function goes here
%   Detailed explanation goes here
global Options;

Vehicles(1:Options.NumVehicles) = Vehicle(0,zeros(1,2));

for i=1:Options.NumVehicles
    Vehicles(i) = Vehicle(i,[randi(Options.Width) randi(Options.Heigh)]);
end

for i=1:Options.NumVehicles
    Vehicles(i).CalcNeighbors(Vehicles);
end
end

