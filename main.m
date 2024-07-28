clc;
clear;
close all

global Options;
global Vehicles;

Options.Method = 'MPSO'; %GWO MPSO 
Options.CommunicatioRange = 200;
Options.NumVehicles = 20;
Options.NumIterations = 30;
Options.NumSends = 1;
Options.nPop = 10;
Options.iw = 1;
Options.c1 = 2;
Options.c2 = 2;
Options.H = 0;
Options.L = 0;
Options.ClusterCount = 4;


Options.Width = 500;
Options.Heigh = 500;



Vehicles = CreateVehicles();
Show();

Options.H = 0;
Options.L = 0;
Options.S = 0;
Options.D = 0;

Options.Delay = RndBtw(0.1,2,Options.NumVehicles);
Options.Connectivity = RndBtw(0.98,1,Options.NumVehicles);
Options.Reliability = RndBtw(0.98,1,Options.NumVehicles);

tic
for it=1:Options.NumSends
    for i=1:Options.NumVehicles
        Vehicles(i).Move();
    end
    for i=1:Options.NumVehicles
        Vehicles(i).CalcNeighbors(Vehicles);
    end
    for i=1:Options.NumVehicles
        Vehicles(i).Routing(randi(Options.NumVehicles));
    end
    %Show();
end

format long g
fprintf('\n----------Results-----------\n');
toc
fprintf('Length %f\n',Options.L/Options.S);
fprintf('Hop Count %f\n',Options.H/Options.S);
fprintf('Successful %d\n',Options.S);
fprintf('Delay %f\n',Options.D/Options.S);


