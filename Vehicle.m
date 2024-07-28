classdef Vehicle < handle
    %CLSAA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        ID = 0;
        Step = 1;
        Position = int16([1 2]);
        Neighbours;
        MoveSteps;
    end
    methods(Static)
        function [d] = Delay(Route)
            global Options;
            d = 0;
            for i=1:numel(Route)-1
                d = d + Options.Delay(Route(i),Route(i+1));
            end
        end
        
        function L = Length(Route)           
            global Vehicles;
            p = [Vehicles.Position];
            p = [p(1:2:end); p(2:2:end)]';
            D = pdist2(p,p);
            L=0;
            for i=1:numel(Route)-1
                L=L+D(Route(i),Route(i+1));
            end
        end
        
        function [v] = IsValid(Route)
            
            global Options;
            v = true;
            if(isempty(Route))
                v = false;
                return;
            end
            for i=1:numel(Route)-1
                if(rand > Options.Connectivity(Route(i),Route(i+1)) || ...
                        rand > Options.Reliability(Route(i),Route(i+1)))
                    v = false;
                    return;
                end
            end
        end
    end
    
    methods
        function obj = Vehicle(id, pos)
            global Options;
            obj.ID = id;
            obj.Position = pos;
            obj.Neighbours = zeros(1,Options.NumVehicles);
            obj.MoveSteps = GetMoveSteps(obj.Position,[randi(Options.Width) randi(Options.Heigh)]);
        end
        
        function Move(obj)
            global Options;
            while(obj.Step > size(obj.MoveSteps,2))
                newPos = [randi(Options.Width) randi(Options.Heigh)];
                while(isequal(newPos,obj.Position))
                    newPos = [randi(Options.Width) randi(Options.Heigh)];
                end
                obj.MoveSteps = GetMoveSteps(obj.Position,newPos);
                obj.Step = 1;
            end
            %fprintf('%d\n',obj.ID);
            obj.Position(1) = obj.MoveSteps(1,obj.Step);
            obj.Position(2) = obj.MoveSteps(2,obj.Step);
            obj.Step = obj.Step + 1;
        end
        
        function CalcNeighbors(obj,Vehicles)
            global Options;
            obj.Neighbours = zeros(1,Options.NumVehicles);
            for i=1:Options.NumVehicles
                if(i ~= obj.ID)
                    X1=obj.Position(1,1);
                    Y1=obj.Position(1,2);
                    X2=Vehicles(i).Position(1,1);
                    Y2=Vehicles(i).Position(1,2);
                    xSide=abs(X2-X1);
                    ySide=abs(Y2-Y1);
                    d=sqrt(xSide^2+ySide^2);
                    if (d<Options.CommunicatioRange)
                        obj.Neighbours(1,i) = 1;
                    end
                end
            end
        end
        
        function Routing(obj,Dest)
            global Options;
            if (Options.Method == "ACO")
                [Route] = ACO(obj.ID, Dest);
                if(obj.IsValid(Route))
                    Options.H = Options.H + numel(Route);
                    Options.L = Options.L + obj.Length(Route);
                    Options.D = Options.D + obj.Delay(Route);
                    Options.S = Options.S + 1;
                end

            elseif (Options.Method == "GWO")
                [Route] = GWO(obj.ID, Dest);
                if(obj.IsValid(Route))
                    Options.H = Options.H + numel(Route);
                    Options.L = Options.L + obj.Length(Route);
                    Options.D = Options.D + obj.Delay(Route);
                    Options.S = Options.S + 1;
                end

            else
                [Route] = MPSO(obj.ID, Dest);
                if(obj.IsValid(Route))
                    Options.H = Options.H + numel(Route);
                    Options.L = Options.L + obj.Length(Route);
                    Options.D = Options.D + obj.Delay(Route);
                    Options.S = Options.S + 1;
                end
            end
        end
    end
end
