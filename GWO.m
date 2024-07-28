function [Route] = GWO(Start, End)

%% Problem Definition
global Options;
global Vehicles;
nVar = Options.NumVehicles; % Number of variables (nodes)
nWolf = Options.nPop;       % Number of wolves (population size)
MaxIt = Options.NumIterations; % Maximum number of iterations

%% GWO Parameters
alpha = 0.5; % Weight for the leader wolf
beta = 0.3;  % Weight for the second leader wolf
delta = 0.2; % Weight for the third leader wolf

%% Initialization
p = [Vehicles.Position];
p = [p(1:2:end); p(2:2:end)]';
D = pdist2(p, p);

% Empty wolf structure
empty_wolf.Position = [];
empty_wolf.Cost = [];
empty_wolf.PCs = [];
empty_wolf.PDRs = [];
empty_wolf.Ds = [];
empty_wolf.Dvs = [];

% Wolf pack
wolves = repmat(empty_wolf, nWolf, 1);

% Initialize wolves
for i = 1:nWolf
    wolves(i).Position = Start;
    wolves(i).Route = [];
    wolves(i).Cost = inf;
    wolves(i).PCs = [];
    wolves(i).PDRs = [];
    wolves(i).Ds = [];
    wolves(i).Dvs = [];
end

% Leader wolves
Alpha = empty_wolf;
Beta = empty_wolf;
Delta = empty_wolf;
Alpha.Cost = -inf;
Beta.Cost = -inf;
Delta.Cost = -inf;

%% GWO Main Loop

for it = 1:MaxIt
    
    % Move Wolves
    for k = 1:nWolf
        wolves(k).Route = Start;
        Current = Start;
        tabu = [];
        while (Current ~= End)
            if isempty(tabu)
                next_node = randperm(nVar, 1);
            else
                candidates = setdiff(1:nVar, [wolves(k).Route, tabu]);
                if isempty(candidates)
                    tabu = [tabu wolves(k).Route(end)];
                    wolves(k).Route(end) = [];
                    Current = wolves(k).Route(end);
                    if isempty(wolves(k).Route)
                        disp(['Cannot find route between ' num2str(Start) ' and ' num2str(End)]);
                        Route = [];
                        return;
                    end
                    continue;
                end
                next_node = candidates(randperm(length(candidates), 1));
            end
            wolves(k).Route = [wolves(k).Route next_node];
            Current = next_node;
        end
        
        [wolves(k).PCs, wolves(k).PDRs, wolves(k).Ds, wolves(k).Dvs, wolves(k).Cost] = CostFunctionGWO(wolves(k).Route);
        
        % Update the leader wolves
        if wolves(k).Cost > Alpha.Cost
            Delta = Beta;
            Beta = Alpha;
            Alpha = wolves(k);
        elseif wolves(k).Cost > Beta.Cost
            Delta = Beta;
            Beta = wolves(k);
        elseif wolves(k).Cost > Delta.Cost
            Delta = wolves(k);
        end
    end
    
    % Update positions of wolves
    for k = 1:nWolf
        for j = 1:length(wolves(k).Route)
            A1 = 2 * alpha * rand() - alpha;
            C1 = 2 * rand();
            D_alpha = abs(C1 * Alpha.Position - wolves(k).Position);
            X1 = Alpha.Position - A1 * D_alpha;
            
            A2 = 2 * beta * rand() - beta;
            C2 = 2 * rand();
            D_beta = abs(C2 * Beta.Position - wolves(k).Position);
            X2 = Beta.Position - A2 * D_beta;
            
            A3 = 2 * delta * rand() - delta;
            C3 = 2 * rand();
            D_delta = abs(C3 * Delta.Position - wolves(k).Position);
            X3 = Delta.Position - A3 * D_delta;
            
            wolves(k).Position = (X1 + X2 + X3) / 3;
        end
    end
    
end

Route = Alpha.Route;

end
