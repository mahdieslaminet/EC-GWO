 function [Route] = MPSO(Start, End)

%% Problem Definition
global Options;
global Vehicles;
nVar = Options.NumVehicles;
VarSize = [1 nVar];

%% PSO Parameters
VarMin= 1;         % Lower Bound of Variables
VarMax= nVar;        % Upper Bound of Variables


% Velocity Limits
VelMax=0.1*(VarMax-VarMin);
VelMin=-VelMax;

%% Initialization

empty_particle.Position=[];
empty_particle.Costs=[];
empty_particle.Velocity=[];
empty_particle.PBest.Position=[];
empty_particle.PBest.Costs=[];

Particles=repmat(empty_particle,Options.nPop,1);

p = [Vehicles.Position];
p = [p(1:2:end); p(2:2:end)]';
D = pdist2(p,p);
C = kmeans(p,Options.ClusterCount);

% Initialize Position
for i=1:Options.nPop
    Particles(i).Position=Start;
    Current = Start;
    tabu = [];
    while(Current ~= End)
        
        P=D(Current,:);
        P(Particles(i).Position)=0;
        P(Vehicles(Current).Neighbours == 0) = 0;
        P(tabu) = 0;
        if(sum(P) == 0)
            tabu = [tabu Particles(i).Position(end)]; %#ok
            Particles(i).Position(end) = [];
            if(~isempty(Particles(i).Position))
                Current = Particles(i).Position(end);
            else
                disp(['Can not find route between ' num2str(Start) ' and ' num2str(End)]);
                Cost = -1;
                return;
            end
            continue;
        end
        P = 1./P;
        P(P == inf) = 0;
        P=P/sum(P);
        
        Current=RouletteWheelSelection(P);
        
        Particles(i).Position=[Particles(i).Position Current];
        
    end
    
    % Initialize Velocity
    Particles(i).Velocity = zeros(size(Particles(i).Position));
    
    % Evaluation
    Particles(i).Costs = CostFunctionMPSO(Particles(i).Position, D, C);
    
    % Update Personal Best
    Particles(i).PBest.Position=Particles(i).Position;
    Particles(i).PBest.Costs=Particles(i).Costs;
end

Costs = [Particles.Costs];
DOMINATED= CheckDomination([Costs(1:3:end)' Costs(2:3:end)' Costs(3:3:end)']);
REP  = Particles(~DOMINATED);

% Main MPSO loop
for it=1:Options.NumIterations
    for i=1:Options.nPop
        % Select leader
        leader = REP(randi(size(REP,1)));
        
        % Update speeds and positions
        % Update Velocity
        Particles(i).Velocity = Options.iw*Particles(i).Velocity;
        s = numel(Particles(i).Position);
        if(numel(Particles(i).PBest.Position) == numel(Particles(i).Position))
            Particles(i).Velocity = Particles(i).Velocity + ...
                Options.c1*rand([1 s]).*(Particles(i).PBest.Position-Particles(i).Position);
        elseif(numel(Particles(i).PBest.Position) > numel(Particles(i).Position))
            Particles(i).Velocity = Particles(i).Velocity + ...
                Options.c1*rand([1 s]).*(Particles(i).PBest.Position(1:s)-Particles(i).Position);
        else
            b = zeros([1 s]);
            b(1:numel(Particles(i).PBest.Position)) = Particles(i).PBest.Position(1:end-1);
            b(end) = Particles(i).PBest.Position(end);
            Particles(i).Velocity = Particles(i).Velocity + ...
                Options.c1*rand([1 s]).*(b-Particles(i).Position(1:s));
        end
        
        if (numel(leader.Position) == numel(Particles(i).Position))
            Particles(i).Velocity = Particles(i).Velocity + ...
                Options.c2*rand([1 s]).*(leader.Position-Particles(i).Position);
        elseif(numel(leader.Position) > numel(Particles(i).Position))
            Particles(i).Velocity = Particles(i).Velocity + ...
                Options.c2*rand([1 s]).*(leader.Position(1:s)-Particles(i).Position);
        else
            b = zeros(size(Particles(i).Position));
            b(1:numel(leader.Position)-1) = leader.Position(1:end-1);
            b(end) = leader.Position(end);
            Particles(i).Velocity = Particles(i).Velocity + ...
                Options.c1*rand([1 s]).*(b-Particles(i).Position(1:s));
        end
        
        % Apply Velocity Limits
        Particles(i).Velocity = max(Particles(i).Velocity,VelMin);
        Particles(i).Velocity = min(Particles(i).Velocity,VelMax);
        
        
        % Update Position
        v=2;

        while(v<numel(Particles(i).Velocity)-1)
            if(Particles(i).Velocity(v) > mean([VelMin VelMax])/4)
                m = -1;
                for k=v+1:size(Particles(i).Position)
                    if(IsNeighbour(Particles(i).Position(v-1),Particles(i).Position(k)))
                        m = k;
                    end
                end
                if(m ~= -1)
                    Particles(i).Position(v:m-1) = [];
                    Particles(i).Velocity(v:m-1) = [];
                else
                    nv = find(Vehicles(Particles(i).Position(v-1)).Neighbours == 1);
                    nvp = find(Vehicles(Particles(i).Position(v+1)).Neighbours == 1);
                    nv = intersect(nv,nvp);
                    if(~isempty(nv))
                        Particles(i).Position(v) = nv(randi(numel(nv)));
                    end
                end
            end
            v = v+1;
        end
       
        
        
        % Evaluation
        Particles(i).Costs = CostFunctionMPSO(Particles(i).Position, D, C);
        
        % Update PBest for each Particles
        if(Particles(i).Costs(1) < Particles(i).PBest.Costs(1) && ...
                Particles(i).Costs(2) < Particles(i).PBest.Costs(2) && ...
                Particles(i).Costs(3) < Particles(i).PBest.Costs(3))
            Particles(i).PBest.Position=Particles(i).Position;
            Particles(i).PBest.Costs=Particles(i).Costs;
        elseif (Particles(i).Costs(1) > Particles(i).PBest.Costs(1) && ...
                Particles(i).Costs(2) > Particles(i).PBest.Costs(2) && ...
                Particles(i).Costs(3) > Particles(i).PBest.Costs(3))
            continue;
        else
            if(rand()<0.5)
                Particles(i).PBest.Position=Particles(i).Position;
                Particles(i).PBest.Costs=Particles(i).Costs;
            end
        end
    end
    % Update the repository
    REP = updateRepository(REP,Particles);
end
Costs = [REP.Costs];
UtopiaPoint = REP(GetUtopia([Costs(1:3:end)' Costs(2:3:end)' Costs(3:3:end)']));
Route = UtopiaPoint.Position;
%Cost = UtopiaPoint.Costs;
end

% Function that updates the repository given a new population and its
% fitness
function REP = updateRepository(REP,Particles)
% Domination between Particles
Costs = [Particles.Costs];
DOMINATED= CheckDomination([Costs(1:3:end)' Costs(2:3:end)' Costs(3:3:end)']);
REP  = [REP; Particles(~DOMINATED)];

% Domination between nondominated Particles and the last repository
Costs = [REP.Costs];
DOMINATED= CheckDomination([Costs(1:3:end)' Costs(2:3:end)' Costs(3:3:end)']);
REP  = REP(~DOMINATED);
end

% Function for checking the domination between the population. It
% returns a vector that indicates if each Particles is dominated (1) or not
function dom_vector = CheckDomination(fitness)
Np = size(fitness,1);
dom_vector = zeros(Np,1);
all_perm = nchoosek(1:Np,2);    % Possible permutations
all_perm = [all_perm; [all_perm(:,2) all_perm(:,1)]];

d = dominates(fitness(all_perm(:,1),:),fitness(all_perm(:,2),:));
dominated_Particles = unique(all_perm(d==1,2));
dom_vector(dominated_Particles) = 1;
end

% Function that returns 1 if x dominates y and 0 otherwise
function d = dominates(x,y)
d = all(x<=y,2) & any(x<y,2);
end

function [n] = IsNeighbour(v,w)
global Vehicles;
if(Vehicles(v).Neighbours(w))
    n = true;
else
    n = false;
end
end
