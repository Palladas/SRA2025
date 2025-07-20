function [Model, Sol] = threeDDStar(gridSize, displayPlot, genTerrain, randomST, dynamicObst)
%THREEDDSTAR Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    gridSize
    displayPlot
    genTerrain
    randomST
    dynamicObst
end

arguments (Output)
    Model
    Sol
end



if genTerrain
    [terrainGrid, costGrid] = terrainGen(gridSize);
    altitudeGrid = altitudeGen(gridSize);
    
    save('terrain_data.mat', 'altitudeGrid','terrainGrid',"costGrid");
end
fprintf('Loading pre-generated terrain data.\n');
load("terrain_data.mat","altitudeGrid","terrainGrid","costGrid");


%% Map
Map.lim = gridSize^2;
Map.xMin = 1;
Map.xMax = gridSize;
Map.yMin = 1;
Map.yMax = gridSize;
Map.nX = Map.xMax - Map.xMin + 1;
Map.nY = Map.yMax - Map.yMin + 1;

%% robot data
% dir: direction
Robot.dir = deg2rad(90); %randsample([0 90 180 270], 1);

% start & goal - coordinates
if randomST
    xs = randi([Map.xMin Map.xMax],1);
    ys = randi([Map.yMin Map.yMax],1);
    xt = randi([Map.xMin Map.xMax],1);
    yt = randi([Map.yMin Map.yMax],1);
    save("robot_data.mat","xs","ys","xt","yt");
end
load("robot_data.mat","xs","ys","xt","yt");
Robot.xs = xs;
Robot.ys = ys;
Robot.xt = xt;
Robot.yt = yt;


%  start & goal - node numbers
Robot.startNode = (Robot.ys - Map.yMin) * Map.nX + Robot.xs - Map.xMin + 1;
Robot.targetNode = (Robot.yt - Map.yMin) * Map.nX + Robot.xt - Map.xMin + 1;

%% Nodes & Adj
k = 1;
adj = cell(1, 1);
for j = 1:Map.yMax
    for i = 1:Map.xMax
        adj{k, 1} = k; % node number
        adj{k, 2} = [i, j]; % node coordinates
        Nodes.cord(1:2, k) = [i, j]'; % node coordinates
        Nodes.number(1, k) = k; % node number
        if i == Robot.xs && j == Robot.ys
            startNode = k; % start node number
        elseif i == Robot.xt && j == Robot.yt
            targetNode = k; % target (final) node number
        end

        k = k + 1;
   end
end

Nodes.count = numel(Nodes.number);

%generate obstacles
xS = [];
yS = [];
xD = [];
yD = [];

Obst.r = 0.25;

Obst.x = xS;
Obst.y = yS;
Obst.count = length(Obst.x);
zS = zeros(length(xS));
for i = 1:length(xS)
    zS(i) = altitudeGrid(yS(i),xS(i));
end

% obstacle node numbers
Obst.number = zeros(1, Obst.count);

for ix = 1:Obst.count
    Obst.nodeNumber(ix) = (Obst.y(ix) - Map.yMin) * Map.nX + Obst.x(ix) - Map.xMin + 1;
end

if displayPlot
    visTerrain(terrainGrid);
    visAlti(altitudeGrid,terrainGrid);
    plot3(xS, yS, zS,'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'w');
end
%% update model
Model.Nodes = Nodes;
Model.Robot = Robot;
Model.Map = Map;
Model.Alti = altitudeGrid;
Model.Terr = terrainGrid;
Model.TerrCost = costGrid;
Model.Obsts = Obst;

% Initialize the robot's position on the altitude grid
Robot.altitude = altitudeGrid(Robot.ys, Robot.xs);
fprintf('Robot initialized at position (%d, %d) with altitude %.2f.\n', Robot.xs, Robot.ys, Robot.altitude);

%-------Run D* Lite--------

%% settings
Model.expandMethod = 'random'; % random or heading
Model.distType = 'euclidean'; % euclidean or manhattan;
Model.adjType = '8adj'; % 4adj or 8adj
Model.prioType = 'balanced'; %speed, eff, balanced

Model = create3DModel(Model);
Model = new3DObstacles(Model,xD,yD);
Model.dObstCount = dynamicObst;
tic
[Model, Path] = my3DDstarLite(Model, displayPlot);
if ~isfield(Model, 'distType')
    Sol = -1;
    return
end
Sol = Path;
Sol.runTime = toc;
Sol.ticks = Model.time;
Sol.cost = calCostL(Sol.coords);
Sol.smoothness = calSmoothnessbyDir(Sol);
Sol.nodesVisited = Model.nodesVisited;

%% display data and plot solution
disp(['run time for path= ' num2str(Sol.runTime)])
disp(Sol)

%% clear temporal data
clear adj_type dist_type

end