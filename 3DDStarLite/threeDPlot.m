clear;
clc;
close all;

gridSize = 30;
genTerrain = true;
randomST = true;


if genTerrain
    [terrainGrid, costGrid] = terrainGen(gridSize);
    altitudeGrid = altitudeGen(gridSize);
    
    save('terrain_data.mat', 'altitudeGrid','terrainGrid',"costGrid");
end
fprintf('Loading pre-generated terrain data.\n');
load("terrain_data.mat","altitudeGrid","terrainGrid","costGrid");


% --- Visualize Altitude Map ---
visTerrain(terrainGrid);
visAlti(altitudeGrid,terrainGrid);
%visTerrain(terrainGrid);

%% Map
Map.lim = gridSize^2;
Map.xMin = 1;
Map.xMax = 30;
Map.yMin = 1;
Map.yMax = 30;
Map.nX = Map.xMax - Map.xMin + 1;
Map.nY = Map.yMax - Map.yMin + 1;

%% robot data
% dir: direction
Robot.dir = deg2rad(90); %randsample([0 90 180 270], 1);

% start & goal - coordinates
if randomST
    Robot.xs = randi([Map.xMin Map.xMax],1);
    Robot.ys = randi([Map.yMin Map.yMax],1);
    Robot.xt = randi([Map.xMin Map.xMax],1);
    Robot.yt = randi([Map.yMin Map.yMax],1);
else
    Robot.xs = 1;
    Robot.ys = 1;
    Robot.xt = 30;
    Robot.yt = 30;
end


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
xD = [13, 13];
yD = [12, 10];

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

plot3(xS, yS, zS,'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'w');

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
Model.expandMethod = 'heading'; % random or heading
Model.distType = 'euclidean'; % euclidean or manhattan;
Model.adjType = '8adj'; % 4adj or 8adj
Model.prioType = 'balanced'; %speed, eff, balanced

Model = create3DModel(Model);
Model = new3DObstacles(Model,xD,yD);
tic
[Model, Path] = my3DDstarLite(Model);
Sol = Path;
Sol.runTime = toc;
Sol.cost = calCostL(Sol.coords);
Sol.smoothness = calSmoothnessbyDir(Sol);

%% display data and plot solution
disp(['run time for path= ' num2str(Sol.runTime)])
disp(Sol)

%% clear temporal data
clear adj_type dist_type
