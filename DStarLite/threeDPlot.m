clear;
clc;
close all;

gridSize = 30;
genTerrain = false;

if genTerrain
    fprintf('Generating altitude grid using Perlin noise...\n');
    % Parameters for Perlin Noise
    scale = 0.02; % Controls "zoom" level of noise
    octaves = 4; % Number of noise layers
    persistence = 0.5; % How much each octave contributes
    lacunarity = 2.0; % How much larger each octave is
    
    altitudeGrid = zeros(gridSize, gridSize);
    for i = 1:gridSize
        for j = 1:gridSize
            x = (j - 1) * scale; % Map grid coords to noise coords
            y = (i - 1) * scale;
            
            noise_value = 0;
            amplitude = 1;
            frequency = 1;
            
            for o = 1:octaves
                % MATLAB's perlin noise is typically 3D, we'll use 2D by fixing z
                noise_value = noise_value + PerlinNoise2D(x * frequency, y * frequency) * amplitude;
                amplitude = amplitude * persistence;
                frequency = frequency * lacunarity;
            end
            altitudeGrid(i, j) = noise_value;
        end
    end
    
    % Normalize and scale altitude values (e.g., to a range of 0 to 100)
    minAlt = min(altitudeGrid(:));
    maxAlt = max(altitudeGrid(:));
    altitudeGrid = (altitudeGrid - minAlt) / (maxAlt - minAlt) * 20; % Scale to 0-100 for better visualization and cost impact
    
    fprintf('Altitude grid generated.\n');
    
    save('terrain_data.mat', 'altitudeGrid');
end
fprintf('Loading pre-generated terrain data.\n');
load("terrain_data.mat","altitudeGrid");
% --- Visualize Altitude Map ---
figure('Name', 'Generated Altitude Map', 'NumberTitle', 'off', 'Color', [0.95 0.95 0.95]);
surf(altitudeGrid)
hold on;
imagesc(altitudeGrid);
colormap('gray'); % Or 'jet', 'gray', etc. for elevation
colorbar;
xlim([0.5, gridSize + 0.5]); % Adjust limits for imagesc
ylim([0.5, gridSize + 0.5]); % Adjust limits for imagesc
set(gca, 'YDir', 'normal');
title('Generated Altitude Map (Z-values)');
xlabel('X-coordinate');
ylabel('Y-coordinate');
fprintf('Altitude map visualized.\n');



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
Robot.xs = 19;
Robot.ys = 23;
Robot.xt = 10;
Robot.yt = 7;

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

%% update model
Model.Nodes = Nodes;
Model.Robot = Robot;
Model.Map = Map;
Model.Alti = altitudeGrid;

% Initialize the robot's position on the altitude grid
Robot.altitude = altitudeGrid(Robot.ys, Robot.xs);
fprintf('Robot initialized at position (%d, %d) with altitude %.2f.\n', Robot.xs, Robot.ys, Robot.altitude);

%-------Run D* Lite--------

%% settings
Model.expandMethod = 'heading'; % random or heading
Model.distType = 'euclidean'; % euclidean or manhattan;
Model.adjType = '8adj'; % 4adj or 8adj

Model = create3DModel(Model);

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
