function [terrainGrid,costGrid] = terrainGen(gridSize)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    gridSize
end

arguments (Output)
    terrainGrid
    costGrid
end

clear PerlinNoise2D; % Clears the persistent variable 'p' from the PerlinNoise2D function
fprintf('Starting terrain generation script...\n');

% Seed the random number generator to ensure different terrains each time
rng('shuffle');

terrainTypes = {
    struct('name', 'Grass',  'color', [0.3 0.6 0.2], 'cost', 1.0, 'climbFactor', 1.5, 'descentFactor', 0.8, 'climbSteepnessFactor', 0.1, 'descentGentlenessFactor', 0.05);
    struct('name', 'Sand',   'color', [0.9 0.8 0.5], 'cost', 2.0, 'climbFactor', 2.0, 'descentFactor', 1.0, 'climbSteepnessFactor', 0.2, 'descentGentlenessFactor', 0.1);
    struct('name', 'Mud',    'color', [0.5 0.3 0.1], 'cost', 3.0, 'climbFactor', 2.5, 'descentFactor', 1.2, 'climbSteepnessFactor', 0.3, 'descentGentlenessFactor', 0.15);
    struct('name', 'Road',   'color', [0.6 0.6 0.6], 'cost', 0.5, 'climbFactor', 1.2, 'descentFactor', 0.7, 'climbSteepnessFactor', 0.05, 'descentGentlenessFactor', 0.02);
    struct('name', 'Water',  'color', [0.2 0.4 0.8], 'cost', inf, 'climbFactor', inf, 'descentFactor', inf, 'climbSteepnessFactor', inf, 'descentGentlenessFactor', inf); % Impassable
    struct('name', 'Forest', 'color', [0.1 0.4 0.1], 'cost', 2.5, 'climbFactor', 2.0, 'descentFactor', 1.0, 'climbSteepnessFactor', 0.25, 'descentGentlenessFactor', 0.1);
};
numTerrainTypes = length(terrainTypes);
terrainGrid = zeros(gridSize, gridSize);

% --- Initial Random Terrain Generation ---
% Assign a random terrain type to each cell in the grid
for i = 1:gridSize
    for j = 1:gridSize
        terrainGrid(i, j) = randi(numTerrainTypes);
    end
end

% --- Terrain Smoothing (to create grouped regions) ---
numSmoothingIterations = 5;
for iter = 1:numSmoothingIterations
    newTerrainGrid = terrainGrid; % Create a copy to store updates for this iteration
    for i = 1:gridSize
        for j = 1:gridSize
            neighbors = [];
            % Check 8-directional neighbors
            for dx = -1:1
                for dy = -1:1
                    if dx == 0 && dy == 0
                        continue; % Skip the current cell itself
                    end
                    ni = i + dx;
                    nj = j + dy;
                    % Check if neighbor is within grid bounds
                    if ni >= 1 && ni <= gridSize && nj >= 1 && nj <= gridSize
                        neighbors = [neighbors, terrainGrid(ni, nj)];
                    end
                end
            end
            % If the cell has neighbors, find the most frequent terrain type among them
            if ~isempty(neighbors)
                uniqueNeighbors = unique(neighbors);
                counts = zeros(1, length(uniqueNeighbors));
                for k = 1:length(uniqueNeighbors)
                    counts(k) = sum(neighbors == uniqueNeighbors(k));
                end
                [~, maxIdx] = max(counts);
                mostFrequentTerrain = uniqueNeighbors(maxIdx);
                newTerrainGrid(i, j) = mostFrequentTerrain;
            end
        end
    end
    terrainGrid = newTerrainGrid; % Update the main grid for the next iteration
end
costGrid = zeros(gridSize,gridSize);
% --- Cost Calculation based on Terrain Types ---
for i = 1:gridSize
    for j = 1:gridSize
        terrainTypeIndex = terrainGrid(i, j);
        costGrid(i, j) = terrainTypes{terrainTypeIndex}.cost;
    end
end

end