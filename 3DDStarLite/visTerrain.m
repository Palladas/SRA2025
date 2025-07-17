function terrainColor = visTerrain(terrainGrid)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    terrainGrid
end

arguments (Output)
    terrainColor
end

terrainTypes = {
    struct('name', 'Grass',  'color', [0.3 0.6 0.2], 'cost', 1.0, 'climbFactor', 1.5, 'descentFactor', 0.8, 'climbSteepnessFactor', 0.1, 'descentGentlenessFactor', 0.05);
    struct('name', 'Sand',   'color', [0.9 0.8 0.5], 'cost', 2.0, 'climbFactor', 2.0, 'descentFactor', 1.0, 'climbSteepnessFactor', 0.2, 'descentGentlenessFactor', 0.1);
    struct('name', 'Mud',    'color', [0.5 0.3 0.1], 'cost', 3.0, 'climbFactor', 2.5, 'descentFactor', 1.2, 'climbSteepnessFactor', 0.3, 'descentGentlenessFactor', 0.15);
    struct('name', 'Road',   'color', [0.6 0.6 0.6], 'cost', 0.5, 'climbFactor', 1.2, 'descentFactor', 0.7, 'climbSteepnessFactor', 0.05, 'descentGentlenessFactor', 0.02);
    struct('name', 'Water',  'color', [0.2 0.4 0.8], 'cost', inf, 'climbFactor', inf, 'descentFactor', inf, 'climbSteepnessFactor', inf, 'descentGentlenessFactor', inf); % Impassable
    struct('name', 'Forest', 'color', [0.1 0.4 0.1], 'cost', 2.5, 'climbFactor', 2.0, 'descentFactor', 1.0, 'climbSteepnessFactor', 0.25, 'descentGentlenessFactor', 0.1);
};


hFig = figure('Name', 'HPA* - Click START then GOAL. Press ''r'' to Reset', ...
              'NumberTitle', 'off', 'Color', [0.95 0.95 0.95]);
ax = gca;
hold on;
gridSize = length(terrainGrid);
% Plot cells with terrain colors (initial render only, AStar's show will re-render if called with 'ShowProcess')
for i = 1:gridSize
    for j = 1:gridSize
        currentTerrainIndex = terrainGrid(i, j);
        currentColor = terrainTypes{currentTerrainIndex}.color;
        rectangle('Position', [j-1, i-1, 1, 1], ...
                  'FaceColor', currentColor, 'EdgeColor', [0.8 0.8 0.8], 'LineWidth', 0.1);
    end
end

end