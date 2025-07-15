% MATLAB Script: interactive_terrain_pathfinder.m
% This script first generates a new random terrain, then loads the data,
% allows interactive selection of a start point, calculates minimum costs,
% and displays the costs directly on the grid cells.

fprintf('Starting interactive terrain pathfinder...\n');

% --- 1. Generate a NEW terrain data file ---
% This ensures a new random terrain map is created every time this script runs.
fprintf('Generating new terrain map...\n');
run('terrain_generation.m'); % Execute terrain_generation.m to create fresh terrain_data.mat
fprintf('New terrain map generated and saved to terrain_data.mat.\n');

% --- 2. Load the newly generated terrain data ---
try
    load('terrain_data.mat');
catch
    error('Could not load terrain_data.mat after generation. Check terrain_generation.m for errors.');
end

fprintf('Loaded terrain data. Grid size: %d\n', gridSize);

% --- 3. Initial Visualization of Terrain Map for Selection ---
hFig = figure('Name', 'Terrain Map - Click to Select Start Point', ...
              'NumberTitle', 'off', 'Color', [0.95 0.95 0.95]);
ax = gca; % Get current axes handle
hold on;

% Plot each cell with its corresponding terrain color
for i = 1:gridSize
    for j = 1:gridSize
        currentTerrainIndex = terrainGrid(i, j);
        currentColor = terrainTypes{currentTerrainIndex}.color;
        
        rectangle('Position', [j-1, i-1, 1, 1], ...
                  'FaceColor', currentColor, ...
                  'EdgeColor', [0.8 0.8 0.8], ...
                  'LineWidth', 0.1);
    end
end

% Draw the sector lines
for x = 0:sectorSize:gridSize
    plot([x, x], [0, gridSize], 'k-', 'LineWidth', 1.5);
end
for y = 0:sectorSize:gridSize
    plot([0, gridSize], [y, y], 'k-', 'LineWidth', 1.5);
end

axis equal;
xlim([0, gridSize]);
ylim([0, gridSize]);
set(ax, 'YDir', 'normal');
set(ax, 'XTick', 0:gridSize, 'YTick', 0:gridSize);
grid on;
box on;
title('Terrain Map - Click on a cell to select the Start Point');
xlabel('X-coordinate');
ylabel('Y-coordinate');

fprintf('Please click on a cell in the figure to select your start point...\n');
% Wait for user click
[x, y] = ginput(1); % Get one click from the user

% Convert clicked coordinates to grid indices
startCol = floor(x) + 1;
startRow = floor(y) + 1;

% Clear existing plots to redraw with distances
cla(ax); % Clear axes but keep figure

% Validate selected start point
if startRow < 1 || startRow > gridSize || startCol < 1 || startCol > gridSize
    error('Selected start point (%.1f, %.1f) is outside the grid. Please restart and click inside the grid.', x, y);
end

% Get the terrain cost of the start cell
startCellTerrainCost = terrainTypes{terrainGrid(startRow, startCol)}.cost;
if isinf(startCellTerrainCost)
    error('Start point is on impassable terrain (Water). Cannot calculate distances from here.');
end

fprintf('Selected start point: (%d, %d). Calculating distances...\n', startRow, startCol);

% --- 4. Dijkstra's Algorithm for Distance Calculation ---
% distances: Stores the minimum cost from the start point to each cell
distances = ones(gridSize, gridSize) * inf;
distances(startRow, startCol) = 0;

% visited: Tracks which cells have been finalized
visited = false(gridSize, gridSize);

% Directions for 4-directional movement (up, down, left, right)
dr = [-1, 1, 0, 0];
dc = [0, 0, -1, 1];

numCells = gridSize * gridSize;
cellsProcessed = 0;

% Main Dijkstra's loop
while cellsProcessed < numCells
    minDist = inf;
    u_r = -1; % Row of the unvisited cell with the minimum distance
    u_c = -1; % Column of the unvisited cell with the minimum distance

    % Find the unvisited cell with the smallest distance
    for r = 1:gridSize
        for c = 1:gridSize
            if ~visited(r, c) && distances(r, c) < minDist
                minDist = distances(r, c);
                u_r = r;
                u_c = c;
            end
        end
    end

    % If no unvisited cell is reachable, break
    if u_r == -1
        break;
    end

    % Mark the current cell as visited
    visited(u_r, u_c) = true;
    cellsProcessed = cellsProcessed + 1;

    % Explore neighbors
    for k = 1:length(dr)
        v_r = u_r + dr(k);
        v_c = u_c + dc(k);

        % Check if neighbor is within grid bounds
        if v_r >= 1 && v_r <= gridSize && v_c >= 1 && v_c <= gridSize
            neighborTerrainCost = terrainTypes{terrainGrid(v_r, v_c)}.cost;

            if ~visited(v_r, v_c) && ~isinf(neighborTerrainCost)
                cost_to_neighbor = neighborTerrainCost; % Cost to enter the neighbor cell
                if distances(u_r, u_c) + cost_to_neighbor < distances(v_r, v_c)
                    distances(v_r, v_c) = distances(u_r, u_c) + cost_to_neighbor;
                end
            end
        end
    end
end

fprintf('Finished calculating distances. Displaying results...\n');

% --- 5. Visualization of Distances on Grid Cells ---
set(hFig, 'Name', 'Grid Distances with Costs on Cells'); % Update figure title

for i = 1:gridSize
    for j = 1:gridSize
        currentTerrainIndex = terrainGrid(i, j);
        currentColor = terrainTypes{currentTerrainIndex}.color;
        
        rectangle('Position', [j-1, i-1, 1, 1], ...
                  'FaceColor', currentColor, ... % Use terrain color as background
                  'EdgeColor', [0.8 0.8 0.8], ...
                  'LineWidth', 0.1);
        
        % Display distance value on the cell
        if isinf(distances(i, j))
            distText = 'Inf'; % Or 'X' for unreachable
            textColor = [0.2 0.2 0.2]; % Darker color for Inf
        else
            distText = sprintf('%.1f', distances(i, j)); % Display with one decimal
            textColor = [0 0 0]; % Black for readable numbers
        end

        text(j - 0.5, i - 0.5, distText, ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
             'FontSize', 8, 'Color', textColor, 'FontWeight', 'bold');
    end
end

% Re-draw the sector lines over the numbers
for x = 0:sectorSize:gridSize
    plot([x, x], [0, gridSize], 'k-', 'LineWidth', 1.5);
end
for y = 0:sectorSize:gridSize
    plot([0, gridSize], [y, y], 'k-', 'LineWidth', 1.5);
end

% Plot the start point marker and 'S'
plot(startCol - 0.5, startRow - 0.5, 'p', 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'MarkerSize', 15);
text(startCol - 0.5, startRow - 0.5, 'S', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
     'FontSize', 10, 'Color', 'k', 'FontWeight', 'bold');

% Set axis properties again as cla might reset some
axis equal;
xlim([0, gridSize]);
ylim([0, gridSize]);
set(gca, 'YDir', 'normal');
set(gca, 'XTick', 0:gridSize, 'YTick', 0:gridSize);
grid on;
box on;
title(sprintf('Grid Distances from Start Point (%d, %d)', startRow, startCol));
xlabel('X-coordinate');
ylabel('Y-coordinate');

hold off;

fprintf('Interactive visualization complete.\n');