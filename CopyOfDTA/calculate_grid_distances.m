% MATLAB Script: calculate_grid_distances.m
% This script calculates the minimum cost to reach every cell in the terrain grid
% from a specified start point, using Dijkstra's algorithm.
% Movement is restricted to 4-directional (no diagonals).

fprintf('Starting grid distance calculation...\n');

% --- 1. Load the generated terrain data ---
% Make sure 'terrain_data.mat' exists from running terrain_abstraction_main.m
load('terrain_data.mat');

fprintf('Loaded terrain data. Grid size: %d\n', gridSize);

% --- 2. Define Start Point ---
% For now, hardcode. This will be made interactive later.
startRow = 5;
startCol = 5;

% Validate start point
if startRow < 1 || startRow > gridSize || startCol < 1 || startCol > gridSize
    error('Start point (%d, %d) is out of grid bounds.', startRow, startCol);
end

% Get the terrain cost of the start cell
startCellTerrainCost = terrainTypes{terrainGrid(startRow, startCol)}.cost;
if isinf(startCellTerrainCost)
    error('Start point is on impassable terrain. Cannot calculate distances.');
end

fprintf('Calculating distances from start point: (%d, %d)\n', startRow, startCol);

% --- 3. Initialize Dijkstra's Algorithm Data Structures ---
% distances: Stores the minimum cost from the start point to each cell
% Initialize all distances to infinity, except the start point
distances = ones(gridSize, gridSize) * inf;
distances(startRow, startCol) = 0;

% visited: Tracks which cells have been finalized (their minimum distance is known)
visited = false(gridSize, gridSize);

% priorityQueue: A simple way to manage cells to visit, ordered by current minimum distance
% In a real-world scenario, you'd use a min-priority queue data structure.
% For simplicity here, we'll iterate and find the unvisited node with min distance.

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

    % If no unvisited cell is reachable (minDist is still inf), break
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
            % Get terrain cost of the neighbor cell
            neighborTerrainCost = terrainTypes{terrainGrid(v_r, v_c)}.cost;

            % Only consider if neighbor is not visited and is traversable
            if ~visited(v_r, v_c) && ~isinf(neighborTerrainCost)
                % The cost to move to a neighbor is the neighbor's terrain cost.
                % This is a common way to model it: cost to ENTER the cell.
                cost_to_neighbor = neighborTerrainCost;

                % If the current path to neighbor is shorter, update it
                if distances(u_r, u_c) + cost_to_neighbor < distances(v_r, v_c)
                    distances(v_r, v_c) = distances(u_r, u_c) + cost_to_neighbor;
                end
            end
        end
    end
end

fprintf('Finished calculating distances.\n');

% --- 4. Visualization of Distances ---
figure('Name', 'Grid Distances from Start Point', 'NumberTitle', 'off', 'Color', [0.95 0.95 0.95]);
ax = gca; % Get current axes handle
hold on;

% Use a colormap for visualization
% We'll normalize distances for color mapping, ignoring Inf values
finiteDistances = distances(~isinf(distances));
if isempty(finiteDistances)
    warning('No reachable cells from the start point.');
    % Just plot terrain as fallback
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
    title('Grid Distances (No reachable cells)');
else
    maxFiniteDist = max(finiteDistances);
    minFiniteDist = min(finiteDistances);
    
    % Generate the colormap array
    numColors = 256; % Standard number of colors for colormaps
    my_colormap = jet(numColors); % Get 'jet' colormap with 256 colors
    
    for i = 1:gridSize
        for j = 1:gridSize
            if isinf(distances(i, j))
                % Impassable or unreachable cells
                currentColor = [0.7 0.7 0.7]; % Grey for unreachable/impassable
            else
                % Normalize distance to [0, 1] for colormap mapping
                normalizedDist = (distances(i, j) - minFiniteDist) / (maxFiniteDist - minFiniteDist + eps); % Add eps to avoid division by zero if all distances are same
                
                % Map normalizedDist to an index in the colormap array
                color_index = max(1, min(numColors, round(normalizedDist * (numColors - 1)) + 1));
                currentColor = my_colormap(color_index, :);
            end
            
            rectangle('Position', [j-1, i-1, 1, 1], ...
                      'FaceColor', currentColor, ...
                      'EdgeColor', [0.8 0.8 0.8], ...
                      'LineWidth', 0.1);
            
            % Optionally, display the distance value on the cell
            % text(j - 0.5, i - 0.5, sprintf('%.1f', distances(i, j)), ...
            %      'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            %      'FontSize', 6, 'Color', 'k');
        end
    end
    
    % Add a color bar to show the distance scale
    colorbar('Ticks', linspace(0, 1, 5), ...
             'TickLabels', arrayfun(@(x) sprintf('%.1f', x), linspace(minFiniteDist, maxFiniteDist, 5), 'UniformOutput', false), ...
             'Label', 'Cost from Start Point');

    title(sprintf('Grid Distances from Start Point (%d, %d)', startRow, startCol));
end

% Plot the start point
plot(startCol - 0.5, startRow - 0.5, 'p', 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'MarkerSize', 15);
text(startCol - 0.5, startRow - 0.5, 'S', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
     'FontSize', 10, 'Color', 'k', 'FontWeight', 'bold');


axis equal;
xlim([0, gridSize]);
ylim([0, gridSize]);
set(gca, 'YDir', 'normal');
set(gca, 'XTick', 0:gridSize, 'YTick', 0:gridSize);
grid on;
box on;
xlabel('X-coordinate');
ylabel('Y-coordinate');

hold off;
fprintf('Distance calculation and visualization complete.\n');