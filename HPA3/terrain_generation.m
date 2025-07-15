clear PerlinNoise2D; % Clears the persistent variable 'p' from the PerlinNoise2D function
fprintf('Starting terrain generation script...\n');

% Seed the random number generator to ensure different terrains each time
rng('shuffle');

% Define grid dimensions
gridSize = 20;

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

% Define sector size (controlled here)
sectorSize = 5; % You can change this for DTA analysis later
fprintf('Terrain generation complete. Grid size: %d, Sector size: %d\n', gridSize, sectorSize);

% --- Generate Altitude/Elevation Grid (Z-values) using Perlin Noise ---
fprintf('Generating altitude grid using Perlin noise...\n');
% Parameters for Perlin Noise (you can adjust these)
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

% --- Save the generated terrain data to a .mat file ---
save('terrain_data.mat', 'terrainGrid', 'gridSize', 'sectorSize', 'terrainTypes', 'altitudeGrid');
fprintf('Terrain data saved to terrain_data.mat\n');

% --- Visualization Code (runs automatically as part of the script) ---
figure('Name', 'Generated Terrain Map', 'NumberTitle', 'off', 'Color', [0.95 0.95 0.95]);
hold on;
% Initialize arrays to hold all legend handles and labels
allLegendHandles = [];
allLegendLabels = {};

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

% Create dummy plots for the legend entries (terrain types)
for k = 1:length(terrainTypes)
    h = plot(NaN, NaN, 's', 'MarkerFaceColor', terrainTypes{k}.color, ...
                           'MarkerEdgeColor', 'none', 'MarkerSize', 10);
    allLegendHandles = [allLegendHandles, h];
    % Display cost in legend label
    costStr = sprintf('%.1f', terrainTypes{k}.cost);
    if isinf(terrainTypes{k}.cost)
        costStr = 'Inf';
    end
    allLegendLabels = [allLegendLabels, {sprintf('%s (Cost: %s)', terrainTypes{k}.name, costStr)}];
end
legend(allLegendHandles, allLegendLabels, 'Location', 'eastoutside', 'FontSize', 10);

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
set(gca, 'YDir', 'normal');
set(gca, 'XTick', 0:gridSize, 'YTick', 0:gridSize);
grid on;
box on;
title('Generated Terrain Map');
xlabel('X-coordinate');
ylabel('Y-coordinate');
hold off;

% --- Visualize Altitude Map ---
figure('Name', 'Generated Altitude Map', 'NumberTitle', 'off', 'Color', [0.95 0.95 0.95]);
surf(altitudeGrid)
hold on;
imagesc(altitudeGrid);
colormap('parula'); % Or 'jet', 'gray', etc. for elevation
colorbar;
xlim([0.5, gridSize + 0.5]); % Adjust limits for imagesc
ylim([0.5, gridSize + 0.5]); % Adjust limits for imagesc
set(gca, 'YDir', 'normal');
title('Generated Altitude Map (Z-values)');
xlabel('X-coordinate');
ylabel('Y-coordinate');
fprintf('Altitude map visualized.\n');