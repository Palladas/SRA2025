clear;
clc;
close all;
fprintf('Starting Hierarchical Pathfinding A* (HPA*) process...\n');

% --- Generate and Load Terrain Data ---
fprintf('Generating new terrain map...\n');
clear PerlinNoise2D; % Ensure Perlin noise re-randomizes
run('terrain_generation.m'); % Make sure terrain_generation.m is in path
load('terrain_data.mat'); % Loads terrainGrid, terrainTypes, altitudeGrid, gridSize, sectorSize
fprintf('Loaded terrain data. Grid size: %d, Sector size: %d\n', gridSize, sectorSize);

% Create a GridMap object for low-level pathfinding
detailedMap = GridMap(terrainGrid, terrainTypes, altitudeGrid, inf, sectorSize);

% --- Build the Abstract Graph with Gateways ---
fprintf('Building the abstract graph using gateways...\n');

% Identify Regions (Existing BFS logic)
regions = {}; % Stores information about each identified region (node in G')
regionIdCounter = 0;
labeledGrid = zeros(gridSize, gridSize); % Maps grid cells to region IDs (used for terrain type uniformity check within sector)

numSectorsX = gridSize / sectorSize;
numSectorsY = gridSize / sectorSize;

for sRow = 1:numSectorsY
    for sCol = 1:numSectorsX
        startSectorRow = (sRow - 1) * sectorSize + 1;
        endSectorRow = sRow * sectorSize;
        startSectorCol = (sCol - 1) * sectorSize + 1;
        endSectorCol = sCol * sectorSize;

        for r = startSectorRow:endSectorRow
            for c = startSectorCol:endSectorCol
                if labeledGrid(r, c) == 0 % If cell is not yet part of any region
                    currentTerrainTypeIdx = terrainGrid(r, c);
                    if isinf(terrainTypes{currentTerrainTypeIdx}.cost) % Skip water/impassable cells
                        continue;
                    end

                    regionIdCounter = regionIdCounter + 1;
                    
                    q = [r, c]; % Queue for BFS
                    currentRegionCells = []; % Cells belonging to this region
                    
                    labeledGrid(r, c) = regionIdCounter;
                    currentRegionCells = [currentRegionCells; r, c];
                    
                    qIdx = 1;
                    while qIdx <= size(q, 1)
                        currentCell = q(qIdx, :);
                        currR = currentCell(1);
                        currC = currentCell(2);
                        qIdx = qIdx + 1;
                        
                        dr_bfs = [-1, 1, 0, 0]; % 4-directional neighbors for BFS within sector
                        dc_bfs = [0, 0, -1, 1];
                        
                        for k = 1:length(dr_bfs)
                            n_r = currR + dr_bfs(k);
                            n_c = currC + dc_bfs(k); % Corrected line: 'n_c = currC + dc_bfs(k)'
                            
                            % Check if neighbor is within current sector and within grid bounds
                            if n_r >= startSectorRow && n_r <= endSectorRow && ...
                               n_c >= startSectorCol && n_c <= endSectorCol && ...
                               n_r >= 1 && n_r <= gridSize && ...
                               n_c >= 1 && n_c <= gridSize
                               
                                if labeledGrid(n_r, n_c) == 0 && ... % Not yet visited
                                   terrainGrid(n_r, n_c) == currentTerrainTypeIdx % Same terrain type
                                    
                                    labeledGrid(n_r, n_c) = regionIdCounter;
                                    currentRegionCells = [currentRegionCells; n_r, n_c];
                                    q = [q; n_r, n_c];
                                    
                                end
                            end
                        end
                    end
                    
                    % Only create a region if it has cells (e.g., if not skipped due to water)
                    if ~isempty(currentRegionCells)
                        avgR = mean(currentRegionCells(:, 1));
                        avgC = mean(currentRegionCells(:, 2));
                        representativeState = [avgR, avgC];
                        
                        newRegion = struct(...
                            'id', regionIdCounter, ...
                            'sector_row', sRow, ...
                            'sector_col', sCol, ...
                            'terrain_type_idx', currentTerrainTypeIdx, ...
                            'cells', currentRegionCells, ...
                            'representative_state', representativeState ...
                        );
                        regions{regionIdCounter} = newRegion;
                    else
                        regionIdCounter = regionIdCounter - 1; % Decrement if no cells were added (e.g., small water patch)
                    end
                end
            end
        end
    end
end
% Filter out empty regions if any were created due to 'continue'
regions = regions(~cellfun('isempty', regions));
regionIdCounter = length(regions); % Recalculate actual number of regions
fprintf('Finished identifying regions. Total regions found: %d\n', regionIdCounter);


% Identify Gateways (Cells on sector boundaries)
allGatewaysMap = containers.Map('KeyType', 'char', 'ValueType', 'any'); % Map 'r_c' to Node object
gatewaysInSector = cell(numSectorsY, numSectorsX); % Cell array to store Node objects per sector [sRow, sCol]
gatewaySectorMap = containers.Map('KeyType', 'char', 'ValueType', 'any');

fprintf('Identifying gateway nodes...\n');
for r = 1:gridSize
    for c = 1:gridSize
        % Skip impassable cells as potential gateways
        if isinf(terrainTypes{terrainGrid(r,c)}.cost)
            continue;
        end
        
        curr_sRow = ceil(r / sectorSize);
        curr_sCol = ceil(c / sectorSize);

        % Check 4-directional neighbors for inter-sector connections
        dr = [-1, 1, 0, 0];
        dc = [0, 0, -1, 1];
        isGateway = false;
        
        for k = 1:length(dr)
            n_r = r + dr(k);
            n_c = c + dc(k);
            
            if n_r >= 1 && n_r <= gridSize && n_c >= 1 && n_c <= gridSize
                n_sRow = ceil(n_r / sectorSize);
                n_sCol = ceil(n_c / sectorSize);
                
                if n_sRow ~= curr_sRow || n_sCol ~= curr_sCol % Neighbor is in a different sector
                    % Ensure the neighbor is also passable to truly be a "gateway"
                    if ~isinf(terrainTypes{terrainGrid(n_r,n_c)}.cost)
                        isGateway = true;
                        break; 
                    end
                end
            end
        end

        if isGateway
            gatewayNode = Node(r, c);
            if ~isKey(allGatewaysMap, gatewayNode.key)
                allGatewaysMap(gatewayNode.key) = {gatewayNode}; % Store Node object in a cell array
                gatewaysInSector{curr_sRow, curr_sCol} = [gatewaysInSector{curr_sRow, curr_sCol}, gatewayNode];
                gatewaySectorMap(gatewayNode.key) = {struct('sRow', curr_sRow, 'sCol', curr_sCol)};
            end
        end
    end
end

allUniqueGateways_cells = values(allGatewaysMap); % Get a cell array where each element is {NodeObject}
allUniqueGateways = cell(1, length(allUniqueGateways_cells)); % Preallocate
for i = 1:length(allUniqueGateways_cells)
    if iscell(allUniqueGateways_cells{i}) && ~isempty(allUniqueGateways_cells{i})
        allUniqueGateways{i} = allUniqueGateways_cells{i}{1}; % Extract the Node object
    else
        allUniqueGateways{i} = [];
    end
end
allUniqueGateways = [allUniqueGateways{:}];
fprintf('Identified %d unique gateway nodes.\n', length(allUniqueGateways));

%{
tic

% Precompute Intra-Sector Paths (Using A* within sector bounds)
intraSectorPathsCache = containers.Map('KeyType', 'char', 'ValueType', 'any'); 

fprintf('Precomputing intra-sector paths...\n');
for sRow = 1:numSectorsY
    for sCol = 1:numSectorsX
        currentSectorGateways = gatewaysInSector{sRow, sCol};
        if isempty(currentSectorGateways)
            continue;
        end
        
        % Sector bounds for constrained A*
        sector_min_r = (sRow - 1) * sectorSize + 1;
        sector_max_r = sRow * sectorSize;
        sector_min_c = (sCol - 1) * sectorSize + 1;
        sector_max_c = sCol * sectorSize;

        % Iterate over all unique pairs of gateways within this sector
        for i = 1:length(currentSectorGateways)
            g1_node = currentSectorGateways(i); % Start gateway Node
            
            % Run Dijkstra from g1_node constrained to this sector
            % For simplicity, we'll use AStar repeatedly for now.
            % A true Dijkstra from a single source would be more efficient for all-pairs from one node.
            
            for j = (i+1):length(currentSectorGateways) % Only compute unique pairs (g1_node, g2_node)
                g2_node = currentSectorGateways(j); % Goal gateway Node
                
                % Check if path already computed (due to symmetry or previous run)
                key_fwd = sprintf('%s_to_%s', g1_node.key, g2_node.key);
                key_rev = sprintf('%s_to_%s', g2_node.key, g1_node.key);

                if isKey(intraSectorPathsCache, key_fwd)
                    continue; % Already computed
                end

                % Set initial cost and parent for A*
                start_node_for_astar = Node(g1_node.row, g1_node.col); start_node_for_astar.cost = 0;
                goal_node_for_astar = Node(g2_node.row, g2_node.col); goal_node_for_astar.cost = 0;
                
                % Call AStar with sector constraints (needs modification to AStar or a new function)
                detailed_segment_path_data = AStar(detailedMap, start_node_for_astar, goal_node_for_astar, 'ShowProcess', false); 
                
                if ~isempty(detailed_segment_path_data)
                    segment_cost = detailed_segment_path_data(end, 3);
                    intraSectorPathsCache(key_fwd) = struct('cost', segment_cost, 'path', detailed_segment_path_data);
                    % Store reverse path (optional, can be computed on the fly by flipping)
                    intraSectorPathsCache(key_rev) = struct('cost', segment_cost, 'path', flipud(detailed_segment_path_data)); 
                else
                    % If no path, record infinite cost (important for abstract graph)
                    intraSectorPathsCache(key_fwd) = struct('cost', inf, 'path', []);
                    intraSectorPathsCache(key_rev) = struct('cost', inf, 'path', []);
                end
            end
        end
    end
end
fprintf('Finished precomputing intra-sector paths. Cache size: %d\n', intraSectorPathsCache.size);
toc
%}



% Build Abstract Graph (nodes are Gateway Node objects, edges have precomputed costs)
abstractGraph = containers.Map('KeyType', 'char', 'ValueType', 'any'); 

% Add all unique gateways as nodes in the abstract graph
for k = 1:length(allUniqueGateways)
    abstractGraph(allUniqueGateways(k).key) = []; % Initialize empty adjacency list for each gateway
end

%{
tic
% Add Intra-sector edges (from cache)
fprintf('Building intra-sector edges...\n');
keys_cache = keys(intraSectorPathsCache);
for i = 1:length(keys_cache)
    key_str = keys_cache{i};
    path_info = intraSectorPathsCache(key_str);
    
    if isempty(path_info.path) || isinf(path_info.cost)
        continue;
    end

    % Extract start and end gateway keys from the cache key string
    parts = strsplit(key_str, '_to_');
    g1_key = parts{1};
    g2_key = parts{2};
    
    % Add edge from g1 to g2
    abstractGraph(g1_key) = [abstractGraph(g1_key); struct('target_gateway_key', g2_key, 'cost', path_info.cost)];
end

% Add Inter-sector edges (1-cell moves between adjacent gateways in different sectors)
fprintf('Building inter-sector edges...\n');
for r = 1:gridSize
    for c = 1:gridSize
        current_gateway_key = sprintf('%d_%d', r, c);
        if isKey(allGatewaysMap, current_gateway_key) % If this cell is a gateway
            current_gateway_val = allGatewaysMap(current_gateway_key);
            if iscell(current_gateway_val) && ~isempty(current_gateway_val)
                current_gateway_node = current_gateway_val{1}; % Retrieve Node object from cell array
            else
                warning('Value for gateway key %s is not a valid Node cell. Skipping.', current_gateway_key);
                continue;
            end
            
            curr_sector_cell = gatewaySectorMap(current_gateway_key);
            curr_sector = curr_sector_cell{1};

            dr = [-1, -1, -1,  0, 0,  1, 1, 1]; % 8-directional for inter-sector
            dc = [-1,  0,  1, -1, 1, -1, 0, 1];

            for k = 1:length(dr)
                n_r = r + dr(k);
                n_c = c + dc(k);
                neighbor_gateway_key = sprintf('%d_%d', n_r, n_c);

                if n_r >= 1 && n_r <= gridSize && n_c >= 1 && n_c <= gridSize && ...
                   isKey(allGatewaysMap, neighbor_gateway_key) % If neighbor is also a gateway
                    
                    neighbor_sector_cell = gatewaySectorMap(neighbor_gateway_key);
                    neighbor_sector = neighbor_sector_cell{1};

                    if neighbor_sector.sRow ~= curr_sector.sRow || neighbor_sector.sCol ~= curr_sector.sCol % If they are in different sectors
                        % This is an inter-sector gateway connection (1 cell move)
                        cost_inter_sector = detailedMap.getCost(r, c, n_r, n_c);

                        if ~isinf(cost_inter_sector)
                            % Add edge from current gateway to neighbor gateway
                            abstractGraph(current_gateway_key) = [abstractGraph(current_gateway_key); ...
                                struct('target_gateway_key', neighbor_gateway_key, 'cost', cost_inter_sector)];
                            % Add reverse edge as well (assuming symmetric costs)
                            abstractGraph(neighbor_gateway_key) = [abstractGraph(neighbor_gateway_key); ...
                                struct('target_gateway_key', current_gateway_key, 'cost', cost_inter_sector)];
                        end
                    end
                end
            end
        end
    end
end
fprintf('Finished building abstract graph edges. Total abstract nodes: %d\n', abstractGraph.size);
toc
%}

% --- Initial Visualization Setup ---
hFig = figure('Name', 'HPA* - Click START then GOAL. Press ''r'' to Reset', ...
              'NumberTitle', 'off', 'Color', [0.95 0.95 0.95]);
ax = gca;
hold on;

% Plot cells with terrain colors (initial render only, AStar's show will re-render if called with 'ShowProcess')
for i = 1:gridSize
    for j = 1:gridSize
        currentTerrainIndex = terrainGrid(i, j);
        currentColor = terrainTypes{currentTerrainIndex}.color;
        rectangle('Position', [j-1, i-1, 1, 1], ...
                  'FaceColor', currentColor, 'EdgeColor', [0.8 0.8 0.8], 'LineWidth', 0.1);
    end
end

% Draw sector lines
for x = 0:sectorSize:gridSize
    plot([x, x], [0, gridSize], 'k-', 'LineWidth', 1.5);
end
for y = 0:sectorSize:gridSize
    plot([0, gridSize], [y, y], 'k-', 'LineWidth', 1.5);
end

% Plot Abstract Graph Gateway Nodes
gatewayPlotHandle = [];
if ~isempty(allUniqueGateways)
    gateway_rows = [allUniqueGateways(:).row];
    gateway_cols = [allUniqueGateways(:).col];
    gatewayPlotHandle = plot(gateway_cols - 0.5, gateway_rows - 0.5, 'o', ...
                         'MarkerFaceColor', [0.8 0.8 0.8], 'MarkerEdgeColor', 'k', 'MarkerSize', 5, 'LineWidth', 0.5); % Smaller, gray circles for gateways
end

% Plot Abstract Graph Edges (connecting gateways)
abstractEdgePlotHandle = [];
gateway_keys = keys(abstractGraph);
for i = 1:length(gateway_keys)
    source_key = gateway_keys{i};
    if isKey(allGatewaysMap, source_key)
        source_node_val = allGatewaysMap(source_key);
        if iscell(source_node_val) && ~isempty(source_node_val)
            source_node = source_node_val{1}; % Retrieve Node object from cell array
        else
            warning('Source gateway value for key %s is not a valid Node cell. Skipping plotting edge.', source_key);
            continue;
        end
    else
        warning('Source gateway key %s not found in allGatewaysMap during plotting.', source_key);
        continue;
    end
    
    edges_from_source = abstractGraph(source_key);
    for j = 1:length(edges_from_source)
        target_key = edges_from_source(j).target_gateway_key;
        if isKey(allGatewaysMap, target_key)
            target_node_val = allGatewaysMap(target_key);
            if iscell(target_node_val) && ~isempty(target_node_val)
                target_node = target_node_val{1}; % Retrieve Node object from cell array
            else
                warning('Target gateway value for key %s is not a valid Node cell. Skipping plotting edge.', target_key);
                continue;
            end
        else
            warning('Target gateway key %s not found in allGatewaysMap during plotting.', target_key);
            continue;
        end
        
        % Only plot each edge once (e.g., source_key < target_key lexicographically)
        % New logic for unique plotting:
        if strcmp(source_key, target_key) % Skip self-loops if they exist
            continue;
        end
        
        % Ensure the edge is plotted only once (e.g., for 'A_B' and 'B_A', only plot 'A_B')
        sorted_keys = sort({source_key, target_key});
        if strcmp(source_key, sorted_keys{1}) 
            p = plot([source_node.col - 0.5, target_node.col - 0.5], ...
                     [source_node.row - 0.5, target_node.row - 0.5], ...
                     'b-', 'LineWidth', 0.5, 'Color', [0 0.5 0]); % Dark green abstract edges
            if isempty(abstractEdgePlotHandle), abstractEdgePlotHandle = p; 
            end
        end
    end
end

axis equal; xlim([0, gridSize]); ylim([0, gridSize]); set(ax, 'YDir', 'normal');
set(ax, 'XTick', 0:gridSize, 'YTick', 0:gridSize); grid on; box on;
title('HPA*: Click START then GOAL. Press ''r'' to Reset.');
xlabel('X-coordinate'); ylabel('Y-coordinate');


% Initialize legend handles for static map elements
terrainLegendHandles = []; terrainLegendLabels = {};
for k = 1:length(terrainTypes)
    h = plot(NaN, NaN, 's', 'MarkerFaceColor', terrainTypes{k}.color, ...
                           'MarkerEdgeColor', 'none', 'MarkerSize', 10);
    terrainLegendHandles = [terrainLegendHandles, h];
    costStr = sprintf('%.1f', terrainTypes{k}.cost);
    if isinf(terrainTypes{k}.cost), costStr = 'Inf'; end
    terrainLegendLabels = [terrainLegendLabels, {sprintf('%s (Cost: %s)', terrainTypes{k}.name, costStr)}];
end

% Base legend for map and abstract graph elements (these remain constant)
baseLegendHandles = terrainLegendHandles;
baseLegendLabels = terrainLegendLabels;

% Define the figure handle and axes for plotting
figure_handle = figure();
ax = axes('Parent', figure_handle);
set(ax, 'DataAspectRatio', [1 1 1]); % Maintain aspect ratio
hold(ax, 'on');

% --- Plot base map terrain ---
detailedMap.show(ax);

baseLegendHandles = gobjects(0); % Initialize as empty graphics object array
baseLegendLabels = {};

for i = 1:length(detailedMap.TerrainInfo)
    terrain_props = detailedMap.TerrainInfo{i};
    % Create an invisible patch for each terrain type to use in the legend
    h_temp = patch(ax, NaN, NaN, NaN, 'FaceColor', terrain_props.color, 'EdgeColor', 'none', 'Visible', 'off');
    baseLegendHandles(end+1) = h_temp;
    baseLegendLabels{end+1} = sprintf('%s (Cost: %.1f)', terrain_props.name, terrain_props.cost);
end

gatewayPlotHandle_invisible = gobjects(0); % Initialize as empty graphics object array
if ~isempty(allUniqueGateways)
    % Use parenthesis () for array indexing, not braces {}
    for i = 1:length(allUniqueGateways)
        gw = allUniqueGateways(i); % Corrected: Use () for indexing Node objects in array
        if isa(gw, 'Node') % Ensure it's a Node object
            % Plot directly onto 'ax' and set Visible to 'off'
            h = plot(ax, gw.col - 0.5, gw.row - 0.5, ...
                     'o', 'MarkerSize', 6, 'MarkerFaceColor', [0.8 0.4 0.0], ...
                     'MarkerEdgeColor', 'k', 'Tag', 'GatewayNodes', 'Visible', 'off');
            gatewayPlotHandle_invisible(end+1) = h;
        end
    end
end

%{
abstractEdgePlotHandle_invisible = gobjects(0); % Initialize as empty graphics object array
if abstractGraph.Count > 0
    abstract_keys = keys(abstractGraph);
    for i = 1:length(abstract_keys)
        node_key = abstract_keys{i};
        from_node = abstractGraph(node_key).representative_node;
        for j = 1:length(abstractGraph(node_key).edges)
            to_node = abstractGraph(node_key).edges(j).to_node;
            % Plot directly onto 'ax' and set Visible to 'off'
            line_h = plot(ax, [from_node.col-0.5, to_node.col-0.5], ...
                                    [from_node.row-0.5, to_node.row-0.5], ...
                                    'g-', 'LineWidth', 1.5, 'Tag', 'AbstractEdges', 'Visible', 'off');
            abstractEdgePlotHandle_invisible(end+1) = line_h;
        end
    end
end
%}

% Set the initial axis limits if necessary
xlim(ax, [0, gridSize]);
ylim(ax, [0, gridSize]);
set(ax, 'YDir', 'normal'); % Ensure Y-axis is oriented correctly
set(ax, 'XTick', 0:gridSize, 'YTick', 0:gridSize);
grid(ax, 'on');
box(ax, 'on');

valid_idx_initial = isvalid(baseLegendHandles);
legend(baseLegendHandles(valid_idx_initial), baseLegendLabels(valid_idx_initial), 'Location', 'eastoutside', 'FontSize', 9, 'AutoUpdate', 'off');
%{
% --- Define the main interactive plotting function ---
plotAndFindPath = @(ax_handle) local_plotAndFindPath(...
    ax_handle, detailedMap, allGatewaysMap, gatewaysInSector, intraSectorPathsCache, ...
    abstractGraph, terrainGrid, terrainTypes, baseLegendHandles, baseLegendLabels, gridSize, sectorSize);

% --- Set up KeyPressFcn ---
set(hFig, 'KeyPressFcn', @(src, event) local_keyPressCallback(src, event, plotAndFindPath, ax));

% --- Initial run of pathfinding ---
plotAndFindPath(ax);

% --- Local Helper Functions (defined at the end of the script) ---
function local_plotAndFindPath(ax, detailedMap, allGatewaysMap, gatewaysInSector, intraSectorPathsCache, ...
                               abstractGraph, terrainGrid, terrainTypes, baseLegendHandles, baseLegendLabels, gridSize, sectorSize)
    % Clear previous path and markers
    delete(findobj(ax, 'Tag', 'PathLine'));
    delete(findobj(ax, 'Tag', 'StartMarker'));
    delete(findobj(ax, 'Tag', 'GoalMarker'));
    delete(findobj(ax, 'Tag', 'StartText'));
    delete(findobj(ax, 'Tag', 'GoalText'));
    delete(findobj(ax, 'Tag', 'PathLegend'));

    % Initialize plot handles to empty, just in case
    startPointPlotHandle = [];
    goalPointPlotHandle = [];
    pathLegendHandle = [];

    % Re-prompt for start/goal
    title('HPA*: Click START node, then GOAL node');
    fprintf('Please click on the START node in the figure...\n');
    [start_col, start_row] = ginput(1);
    start_col = floor(start_col) + 1;
    start_row = floor(start_row) + 1;
    startNodeDetailed = Node(start_row, start_col);

    fprintf('Please click on the GOAL node in the figure...\n');
    [goal_col, goal_row] = ginput(1);
    goal_col = floor(goal_col) + 1;
    goal_row = floor(goal_row) + 1;
    goalNodeDetailed = Node(goal_row, goal_col);

    % Validate start/goal points
    if start_row < 1 || start_row > gridSize || start_col < 1 || start_col > gridSize || ...
       goal_row < 1 || goal_row > gridSize || goal_col < 1 || goal_col > gridSize
        warning('Selected start or goal point is outside the grid. Please re-select or press ''r''.');
        return;
    end
    if isinf(terrainTypes{terrainGrid(start_row, start_col)}.cost)
        warning('Start point is on impassable terrain (Water). Please re-select or press ''r''.');
        return;
    end
    if isinf(terrainTypes{terrainGrid(goal_row, goal_col)}.cost)
        warning('Goal point is on impassable terrain (Water). Please re-select or press ''r''.');
        return;
    end
    fprintf('Selected Start: (%d, %d), Goal: (%d, %d)\n', start_row, start_col, goal_row, goal_col);

    % Plot start and goal markers
    startPointPlotHandle = plot(ax, start_col - 0.5, start_row - 0.5, 'p', 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'MarkerSize', 18, 'Tag', 'StartMarker');
    text(ax, start_col - 0.5, start_row - 0.5, 'S', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 12, 'Color', 'k', 'FontWeight', 'bold', 'Tag', 'StartText');
    goalPointPlotHandle = plot(ax, goal_col - 0.5, goal_row - 0.5, 'h', 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'k', 'MarkerSize', 18, 'Tag', 'GoalMarker');
    text(ax, goal_col - 0.5, goal_row - 0.5, 'G', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 12, 'Color', 'k', 'FontWeight', 'bold', 'Tag', 'GoalText');

    % --- Legend ---
    currentLegendHandles_temp = [baseLegendHandles, startPointPlotHandle, goalPointPlotHandle];
    currentLegendLabels_temp = [baseLegendLabels, {'Start Point'}, {'Goal Point'}];
    
    valid_idx = isvalid(currentLegendHandles_temp);
    legend(ax, currentLegendHandles_temp(valid_idx), currentLegendLabels_temp(valid_idx), ...
           'Location', 'eastoutside', 'FontSize', 9, 'AutoUpdate', 'off');
    title(sprintf('HPA*: Pathfinding from (%d,%d) to (%d,%d)', start_row, start_col, goal_row, goal_col));

    % --- High-Level Pathfinding (A* on Abstract Graph of Gateways) ---
    fprintf('Performing high-level A* search on the abstract graph (gateways)....\n');

    % Find closest gateway to start and goal
    start_sector_r = ceil(startNodeDetailed.row / sectorSize);
    start_sector_c = ceil(startNodeDetailed.col / sectorSize);
    goal_sector_r = ceil(goalNodeDetailed.row / sectorSize);
    goal_sector_c = ceil(goalNodeDetailed.col / sectorSize);

    % Find closest gateway in start sector
    
    min_dist_to_start_gateway = inf;
    closest_start_gateway = [];
    if ~isempty(gatewaysInSector{start_sector_r, start_sector_c})
        for k = 1:length(gatewaysInSector{start_sector_r, start_sector_c})
            gw = gatewaysInSector{start_sector_r, start_sector_c}(k);
            dist = sqrt((startNodeDetailed.row - gw.row)^2 + (startNodeDetailed.col - gw.col)^2);
            if dist < min_dist_to_start_gateway
                min_dist_to_start_gateway = dist;
                closest_start_gateway = gw;
            end
        end
    end

    min_dist_to_goal_gateway = inf;
    closest_goal_gateway = [];
    if ~isempty(gatewaysInSector{goal_sector_r, goal_sector_c})
        for k = 1:length(gatewaysInSector{goal_sector_r, goal_sector_c})
            gw = gatewaysInSector{goal_sector_r, goal_sector_c}(k);
            dist = sqrt((goalNodeDetailed.row - gw.row)^2 + (goalNodeDetailed.col - gw.col)^2);
            if dist < min_dist_to_goal_gateway
                min_dist_to_goal_gateway = dist;
                closest_goal_gateway = gw;
            end
        end
    end

    if isempty(closest_start_gateway) || isempty(closest_goal_gateway)
        if start_sector_r == goal_sector_r && start_sector_c == goal_sector_c % Same sector
            fprintf('Start and Goal are in the same sector, or a sector with no gateways. Running direct A*.\n');
            startNodeDetailed.cost = 0; goalNodeDetailed.cost = 0;
            detailed_final_path = AStar(detailedMap, startNodeDetailed, goalNodeDetailed, 'ShowProcess', false);
        else
            warning('Could not find gateways in start or goal sector. HPA* cannot connect. Please re-select or ensure sectors have gateways.');
            detailed_final_path = [];
        end
    else % Perform HPA* (assuming gateways were found)
        % Run A* on the abstract graph (connecting gateway keys)
        abstract_path_gateway_keys = AStarAbstract_Gateways(allGatewaysMap, abstractGraph, closest_start_gateway.key, closest_goal_gateway.key);

        detailed_final_path = []; % Initialize for HPA* path construction
        HPA_path_successful = true; % Assume success initially for HPA* parts

        if isempty(abstract_path_gateway_keys)
            fprintf('No high-level path found between gateways. HPA* failed. Attempting direct A* as fallback.\n');
            HPA_path_successful = false; % High-level path itself failed
        else
            fprintf('High-level path found (Gateway Keys): %s\n', strjoin(abstract_path_gateway_keys, ' -> '));
            
            % --- Low-Level Path Refinement (Concatenating Precomputed Paths) ---
            fprintf('Refining high-level path into detailed low-level path...\n');
            
            % Initial segment: from startNodeDetailed to first gateway
            start_path_seg = AStar(detailedMap, startNodeDetailed, closest_start_gateway, 'ShowProcess', false);
            
            if isempty(start_path_seg)
                warning('Could not find path from start to its closest gateway. HPA* failed for this segment.');
                HPA_path_successful = false; % Mark HPA* as failed for this run
            else
                detailed_final_path = start_path_seg;
            end
            
            % Only proceed with HPA* path construction if the initial segment was successful
            if HPA_path_successful && length(abstract_path_gateway_keys) > 1
                % Middle segments: between gateways using intraSectorPathsCache
                for i = 1:(length(abstract_path_gateway_keys) - 1)
                    if ~HPA_path_successful, break; end % Stop if a previous segment failed

                    current_gw_key = abstract_path_gateway_keys{i};
                    next_gw_key = abstract_path_gateway_keys{i+1};
                    
                    path_key = sprintf('%s_to_%s', current_gw_key, next_gw_key);
                    
                    if isKey(intraSectorPathsCache, path_key)
                        path_info = intraSectorPathsCache(path_key);
                        if isempty(path_info.path) || isinf(path_info.cost)
                            warning('Precomputed path between gateway %s and %s is missing or infinite. HPA* failed for this segment.', current_gw_key, next_gw_key);
                            HPA_path_successful = false; % Path failed
                        else
                            % Append, avoiding duplicate node at connection point
                            detailed_final_path = [detailed_final_path; path_info.path(2:end,:)];
                        end
                    else
                        % This warning now becomes more critical as cache should be well-populated
                        warning('Path between gateway %s and %s not found in cache. HPA* failed for this segment.', current_gw_key, next_gw_key);
                        HPA_path_successful = false; % Path failed
                    end
                end
                
                % from last gateway to goalNodeDetailed
                if HPA_path_successful % Only if all middle segments were successful
                    last_node_of_path = Node(detailed_final_path(end,1), detailed_final_path(end,2));
                    last_node_of_path.cost = detailed_final_path(end,3); % Preserve actual cost
                    
                    final_path_seg = AStar(detailedMap, last_node_of_path, goalNodeDetailed, 'ShowProcess', false);
                    if isempty(final_path_seg)
                        warning('Could not find path from last gateway to goal. HPA* failed for this segment.');
                        HPA_path_successful = false; % Path failed
                    else
                        % Append, avoiding duplicate node at connection point
                        detailed_final_path = [detailed_final_path; final_path_seg(2:end,:)]; 
                    end
                end
            end
        end

        % If HPA* process was not entirely successful, run direct A*
        if ~HPA_path_successful
             fprintf('Triggering direct A* fallback due to HPA* segment failure.\n');
             startNodeDetailed.cost = 0; goalNodeDetailed.cost = 0; % Reset costs for fresh A* run
             detailed_final_path = AStar(detailedMap, startNodeDetailed, goalNodeDetailed, 'ShowProcess', false);
             if isempty(detailed_final_path)
                 fprintf('Direct A* fallback also failed. No path found at all.\n');
             else
                 fprintf('Direct A* fallback successful.\n');
             end
        end
    end

    % --- Final Legend Update and Plotting ---
    if ~isempty(detailed_final_path)
        fprintf('Detailed HPA* path found with total cost: %.2f\n', detailed_final_path(end, 3));
        plot(ax, detailed_final_path(:,2) - 0.5, detailed_final_path(:,1) - 0.5, 'r-', 'LineWidth', 3, 'Tag', 'PathLine'); % Red path
        
        pathLegendHandle = plot(ax, NaN, NaN, 'r-', 'LineWidth', 3, 'Tag', 'PathLegend');
        
        currentLegendHandles_temp = [baseLegendHandles, startPointPlotHandle, goalPointPlotHandle, pathLegendHandle];
        currentLegendLabels_temp = [baseLegendLabels, {'Start Point'}, {'Goal Point'}, {'HPA* Path'}];
        
        valid_idx = isvalid(currentLegendHandles_temp);
        legend(ax, currentLegendHandles_temp(valid_idx), currentLegendLabels_temp(valid_idx), ...
               'Location', 'eastoutside', 'FontSize', 9, 'AutoUpdate', 'off');
    else
        fprintf('HPA* failed to find a complete detailed path.\n');
        currentLegendHandles_temp = [baseLegendHandles, startPointPlotHandle, goalPointPlotHandle];
        currentLegendLabels_temp = [baseLegendLabels, {'Start Point'}, {'Goal Point'}];
        
        valid_idx = isvalid(currentLegendHandles_temp);
        legend(ax, currentLegendHandles_temp(valid_idx), currentLegendLabels_temp(valid_idx), ...
               'Location', 'eastoutside', 'FontSize', 9, 'AutoUpdate', 'off');
    end
    title(sprintf('HPA*: Pathfinding from (%d,%d) to (%d,%d)', start_row, start_col, goal_row, goal_col));
    fprintf('HPA* pathfinding cycle complete.\n');
end

% run_HPAstar.m (Inside the local_keyPressCallback function definition)

function local_keyPressCallback(src, event, plotAndFindPath_func, ax_handle)
    original_button_down_fcn = get(src, 'WindowButtonDownFcn');
    
    set(src, 'WindowButtonDownFcn', ''); 

    if strcmp(event.Key, 'r')
        fprintf('\n''r'' pressed. Resetting path, start, and goal. Keeping the map.\n');
        plotAndFindPath_func(ax_handle);
    end
   
    set(src, 'WindowButtonDownFcn', original_button_down_fcn);
end
%}