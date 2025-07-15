clear;
clc;
close all;

fprintf('Starting Hierarchical Pathfinding A* (HPA*) process...\n');

% --- Generate and Load Terrain Data ---
fprintf('Generating new terrain map...\n');
run('terrain_generation.m'); % Make sure terrain_generation.m is in path
load('terrain_data.mat');
fprintf('Loaded terrain data. Grid size: %d, Sector size: %d\n', gridSize, sectorSize);

% Create a GridMap object for low-level pathfinding
detailedMap = GridMap(terrainGrid, terrainTypes);

% --- Build the Abstract Graph ---
fprintf('Building the abstract graph (regions, nodes, edges)...\n');

regions = {}; % Stores information about each identified region (node in G')
regionIdCounter = 0;
labeledGrid = zeros(gridSize, gridSize); % Maps grid cells to region IDs

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
                    regionIdCounter = regionIdCounter + 1;
                    currentTerrainTypeIdx = terrainGrid(r, c);
                    
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
                            n_c = currC + dc_bfs(k);
                            
                            if n_r >= startSectorRow && n_r <= endSectorRow && ...
                               n_c >= startSectorCol && n_c <= endSectorCol && ...
                               n_r >= 1 && n_r <= gridSize && ...
                               n_c >= 1 && n_c <= gridSize
                               
                                if labeledGrid(n_r, n_c) == 0 && ...
                                   terrainGrid(n_r, n_c) == currentTerrainTypeIdx
                                    
                                    labeledGrid(n_r, n_c) = regionIdCounter;
                                    currentRegionCells = [currentRegionCells; n_r, n_c];
                                    q = [q; n_r, n_c];
                                    
                                end
                            end
                        end
                    end
                    
                    % Calculate Representative State (average location of cells in region)
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
                end
            end
        end
    end
end
fprintf('Finished identifying regions. Total regions found: %d\n', regionIdCounter);

% Build the Abstract Graph Edges
abstractGraph = cell(1, regionIdCounter); % Adjacency list for G'
for i = 1:regionIdCounter
    abstractGraph{i} = []; % Initialize as empty
end
addedEdges = containers.Map('KeyType', 'char', 'ValueType', 'logical'); % To avoid duplicate edges

for r = 1:gridSize
    for c = 1:gridSize
        currentRegionID = labeledGrid(r, c);
        if currentRegionID > 0
            dr_edge = [-1, 1, 0, 0];
            dc_edge = [0, 0, -1, 1];
            for k = 1:length(dr_edge)
                n_r = r + dr_edge(k);
                n_c = c + dc_edge(k);
                if n_r >= 1 && n_r <= gridSize && n_c >= 1 && n_c <= gridSize
                    neighborRegionID = labeledGrid(n_r, n_c);
                    if neighborRegionID > 0 && neighborRegionID ~= currentRegionID
                        edgeKey = sprintf('%d-%d', min(currentRegionID, neighborRegionID), max(currentRegionID, neighborRegionID));
                        if ~isKey(addedEdges, edgeKey)
                            repState1 = regions{currentRegionID}.representative_state;
                            repState2 = regions{neighborRegionID}.representative_state;
                            
                            dist = sqrt((repState1(1) - repState2(1))^2 + (repState1(2) - repState2(2))^2);
                            
                            cost1 = terrainTypes{regions{currentRegionID}.terrain_type_idx}.cost;
                            cost2 = terrainTypes{regions{neighborRegionID}.terrain_type_idx}.cost;
                            
                            if isinf(cost1) || isinf(cost2)
                                edgeCost = Inf;
                            else
                                edgeCost = dist * (cost1 + cost2) / 2;
                            end
                            
                            abstractGraph{currentRegionID} = [abstractGraph{currentRegionID}; ...
                                struct('target_region_id', neighborRegionID, 'cost', edgeCost)];
                            abstractGraph{neighborRegionID} = [abstractGraph{neighborRegionID}; ...
                                struct('target_region_id', currentRegionID, 'cost', edgeCost)];
                            
                            addedEdges(edgeKey) = true;
                        end
                    end
                end
            end
        end
    end
end
fprintf('Finished building abstract graph edges.\n');

% --- Initial Visualization Setup (without immediate user input) ---
hFig = figure('Name', 'DTA', ...
              'NumberTitle', 'off', 'Color', [0.95 0.95 0.95]);
ax = gca;
hold on;

% Plot cells with terrain colors
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

% Plot Abstract Graph Edges
edgePlotHandle = [];
for regionID = 1:length(regions)
    if ~isempty(regions{regionID})
        sourceRepState = regions{regionID}.representative_state;
        for k = 1:length(abstractGraph{regionID})
            targetRegionID = abstractGraph{regionID}(k).target_region_id;
            targetRepState = regions{targetRegionID}.representative_state;
            p = plot([sourceRepState(2)-0.5, targetRepState(2)-0.5], ...
                     [sourceRepState(1)-0.5, targetRepState(1)-0.5], ...
                     'g-', 'LineWidth', 1.5);
            if isempty(edgePlotHandle), edgePlotHandle = p; end
        end
    end
end

% Plot Abstract Graph Nodes
nodePlotHandle = [];
for k = 1:length(regions)
    if ~isempty(regions{k})
        repState = regions{k}.representative_state;
        p = plot(repState(2) - 0.5, repState(1) - 0.5, 'o', ...
             'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k', 'MarkerSize', 7, 'LineWidth', 1.0);
        if isempty(nodePlotHandle), nodePlotHandle = p; end
    end
end

axis equal; xlim([0, gridSize]); ylim([0, gridSize]); set(ax, 'YDir', 'normal');
set(ax, 'XTick', 0:gridSize, 'YTick', 0:gridSize); grid on; box on;
title('DTA: Click START then GOAL. Press ''r'' to Reset.');
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

% Base legend for map and abstract graph elements
baseLegendHandles = terrainLegendHandles;
baseLegendLabels = terrainLegendLabels;
if ~isempty(nodePlotHandle), baseLegendHandles = [baseLegendHandles, nodePlotHandle]; baseLegendLabels = [baseLegendLabels, {'Region Node'}]; end
if ~isempty(edgePlotHandle), baseLegendHandles = [baseLegendHandles, edgePlotHandle]; baseLegendLabels = [baseLegendLabels, {'Abstract Edge'}]; end

% Initial legend display
valid_idx_initial = isvalid(baseLegendHandles);
legend(baseLegendHandles(valid_idx_initial), baseLegendLabels(valid_idx_initial), 'Location', 'eastoutside', 'FontSize', 9, 'AutoUpdate', 'off'); % AutoUpdate off is crucial

% --- Define the main interactive plotting function ---
plotAndFindPath = @(ax_handle) local_plotAndFindPath(...
    ax_handle, labeledGrid, detailedMap, regions, abstractGraph, ...
    terrainGrid, terrainTypes, baseLegendHandles, baseLegendLabels, ... % Pass base legend handles
    gridSize, sectorSize);

% --- Set up KeyPressFcn ---
set(hFig, 'KeyPressFcn', @(src, event) local_keyPressCallback(src, event, plotAndFindPath, ax));

% --- Initial run of pathfinding ---
plotAndFindPath(ax);

% --- Local Helper Functions (defined at the end of the script) ---

function local_plotAndFindPath(ax, labeledGrid, detailedMap, regions, abstractGraph, ...
                               terrainGrid, terrainTypes, baseLegendHandles, baseLegendLabels, ...
                               gridSize, sectorSize)
    % Clear previous path and markers
    delete(findobj(ax, 'Tag', 'PathLine'));
    delete(findobj(ax, 'Tag', 'StartMarker'));
    delete(findobj(ax, 'Tag', 'GoalMarker'));
    delete(findobj(ax, 'Tag', 'StartText'));
    delete(findobj(ax, 'Tag', 'GoalText'));
    delete(findobj(ax, 'Tag', 'PathLegend')); % Clear old legend entry if present

    % Initialize plot handles to empty, just in case
    startPointPlotHandle = [];
    goalPointPlotHandle = [];
    pathLegendHandle = []; % This will be the dummy handle for the path legend entry

    % Re-prompt for start/goal
    title('DTA: Click START node, then GOAL node');
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
        return; % Exit if invalid
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

    % --- First Legend Update: After Start/Goal Selection ---
    % Combine base handles (static map elements) with newly created start/goal handles
    currentLegendHandles_temp = [baseLegendHandles, startPointPlotHandle, goalPointPlotHandle];
    currentLegendLabels_temp = [baseLegendLabels, {'Start Point'}, {'Goal Point'}];
    
    % Filter out any invalid handles before calling legend
    valid_idx = isvalid(currentLegendHandles_temp);
    legend(ax, currentLegendHandles_temp(valid_idx), currentLegendLabels_temp(valid_idx), ...
           'Location', 'eastoutside', 'FontSize', 9, 'AutoUpdate', 'off');

    title(sprintf('DTA: Pathfinding from (%d,%d) to (%d,%d)', start_row, start_col, goal_row, goal_col));

    % --- High-Level Pathfinding (A* on Abstract Graph) ---
    fprintf('Performing high-level A* search on the abstract graph...\n');

    startRegionID = labeledGrid(start_row, start_col);
    goalRegionID = labeledGrid(goal_row, goal_col);

    if isinf(terrainTypes{terrainGrid(start_row, start_col)}.cost) || isinf(terrainTypes{terrainGrid(goal_row, goal_col)}.cost)
        warning('Start or Goal point is on impassable terrain. Cannot find path. Please re-select or press ''r''.');
        return;
    end
    if startRegionID == 0 || goalRegionID == 0
        warning('Start or Goal point does not belong to a valid region. Please re-select or press ''r''.');
        return;
    end

    [abstract_path_regions, total_abstract_cost] = AStarAbstract(regions, abstractGraph, startRegionID, goalRegionID);

    if isempty(abstract_path_regions)
        fprintf('No high-level path found between regions %d and %d. DTA failed.\n', startRegionID, goalRegionID);
    else
        fprintf('High-level path found with total abstract cost: %.2f\n', total_abstract_cost);
        fprintf('Path (Region IDs): %s\n', mat2str(abstract_path_regions));
    end

    % --- Low-Level Path Refinement ---
    fprintf('Refining high-level path into detailed low-level path...\n');

    detailed_final_path = [];

    if isempty(abstract_path_regions)
        fprintf('No high-level path to refine.\n');
    else
        if length(abstract_path_regions) == 1 % Start and Goal are in the same region
            fprintf('Start and Goal are in the same region. Running direct A* on detailed map.\n');
            % Ensure Node objects are created with initial cost 0 for A* (important for AStar function)
            startNodeDetailed.cost = 0;
            goalNodeDetailed.cost = 0;
            final_detailed_segment = AStar(detailedMap, startNodeDetailed, goalNodeDetailed, 'ShowProcess', false); % No animation for this sub-path
            if ~isempty(final_detailed_segment)
                detailed_final_path = final_detailed_segment;
            end
        else % Path spans multiple regions
            current_detailed_node = startNodeDetailed; % Start from the user's selected point
            for i = 1:(length(abstract_path_regions) - 1)
                current_abstract_region_id = abstract_path_regions(i);
                next_abstract_region_id = abstract_path_regions(i+1);
                
                % Define transition points for low-level A*
                % For simplicity, connect the representative state of the current region
                % to the representative state of the next region.
                
                % If it's the first segment, path from actual start node to current region's rep state
                if i == 1
                    segment_start_node = startNodeDetailed;
                else
                    % Otherwise, use the representative state of the current abstract region
                    rep_state_curr_region = regions{current_abstract_region_id}.representative_state;
                    segment_start_node = Node(round(rep_state_curr_region(1)), round(rep_state_curr_region(2)));
                end
                
                % If it's the last segment, path from next region's rep state to actual goal node
                if i == (length(abstract_path_regions) - 1)
                    segment_goal_node = goalNodeDetailed;
                else
                    % Otherwise, use the representative state of the next abstract region
                    rep_state_next_region = regions{next_abstract_region_id}.representative_state;
                    segment_goal_node = Node(round(rep_state_next_region(1)), round(rep_state_next_region(2)));
                end
                fprintf('  Finding detailed path from (%d,%d) to (%d,%d) (Region %d to %d)...\n', ...
                        segment_start_node.row, segment_start_node.col, ...
                        segment_goal_node.row, segment_goal_node.col, ...
                        current_abstract_region_id, next_abstract_region_id);
                
                % Ensure Node objects are created with initial cost 0 for A*
                segment_start_node.cost = 0;
                segment_goal_node.cost = 0;
                % Run A* on the detailed map for this segment
                detailed_segment_path = AStar(detailedMap, segment_start_node, segment_goal_node, 'ShowProcess', false);
                
                if isempty(detailed_segment_path)
                    fprintf('    Could not find detailed path for segment from region %d to %d. DTA failed.\n', current_abstract_region_id, next_abstract_region_id);
                    detailed_final_path = []; % Indicate failure
                    break;
                end
                
                % Concatenate detailed path segments
                if i > 1 && ~isempty(detailed_final_path)
                    % Check if the first node of the new segment is the same as the last node of the previous segment
                    if detailed_segment_path(1,1) == detailed_final_path(end,1) && ...
                       detailed_segment_path(1,2) == detailed_final_path(end,2)
                        detailed_final_path = [detailed_final_path; detailed_segment_path(2:end,:)];
                    else
                        detailed_final_path = [detailed_final_path; detailed_segment_path];
                    end
                else
                    detailed_final_path = detailed_segment_path;
                end
            end
        end
    end
    % --- Second Legend Update ---
    if ~isempty(detailed_final_path)
        fprintf('Detailed DTA path found with total cost: %.2f\n', detailed_final_path(end, 3));
        % Plot the detailed path (shifted by -0.5 to go through cell centers as per earlier request)
        plot(ax, detailed_final_path(:,2) - 0.5, detailed_final_path(:,1) - 0.5, 'r-', 'LineWidth', 3, 'Tag', 'PathLine'); % Red path with tag
        
        % Add detailed path to legend - create a dummy handle for the legend entry
        pathLegendHandle = plot(ax, NaN, NaN, 'r-', 'LineWidth', 3, 'Tag', 'PathLegend');
        
        % Combine all relevant handles for the final legend
        currentLegendHandles_temp = [baseLegendHandles, startPointPlotHandle, goalPointPlotHandle, pathLegendHandle];
        currentLegendLabels_temp = [baseLegendLabels, {'Start Point'}, {'Goal Point'}, {'Detailed Path'}];
        
        % Filter out any invalid handles before calling legend
        valid_idx = isvalid(currentLegendHandles_temp);
        legend(ax, currentLegendHandles_temp(valid_idx), currentLegendLabels_temp(valid_idx), ...
               'Location', 'eastoutside', 'FontSize', 9, 'AutoUpdate', 'off');
    else
        fprintf('DTA failed to find a complete detailed path.\n');
        % If path failed, legend should only show base map elements + start/goal points
        currentLegendHandles_temp = [baseLegendHandles, startPointPlotHandle, goalPointPlotHandle];
        currentLegendLabels_temp = [baseLegendLabels, {'Start Point'}, {'Goal Point'}];
        
        % Filter out any invalid handles before calling legend
        valid_idx = isvalid(currentLegendHandles_temp);
        legend(ax, currentLegendHandles_temp(valid_idx), currentLegendLabels_temp(valid_idx), ...
               'Location', 'eastoutside', 'FontSize', 9, 'AutoUpdate', 'off');
    end
    title(sprintf('HPA*: Pathfinding from (%d,%d) to (%d,%d)', start_row, start_col, goal_row, goal_col));
    fprintf('HPA* pathfinding cycle complete.\n');
end
function local_keyPressCallback(src, event, plotAndFindPath_func, ax_handle)
    if strcmp(event.Key, 'r')
        fprintf('\n''r'' pressed. Resetting path, start, and goal. Keeping the map.\n');
        plotAndFindPath_func(ax_handle);
    end
end