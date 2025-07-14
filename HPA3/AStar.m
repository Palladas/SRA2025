function astar_path = AStar(map, start, goal, varargin)
    % Parse input parameters
    p = inputParser;
    p.KeepUnmatched = true; % Allows map.show to receive unmatched parameters
    p.addParameter('ShowProcess', false);
    p.addParameter('FrameRate', 0.1);
    p.addParameter('Filename', 'AStarProcess.gif');
    p.addParameter('CurrentColor', [1.0000, 0.7451, 0.4784]); % Color for current node
    p.addParameter('OpenColor', [0.3333, 0.6588, 0.4078]);    % Color for nodes in openSet
    p.addParameter('ClosedColor', [0.2980, 0.4471, 0.6902]);   % Color for nodes processed
    p.addParameter('LineWidth', 2);
    p.addParameter('MarkerSize', 15);
    p.parse(varargin{:});
    
    plot_inited = false; % Flag for GIF writing

    % --- Heuristic function definition ---
    heuristic = @(node) sqrt((goal.row - node.row)^2 + (goal.col - node.col)^2);

    % --- Initialize Start Node for A* Search ---
    % Ensure the start node's A* specific costs (g, h, f) and parent are correctly set.
    start.cost = 0; % g_cost: cost from start to itself is 0
    start.h_cost = heuristic(start); % h_cost: estimated cost from start to goal
    start.f_cost = start.cost + start.h_cost; % f_cost: total estimated cost
    start.parent = []; % Start node has no parent
    start.depth = 1; % Depth of the start node

    % The 'goal' node's specific costs (cost, h_cost, f_cost, parent, depth)
    % will be discovered and updated by the A* algorithm as it runs.

    % --- A* Data Structures ---
    % openSet: A PriorityQueue to store nodes to be evaluated, ordered by f_cost.
    %          Each element pushed is a Node object, with its f_cost as priority.
    openSet = PriorityQueue; 
    openSet.push(start, start.f_cost);

    % cameFrom: A map to reconstruct the path. Stores parent node for each node.
    %           Key: node.key (char), Value: parent Node object.
    cameFrom = containers.Map('KeyType', 'char', 'ValueType', 'any'); 

    % gScore: A map to store the cost of the cheapest path from start to each node found so far.
    %         Key: node.key (char), Value: g_cost (double).
    gScore = containers.Map('KeyType', 'char', 'ValueType', 'double');
    gScore(start.key) = 0; % The g_score for the start node is 0.

    astar_path = []; % Initialize the output path as empty.

    % --- A* Search Loop ---
    while openSet.size > 0
        % Retrieve the node with the lowest f_cost from the openSet.
        current = openSet.pop;

        % Optimization: If a better path to 'current' was already processed (i.e., 'current' is a stale duplicate in PQ), skip.
        % This check is crucial if the PriorityQueue can contain multiple entries for the same node.
        if isKey(gScore, current.key) && current.cost > gScore(current.key)
             continue; 
        end

        % Check if the current node is the goal node.
        if current.row == goal.row && current.col == goal.col
            goal = current; % Update the 'goal' object with the final cost and parent for path reconstruction.
            break; % Path found! Exit the search loop.
        end
        
        % Get all valid neighbors of the current node from the map.
        neighbors = map.neighbors(current); 
        
        % Evaluate each neighbor.
        for k = 1:length(neighbors)
            neighbor = neighbors(k);
            
            % Calculate the g_score (cost from start) if traversing from 'current' to 'neighbor'.
            % neighbor.cost (returned by map.neighbors) is the cost to move FROM 'current' TO 'neighbor'.
            tentative_gScore = gScore(current.key) + neighbor.cost; 
            
            % If this path to 'neighbor' is better (lower tentative_gScore) than any previously found path,
            % or if 'neighbor' has not been visited yet:
            if ~isKey(gScore, neighbor.key) || tentative_gScore < gScore(neighbor.key)
                % Update neighbor's properties with the new, better path.
                neighbor.parent = current;        % Set current node as neighbor's parent.
                neighbor.cost = tentative_gScore; % Update neighbor's g_cost.
                neighbor.depth = current.depth + 1; % Update neighbor's depth.
                
                % Calculate neighbor's h_cost and f_cost based on the new g_cost.
                neighbor.h_cost = heuristic(neighbor); % Re-calculate heuristic.
                neighbor.f_cost = neighbor.cost + neighbor.h_cost; % Calculate total estimated cost.
                
                % Update the gScore map with the new best cost to this neighbor.
                gScore(neighbor.key) = tentative_gScore;
                
                % Add the neighbor to the openSet (or update its priority if it's already there).
                openSet.push(neighbor, neighbor.f_cost);
                
                % Record the parent in the cameFrom map for path reconstruction.
                cameFrom(neighbor.key) = current; 
            end
        end

        % --- Visualization of A* Process (Optional, enabled by 'ShowProcess' parameter) ---
        if p.Results.ShowProcess
            clf; % Clear current figure
            go = map.show(p.Unmatched); % Show the map, passing unmatched parameters (e.g., for terrain colors)
            
            % Plot Start and Goal markers (adjust -0.5 for cell centering)
            plot(start.col - 0.5, start.row - 0.5, 'color', p.Results.CurrentColor, 'Marker', 'o', 'MarkerSize', p.Results.MarkerSize);
            plot(goal.col - 0.5, goal.row - 0.5, 'color', p.Results.CurrentColor, 'Marker', 'pentagram', 'MarkerSize', p.Results.MarkerSize);
            
            % Plot nodes that have been 'closed' (i.e., their gScore is finalized/best found so far)
            % These are represented by nodes present in the gScore map.
            gscore_keys = gScore.keys;
            for k_idx = 1:length(gscore_keys)
                key = gscore_keys{k_idx};
                % Exclude start, goal, and the currently processing node from this general closed coloring,
                % as they have specific markers/colors.
                if strcmp(key, start.key) || strcmp(key, goal.key) || strcmp(key, current.key)
                    continue;
                end
                
                % Extract row and column from the node key.
                parts = strsplit(key, '_');
                r = str2double(parts{1});
                c = str2double(parts{2});

                % Apply closed color and line style to the grid cell.
                go(r, c).FaceColor = p.Results.ClosedColor;
                go(r, c).EdgeColor = p.Results.ClosedColor;
                go(r, c).EdgeAlpha = 1;
                go(r, c).LineWidth = p.Results.LineWidth;
            end

            % Plot nodes currently in the 'openSet' (nodes to be evaluated)
            % Iterates through the PriorityQueue's internal data structure.
            for k_idx = 1:openSet.size
                node_in_open = openSet.data{k_idx, 1}; % Assuming data{k,1} is the Node object.
                % Exclude start, goal, and current node.
                if strcmp(node_in_open.key, start.key) || strcmp(node_in_open.key, goal.key) || strcmp(node_in_open.key, current.key)
                    continue; 
                end
                % Apply open color and line style.
                go(node_in_open.row, node_in_open.col).FaceColor = p.Results.OpenColor;
                go(node_in_open.row, node_in_open.col).EdgeColor = p.Results.OpenColor;
                go(node_in_open.row, node_in_open.col).EdgeAlpha = 1;
                go(node_in_open.row, node_in_open.col).LineWidth = p.Results.LineWidth;
            end

            % Highlight the 'current' node being processed.
            go(current.row, current.col).FaceColor = p.Results.CurrentColor;
            go(current.row, current.col).EdgeColor = p.Results.CurrentColor;
            go(current.row, current.col).EdgeAlpha = 1;
            go(current.row, current.col).LineWidth = p.Results.LineWidth;

            % Pause for animation and capture frame for GIF.
            pause(p.Results.FrameRate);
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256);
            if ~plot_inited
                plot_inited = true;
                imwrite(imind, cm, p.Results.Filename, 'gif', 'Loopcount', inf);
            else
                imwrite(imind, cm, p.Results.Filename, 'gif', 'WriteMode', 'append', 'DelayTime', p.Results.FrameRate);
            end
        end
    end

    % --- Path Reconstruction ---
    if isempty(cameFrom) || (~isKey(cameFrom, goal.key) && ~(start.row == goal.row && start.col == goal.col))
        astar_path = [];
        return;
    end
    
    % Reconstruct path from goal to start using parent pointers
    path_nodes = {}; % Initialize path_nodes as an EMPTY CELL ARRAY
    current_node = goal;

    % Handle the start == goal case separately, as cameFrom won't have goal.key
    if current_node.row == start.row && current_node.col == start.col
        path_nodes = {current_node}; % If start==goal, path is just the goal node in a cell
    else
        while isKey(cameFrom, current_node.key)
            % FIX: Ensure current_node is wrapped in a cell array {} before prepending
            % This makes sure path_nodes remains a cell array throughout.
            path_nodes = [{current_node}, path_nodes]; %#ok<AGROW>
            current_node = cameFrom(current_node.key); % Move to parent
        end
        % FIX: Add the start node itself (which has no parent in cameFrom) as a cell
        path_nodes = [{current_node}, path_nodes]; 
    end
    
    % Convert path_nodes (cell array of Node objects) to desired [row, col, cost] format
    astar_path = zeros(length(path_nodes), 3);
    for k = 1:length(path_nodes)
        % This line now correctly indexes a cell array `path_nodes`
        astar_path(k, :) = [path_nodes{k}.row, path_nodes{k}.col, path_nodes{k}.cost];
    end

    % --- Final Path Visualization ---
    if p.Results.ShowProcess
        clf; % Clear the figure for the final path display
        map.show('ShowPath', astar_path, p.Unmatched); % Show the map with the final path
        
        % Plot Start and Goal markers on the final path display.
        plot(start.col - 0.5, start.row - 0.5, 'color', p.Results.CurrentColor, 'Marker', 'o', 'MarkerSize', p.Results.MarkerSize);
        plot(goal.col - 0.5, goal.row - 0.5, 'color', p.Results.CurrentColor, 'Marker', 'pentagram', 'MarkerSize', p.Results.MarkerSize);
        
        % Capture final frame for GIF.
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        imwrite(imind, cm, p.Results.Filename, 'gif', 'WriteMode', 'append', 'DelayTime', 1);
    end
end