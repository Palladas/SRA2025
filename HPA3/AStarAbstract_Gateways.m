function path_gateway_keys = AStarAbstract_Gateways(allGatewaysMap, abstractGraph, start_gateway_key, goal_gateway_key)

% Initialize data structures
open_set = containers.Map('KeyType', 'char', 'ValueType', 'any'); % Stores Node objects (representing gateways)
closed_set = containers.Map('KeyType', 'char', 'ValueType', 'logical'); % Marks visited gateway keys

% g_score: cost from start to current gateway
g_score = containers.Map('KeyType', 'char', 'ValueType', 'double');
g_score(start_gateway_key) = 0;

% f_score: g_score + heuristic (estimated cost from current to goal)
f_score = containers.Map('KeyType', 'char', 'ValueType', 'double');

% parent_map: Stores the predecessor gateway key for reconstructing the path
parent_map = containers.Map('KeyType', 'char', 'ValueType', 'char');

% Get Node objects for start and goal gateways for heuristic calculation
start_node_cell = allGatewaysMap(start_gateway_key);
start_node = start_node_cell{1};

goal_node_cell = allGatewaysMap(goal_gateway_key);
goal_node = goal_node_cell{1};

% Heuristic function (Euclidean distance between gateway nodes)
h = @(curr_gw_node) sqrt((curr_gw_node.row - goal_node.row)^2 + (curr_gw_node.col - goal_node.col)^2);

% Add start gateway to open_set
start_node_for_open_set = Node(start_node.row, start_node.col); % Create a copy for open_set if Node objects are passed by reference
start_node_for_open_set.cost = 0; % g_score
start_node_for_open_set.h_cost = h(start_node_for_open_set); % Heuristic
start_node_for_open_set.f_cost = start_node_for_open_set.cost + start_node_for_open_set.h_cost;
open_set(start_gateway_key) = {start_node_for_open_set};
f_score(start_gateway_key) = start_node_for_open_set.f_cost; % Redundant if Node stores f_cost, but good for map lookup

path_gateway_keys = {}; % Initialize empty path

while open_set.Count > 0
    % Find gateway with the lowest f_score in open_set
    current_key = '';
    min_f_score = inf;
    open_keys = keys(open_set);
    for k_idx = 1:length(open_keys)
        key = open_keys{k_idx};
        current_f = f_score(key); % Look up f_score from the map
        if current_f < min_f_score
            min_f_score = current_f;
            current_key = key;
        end
    end
    
    current_gateway_cell = open_set(current_key);
    current_gateway_node = current_gateway_cell{1};

    % If goal reached, reconstruct path
    if strcmp(current_key, goal_gateway_key)
        path_gateway_keys = reconstruct_path(parent_map, current_key);
        return;
    end

    % Move current from open_set to closed_set
    remove(open_set, current_key);
    closed_set(current_key) = true;

    % Explore neighbors
    if isKey(abstractGraph, current_key)
        neighbors = abstractGraph(current_key);
        for n_idx = 1:length(neighbors)
            neighbor_info = neighbors(n_idx);
            neighbor_key = neighbor_info.target_gateway_key;
            edge_cost = neighbor_info.cost;

            if isKey(closed_set, neighbor_key) && closed_set(neighbor_key)
                continue; % Already evaluated
            end

            tentative_g_score = g_score(current_key) + edge_cost;

            % Retrieve neighbor_node for heuristic if it's not in open_set or g_score is better
            neighbor_node_cell = allGatewaysMap(neighbor_key);
            neighbor_node = neighbor_node_cell{1};

            if ~isKey(open_set, neighbor_key) || tentative_g_score < g_score(neighbor_key)
                parent_map(neighbor_key) = current_key;
                g_score(neighbor_key) = tentative_g_score;
                f_score(neighbor_key) = tentative_g_score + h(neighbor_node);

                if ~isKey(open_set, neighbor_key)
                    neighbor_node_for_open_set = Node(neighbor_node.row, neighbor_node.col);
                    neighbor_node_for_open_set.cost = tentative_g_score;
                    neighbor_node_for_open_set.h_cost = h(neighbor_node_for_open_set);
                    neighbor_node_for_open_set.f_cost = f_score(neighbor_key); % Sync with f_score map
                    open_set(neighbor_key) = {neighbor_node_for_open_set};
                else
                end
            end
        end
    end
end

% If loop finishes, no path found
path_gateway_keys = {};
fprintf('No abstract path found between gateways %s and %s.\n', start_gateway_key, goal_gateway_key);

end


% Helper function to reconstruct path
function path = reconstruct_path(parent_map, current_key)
    path = {current_key};
    while isKey(parent_map, current_key)
        current_key = parent_map(current_key);
        path = [current_key, path];
    end
end