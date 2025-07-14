\classdef Node
    properties
        row         % Row coordinate of the node in the grid
        col         % Column coordinate of the node in the grid
        cost        % Cost to reach this node from the start node (g-score)
        h_cost      % Heuristic cost: Estimated cost from this node to goal (h-score)
        f_cost      % Total cost: g_cost + h_cost (f-score)
        parent      % Parent Node object in the path
        depth       % Depth of the node in the path (number of steps from start)
        key         % Unique identifier for the node (e.g., 'row_col')
    end
    
    methods
        function obj = Node(row, col)
            obj.row = row;
            obj.col = col;
            obj.cost = inf; % Initialize g-score to infinity
            obj.h_cost = 0; % Initialize h-score
            obj.f_cost = inf; % Initialize f-score (g+h) to infinity
            obj.parent = []; % Initialize parent as empty
            obj.depth = 0; % Initialize depth as 0
            obj.key = sprintf('%d_%d', row, col); % Unique key for easy lookup
        end
        
        % Overload the eq (equality) operator for Node objects
        function tf = eq(obj1, obj2)
            tf = (obj1.row == obj2.row) && (obj1.col == obj2.col);
        end
        
        % Overload the ne (not equal) operator for Node objects
        function tf = ne(obj1, obj2)
            tf = ~(obj1 == obj2);
        end
        
        % Overload the lt (less than) operator for comparing nodes (e.g., in priority queue)
        % This is used by the PriorityQueue if it directly compares Node objects by cost
        % For A*, this comparison should ideally be based on f_cost
        function tf = lt(obj1, obj2)
            % For A*, we generally compare nodes by their f_cost
            tf = obj1.f_cost < obj2.f_cost; 
        end
    end
end