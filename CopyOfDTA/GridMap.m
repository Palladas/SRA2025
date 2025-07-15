% Grid map in graph form
% Ref: https://www.redblobgames.com/pathfinding/grids/graphs.html
% XiaoCY, 2024-10-23
%%
classdef GridMap < handle
    properties
        terrainGrid = [];       % Stores terrain type indices (e.g., 1, 2, 3...)
        terrainTypes = {};      % Stores the properties of each terrain type (colors, costs)
        obstacleCost = inf;     % The cost value considered an obstacle
    end
    methods
        % constructor
        % obj = GridMap(terrainGridData, terrainTypesData, obstacleVal)
        % terrainGridData: A 2D matrix where each element is an index into terrainTypesData.
        % terrainTypesData: A cell array of structs, where each struct has 'name', 'color', 'cost' fields.
        % obstacleVal: (Optional) The cost value considered an obstacle (defaults to inf).
        function obj = GridMap(terrainGridData, terrainTypesData, obstacleVal)
            if nargin < 3
                obstacleVal = inf; % Default obstacle value
            end
            obj.terrainGrid = terrainGridData;
            obj.terrainTypes = terrainTypesData;
            obj.obstacleCost = obstacleVal;
        end

        % return neighbors and cost for a given node
        function neighbor_nodes = neighbors(obj, node)
            neighbor_nodes = [];
            [Ny, Nx] = size(obj.terrainGrid);

            % Ensure the current node is within grid bounds (should be handled by A* calling)
            if node.row < 1 || node.row > Ny || node.col < 1 || node.col > Nx
                return
            end

            % Define 4-directional moves (up, left, down, right)
            dr = [-1, 0, 1, 0];
            dc = [0, -1, 0, 1];

            for k = 1:length(dr)
                row = node.row + dr(k);
                col = node.col + dc(k);

                % Check if neighbor is within grid bounds
                if row >= 1 && row <= Ny && col >= 1 && col <= Nx
                    % Get the terrain type index of the neighbor cell
                    neighborTerrainTypeIdx = obj.terrainGrid(row, col);
                    % Get the actual cost of entering the neighbor cell from terrainTypes
                    neighborCost = obj.terrainTypes{neighborTerrainTypeIdx}.cost;

                    % Check if the neighbor cell is not an obstacle (i.e., its cost is less than obstacleCost)
                    if neighborCost < obj.obstacleCost
                        % The cost to reach this neighbor node is current_node_cost + cost_to_enter_neighbor_cell
                        cost_to_reach_neighbor = node.cost + neighborCost;
                        neighbor_nodes = [neighbor_nodes; Node(row, col, cost_to_reach_neighbor, node)];
                    end
                end
            end
        end

        % show map visualization
        % obj.show(Name, Value)
        % Parameters:
        %   'GridWidth': Line width for grid cells.
        %   'GridAlpha': Transparency for grid lines.
        %   'ShowPath': A matrix [row, col] of nodes forming a path to plot.
        %   'PathWidth': Line width for the path.
        %   'PathColor': Color for the path.
        %   'ShowValue': Boolean, true to display cell costs as text.
        %   'FontSize': Font size for displayed cell costs.
        function varargout = show(obj, varargin)
            p = inputParser;
            p.addParameter('GridWidth', 0.5);
            p.addParameter('GridAlpha', 0.3);
            p.addParameter('ShowPath', []);
            p.addParameter('PathWidth', 5);
            p.addParameter('PathColor', [0.5569, 0.8118, 0.7882]);
            p.addParameter('ShowValue', false); % For showing costs on cells
            p.addParameter('FontSize', 10); % Adjusted default for better visibility
            p.parse(varargin{:});
            
            [Ny, Nx] = size(obj.terrainGrid);
            xbase = [-0.5, 0.5, 0.5, -0.5]; % For patch coordinates
            ybase = [-0.5, -0.5, 0.5, 0.5];
            
            go = gobjects(Ny, Nx); % graphics objects array
            
            % Get the current figure handle (if no figure is open, MATLAB creates one)
            fig_handle = gcf;
            % Clear the current axes before drawing to ensure a clean slate
            cla(gca); 
            
            ax_handle = gca; % Get current axes handle
            axis(ax_handle, 'off', 'equal', 'ij'); % Apply axis settings (turn off axis, equal aspect, invert y-axis)
            hold(ax_handle, 'on'); % Keep content when adding new plots
            
            % Draw individual cells based on terrainType colors
            for x = 1:Nx
                for y = 1:Ny
                    currentTerrainTypeIdx = obj.terrainGrid(y, x);
                    currentColor = obj.terrainTypes{currentTerrainTypeIdx}.color;
                    
                    go(y, x) = patch(ax_handle, x+xbase, y+ybase, currentColor, ...
                        'EdgeColor', [0.8 0.8 0.8], 'LineWidth', p.Results.GridWidth, ...
                        'FaceAlpha', 1.0, 'EdgeAlpha', p.Results.GridAlpha); % Use full alpha for terrain colors
                    
                    % Optionally display cell costs as text
                    if p.Results.ShowValue
                        cellCost = obj.terrainTypes{currentTerrainTypeIdx}.cost;
                        if isinf(cellCost)
                            costStr = 'Inf';
                        else
                            costStr = sprintf('%.1f', cellCost);
                        end
                        text(ax_handle, x, y, costStr, 'FontSize', p.Results.FontSize, ...
                            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                            'Color', 'k', 'FontWeight', 'bold'); % Black text for visibility
                    end
                end
            end
            
            % Plot path if provided
            if ~isempty(p.Results.ShowPath)
                % Assuming p.Results.ShowPath contains [row, col] coordinates of nodes
                % For plotting, MATLAB expects (x,y) which corresponds to (col, row)
                path_cols = p.Results.ShowPath(:,2);
                path_rows = p.Results.ShowPath(:,1);
                
                plot(ax_handle, path_cols, path_rows, ...
                    'Color', p.Results.PathColor, 'LineWidth', p.Results.PathWidth);
            end
            
            % Return graphics objects if requested
            if nargout > 0
                varargout{1} = go;
            end
        end
    end
end