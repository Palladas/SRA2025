classdef GridMap
    properties
        Grid             % The 2D grid itself (e.g., terrain type indices)
        TerrainInfo      % Cell array of terrain type structs (cost, color, etc.)
        AltitudeGrid     % 2D grid of altitude values
        ObstacleCost     % Cost for impassable cells (e.g., Inf)
        SectorSize       % Size of sectors (for HPA* abstraction)
        GridSize         % Store gridSize for convenience
        TerrainColors    % Cache terrain colors for quick access in show method
    end
    
    methods
        % Constructor
        function obj = GridMap(grid, terrainInfo, altitudeGrid, obstacleCost, sectorSize)
            obj.Grid = grid;
            obj.TerrainInfo = terrainInfo;
            obj.AltitudeGrid = altitudeGrid;
            obj.GridSize = size(grid, 1);
            
            if nargin < 4 || isempty(obstacleCost)
                obj.ObstacleCost = inf;
            else
                obj.ObstacleCost = obstacleCost;
            end
            
            if nargin < 5 || isempty(sectorSize)
                obj.SectorSize = 1;
            else
                obj.SectorSize = sectorSize;
            end

            % Cache terrain colors for show method
            obj.TerrainColors = vertcat(terrainInfo{:}).color; % Assumes all terrainInfo structs have a 'color' field
        end
        
        % Method to calculate cost based on altitude difference and terrain type
        function gradeCost = calculateGradeCost(obj, altitude_diff, terrain_properties)
            if isinf(terrain_properties.cost)
                gradeCost = inf;
                return;
            end

            gradeCost = 0;
            if altitude_diff > 0 % Moving uphill
                gradeCost = terrain_properties.climbFactor * (1 + terrain_properties.climbSteepnessFactor * altitude_diff);
            elseif altitude_diff < 0 % Moving downhill
                gradeCost = terrain_properties.descentFactor * (1 - terrain_properties.descentGentlenessFactor * abs(altitude_diff));
                gradeCost = max(0.1, gradeCost);
            end
        end

        % Method to get total cost to traverse from one cell to another
        function total_cost = getCost(obj, from_row, from_col, to_row, to_col)
            currentTerrainTypeIdx = obj.Grid(to_row, to_col);
            currentTerrainProps = obj.TerrainInfo{currentTerrainTypeIdx};
            
            base_terrain_cost = currentTerrainProps.cost;

            if isinf(base_terrain_cost)
                total_cost = inf;
                return;
            end
            
            alt_from = obj.AltitudeGrid(from_row, from_col);
            alt_to = obj.AltitudeGrid(to_row, to_col);
            altitude_diff = alt_to - alt_from;

            grade_cost_component = obj.calculateGradeCost(altitude_diff, currentTerrainProps);
            
            if isinf(grade_cost_component)
                total_cost = inf;
                return;
            end

            dx = abs(from_col - to_col);
            dy = abs(from_row - to_row);
            
            distance_multiplier = 1;
            if dx == 1 && dy == 1 % Diagonal move
                distance_multiplier = sqrt(2);
            elseif dx == 0 && dy == 0 % Should not happen for movement
                distance_multiplier = 0;
            end

            total_cost = (base_terrain_cost + grade_cost_component) * distance_multiplier; 
        end

        % Get valid neighbors for A*
        function neighbors = neighbors(obj, currentNode)
            neighbors = [];
            
            current_row = currentNode.row;
            current_col = currentNode.col;
            current_cost = currentNode.cost;
            current_depth = currentNode.depth;

            dr = [-1, -1, -1,  0, 0,  1, 1, 1];
            dc = [-1,  0,  1, -1, 1, -1, 0, 1];
            
            for k = 1:length(dr)
                n_row = current_row + dr(k);
                n_col = current_col + dc(k);
                
                if n_row >= 1 && n_row <= obj.GridSize && ...
                   n_col >= 1 && n_col <= obj.GridSize
                    
                    cost_to_neighbor = obj.getCost(current_row, current_col, n_row, n_col);
                    
                    if isinf(cost_to_neighbor)
                        continue;
                    end
                    
                    neighborNode = Node(n_row, n_col);
                    neighborNode.cost = current_cost + cost_to_neighbor;
                    neighborNode.parent = currentNode;
                    neighborNode.depth = current_depth + 1;
                    
                    neighbors = [neighbors, neighborNode];
                end
            end
        end

        % show method for A* visualization
        function h_cells = show(obj, varargin)
            p = inputParser;
            p.KeepUnmatched = true; % Keep any unmatched parameters
            p.addOptional('AxesHandle', gca, @(x) isscalar(x) && isgraphics(x, 'axes')); % Optional axes handle, default to gca
            p.addParameter('ShowPath', [], @(x) isempty(x) || ismatrix(x)); % Optional ShowPath parameter
           
            p.parse(varargin{:});

            ax = p.Results.AxesHandle; % Get the axes handle from the parser results
            path_data_to_plot = p.Results.ShowPath; % Get the ShowPath data from the parser results
            
            hold(ax, 'on'); % Keep previous plots (like sector lines, nodes)

            % Initialize h_cells as a matrix of graphics handles
            h_cells = gobjects(obj.GridSize, obj.GridSize); 
            % Draw cells and store their handles
            for r_idx = 1:obj.GridSize
                for c_idx = 1:obj.GridSize
                    currentTerrainIndex = obj.Grid(r_idx, c_idx);
                    currentColor = obj.TerrainInfo{currentTerrainIndex}.color; % Use TerrainInfo for color
                    % Create a rectangle for each cell and store its handle
                    h_cells(r_idx, c_idx) = rectangle(ax, 'Position', [c_idx-1, r_idx-1, 1, 1], ...
                                             'FaceColor', currentColor, ...
                                             'EdgeColor', [0.8 0.8 0.8], ... % Light gray grid lines
                                             'LineWidth', 0.1, ...
                                             'Tag', 'GridCell'); % Add a tag for easy identification
                end
            end
            
            % Plot path if ShowPath is provided by AStar
            if ~isempty(path_data_to_plot)
                plot(ax, path_data_to_plot(:,2) - 0.5, path_data_to_plot(:,1) - 0.5, 'r-', 'LineWidth', 3, 'Tag', 'AStarPath');
            end
            axis(ax, 'equal'); 
            xlim(ax, [0, obj.GridSize]); 
            ylim(ax, [0, obj.GridSize]); 
            set(ax, 'YDir', 'normal');
            set(ax, 'XTick', 0:obj.GridSize, 'YTick', 0:obj.GridSize); 
            grid(ax, 'on'); 
            box(ax, 'on');
            
            hold(ax, 'off');
        end
    end
end