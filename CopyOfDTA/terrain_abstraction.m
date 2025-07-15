% --- 2. Build the Abstract Graph ---
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