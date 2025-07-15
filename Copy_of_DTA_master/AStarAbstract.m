function [abstract_path_regions, total_abstract_cost] = AStarAbstract(regions, abstractGraph, startRegionID, goalRegionID)

    openSet = containers.Map('KeyType', 'double', 'ValueType', 'double');
    openSet(startRegionID) = 0 + heuristic(startRegionID, goalRegionID, regions); 

    gScore = containers.Map('KeyType', 'double', 'ValueType', 'double');
    gScore(startRegionID) = 0;

    cameFrom = containers.Map('KeyType', 'double', 'ValueType', 'double'); 

    evaluated = false(length(regions), 1); % Tracks if a region is in closedSet/has been processed

    abstract_path_regions = [];
    total_abstract_cost = inf;

    while ~isempty(openSet)
        % Get region with lowest f_cost from openSet
        keys = cell2mat(openSet.keys);
        values = cell2mat(openSet.values);
        [~, minIdx] = min(values);
        currentRegionID = keys(minIdx);
        
        remove(openSet, currentRegionID);

        if currentRegionID == goalRegionID
            % Reconstruct path
            path = [];
            current = goalRegionID;
            while isKey(cameFrom, current)
                path = [current, path]; 
                current = cameFrom(current);
            end
            abstract_path_regions = [current, path];
            total_abstract_cost = gScore(goalRegionID);
            return; % path found!
        end

        evaluated(currentRegionID) = true; % Add to closed set conceptually

        % Explore neighbors
        neighbors_data = abstractGraph{currentRegionID};
        for i = 1:length(neighbors_data)
            neighborRegionID = neighbors_data(i).target_region_id;
            edgeCost = neighbors_data(i).cost;

            if isinf(edgeCost) % Impassable abstract edge
                continue;
            end

            if evaluated(neighborRegionID) 
                continue;
            end

            tentative_gScore = gScore(currentRegionID) + edgeCost;

            % If neighbor not in gScore or new path is better
            if ~isKey(gScore, neighborRegionID) || tentative_gScore < gScore(neighborRegionID)
                cameFrom(neighborRegionID) = currentRegionID;
                gScore(neighborRegionID) = tentative_gScore;
                f_cost = tentative_gScore + heuristic(neighborRegionID, goalRegionID, regions);
                openSet(neighborRegionID) = f_cost; % Add/Update in openSet
            end
        end
    end
end

function h = heuristic(regionID1, regionID2, regions)
% Euclidean distance between representative states
    if isempty(regions{regionID1}) || isempty(regions{regionID2})
        h = inf; % Should not happen with valid IDs
        return;
    end
    state1 = regions{regionID1}.representative_state;
    state2 = regions{regionID2}.representative_state;
    h = sqrt((state1(1) - state2(1))^2 + (state1(2) - state2(2))^2);
end