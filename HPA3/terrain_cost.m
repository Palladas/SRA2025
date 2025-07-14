function c = terrain_cost(terrainType, terrainTypes)
% TERRAIN_COST Returns the traversal cost of a given terrain type.
% Input:
% - terrainType: integer terrain ID
% - terrainTypes: array of terrain type structs
    if terrainType > 0 && terrainType <= length(terrainTypes)
        c = terrainTypes(terrainType).cost;
    else
        c = Inf;
    end
end