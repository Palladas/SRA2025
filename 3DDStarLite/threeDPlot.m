clear;
clc;
close all;


displayPlot = false;
genTerrain = true;
randomST = true;
figure;
hold on;
xlabel('Grid Size');
ylabel('Average Nodes Visited');
title('Average Nodes Visited vs Number of Dynamic Obstacles');
times = zeros(51,1);
gridSize = 64;
maxObst = 50;

for numObst = 0:maxObst
    t = 0;
    c = 0;
    for trial = 1:30
        [Model, Sol] = threeDDStar(gridSize, displayPlot, genTerrain, randomST, numObst);
        if ~isfield(Model, 'distType')
            continue
        end
        t = t + Sol.nodesVisited;
        c = c+1;
        disp(numObst)
        disp(trial)
    end
    times(numObst+1) = t/c;
end 
plot([0:maxObst],times)


%[Model, Sol] = threeDDStar(64, true, true, true, 5);

%{
[terrainGrid, costGrid] = terrainGen(32);
altitudeGrid = altitudeGen(32);
visAlti(altitudeGrid,terrainGrid)
%}