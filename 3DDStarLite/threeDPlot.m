clear;
clc;
close all;

%%{
displayPlot = false;
genTerrain = true;
randomST = true;
figure;
hold on;
xlabel('Grid Size');
ylabel('Average Nodes Visited');
title('Average Nodes Visited vs Grid Size');
times = zeros(18,1);

for gridSize = 3:20
    t = 0;
    c = 0;
    for trial = 1:30
        [Model, Sol] = threeDDStar(10*gridSize, displayPlot, genTerrain, randomST);
        if ~isfield(Model, 'distType')
            continue
        end
        t = t + Sol.nodesVisited
        c = c+1
        disp(10*gridSize)
    end
    times(gridSize-2) = t/c;
end 
plot(10*[3:20],times)
%%}

%[Model, Sol] = threeDDStar(30, true, true, true);