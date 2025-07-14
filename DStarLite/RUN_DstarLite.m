% D*Lite: Path Planning Algorithm - MATLAB

% Initialization
clc
clear
close

%% settings
Model.expandMethod = 'heading'; % random or heading
Model.distType = 'euclidean'; % euclidean or manhattan;
Model.adjType = '8adj'; % 4adj or 8adj

sample_model_name = "Obstacle1";
Model = createModelSamples(sample_model_name, Model);
% Complete Base Model for DstarLite
Model = createModelDstarLite(Model);

% add dynamic obstacles
Model = newObstacles(Model);
plotModel(Model, false)

%% # optimal path by Astar
tic
[Model, Path] = myDstarLite(Model);
Sol = Path;
Sol.runTime = toc;
Sol.cost = calCostL(Sol.coords);
Sol.smoothness = calSmoothnessbyDir(Sol);

%% display data and plot solution
disp(['run time for path= ' num2str(Sol.runTime)])
disp(Sol)

%% clear temporal data
clear adj_type dist_type
