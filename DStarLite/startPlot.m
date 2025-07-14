function [outputArg1,outputArg2] = startPlot(inputArg1,inputArg2)
%STARTPLOT Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    inputArg1
    inputArg2
end

arguments (Output)
    outputArg1
    outputArg2
end
Color = 'g';
newObstColor = [0.8500 0.3250 0.0980];

h1 = plot(x(1), y(1), 'o', 'MarkerFaceColor', Color, 'MarkerEdgeColor', Color(1, :), 'MarkerSize', 4);
h2 = plot(x(1:2), y(1:2), 'Color', Color, 'LineWidth', 2);
pause(0.2)

end