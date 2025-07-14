function obstPlot = plotNewObst(Model,newObstColor,t,Start)
%PLOTNEWOBST Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    Model
    newObstColor
    t
    Start
end

arguments (Output)
    obstPlot
end

if isfield(Model, 'NewObsts')

            for iNewObst = 1:Model.NewObsts.count
                if Model.NewObsts.t(iNewObst) == t
                    plot(Model.NewObsts.x(iNewObst), Model.NewObsts.y(iNewObst), 'o', 'MarkerSize', 5, ...
                        'MarkerFaceColor', newObstColor, 'MarkerEdgeColor', newObstColor);
                end

            end

end


end