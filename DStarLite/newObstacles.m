function Model = newObstacles(Model)
    % create newObstacles: inserted in the sppecified time t.

    x = [14, 16, 11];
    y = [14, 20, 22];
    count = numel(x);
    nodeNumbers = zeros(count);

    for i = 1:count
        nodeNumbers(i) = (y(i) - Model.Map.yMin) * Model.Map.nX + x(i) - Model.Map.xMin + 1;
    end

    % NewObsts
    NewObsts.x = x;
    NewObsts.y = y;
    NewObsts.count = count;
    NewObsts.nodeNumbers = nodeNumbers;

    % update Model
    Model.NewObsts = NewObsts;

end
