function currPath = genCurrPath(Start,Model,G,currentDir)
%GENCURRPATH Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    Start
    Model
    G
    currentDir
end


currPath.nodeNumbers = [Start.nodeNumber];
it = 0;

while Start.nodeNumber ~= Model.Robot.targetNode
    it = it + 1;
    if it >Model.Map.lim
        currPath = -1;
        fprintf("No Possible Path")
        return
    end
    sucNodes = Model.Successors{Start.nodeNumber,1};

    dTheta = turnCost(Start.nodeNumber, sucNodes, Model, currentDir);
    [~, sortInds] = sortrows([G(sucNodes) + Model.Successors{Start.nodeNumber, 2}; abs(dTheta)]');
    Start.nodeNumber = sucNodes(sortInds(1));
    currentDir = currentDir + dTheta(sortInds(1));

    Start.coords = nodes2coords(Start.nodeNumber, Model);
    currPath.nodeNumbers(end + 1) = Start.nodeNumber;

end

currPath.coords = nodes2coords(currPath.nodeNumbers, Model);



end