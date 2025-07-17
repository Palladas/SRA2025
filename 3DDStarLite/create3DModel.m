function Model = create3DModel(Model)
    % add neccessary data for Dstar Lite to Astar Base Model

    disp('Complete Model for DstarLite');

    %% edge costs, G, RHS
    Nodes = Model.Nodes;

    switch Model.adjType
        case '4adj'
            ixy = [1 0; 0 1; 0 -1; -1 0];
            nAdj = 4;
        case '8adj'
            ixy = [1 0; 0 1; 0 -1; -1 0; 1 1; -1 -1; 1 -1; -1 1];
            nAdj = 8;
    end

    % euclidean manhattan
    switch Model.distType
        case 'manhattan'
            edgeLength = 2;
        case 'euclidean'
            edgeLength = sqrt(2);
    end

    % prio type
    kS = 0;
    kE = 0;
    switch Model.prioType
        case 'speed'
            kS = 0.9;
            kE = 0.1;
        case 'balanced'
            kS = 0.5;
            kE = 0.5;
        case 'eff'
            kS = 0.1;
            kE = 0.9;
    end

    nNodes = Model.Nodes.count;
    Successors = cell(nNodes, 2);
    Predecessors = cell(nNodes, 2);
    cellSteep = zeros(Model.Map.nY,Model.Map.nX);

    for iNode = 1:nNodes
        xNode = Nodes.cord(1, iNode);
        yNode = Nodes.cord(2, iNode);
        zNode = Model.Alti(yNode,xNode);

        for iAdj = 1:nAdj
            ix = ixy(iAdj, 1);
            iy = ixy(iAdj, 2);
            newX = xNode + ix;
            newY = yNode + iy;
            

            % check if the Node is within array bound
            if (newX >= Model.Map.xMin && newX <= Model.Map.xMax) && (newY >= Model.Map.yMin && newY <= Model.Map.yMax)
                newZ = Model.Alti(newY,newX);
                newNodeNumber = iNode + ix + iy * (Model.Map.nX);

                dist = sqrt((xNode-newX)^2+(yNode-newY)^2+(zNode-newZ)^2);
                steepness = atan(abs(zNode-newZ)/ sqrt((xNode-newX)^2+(yNode-newY)^2));
                terrCost = (Model.TerrCost(newY,newX)+Model.TerrCost(xNode,yNode))/2;

                cost = terrCost*(kS*dist + kE*steepness);
                
                if Model.Obsts.count>0
                    if ~any(newNodeNumber == Model.Obsts.nodeNumber)
                        Successors{iNode, 1} = [Successors{iNode}, newNodeNumber];
                        Predecessors{newNodeNumber, 1} = [Predecessors{newNodeNumber, 1}, iNode];
                        %if ix ~= 0 && iy ~= 0
                            %cost = sqrt((xNode-newX)^2+(yNode-newY)^2+(zNode-newZ)^2);
                        %else
                        
                        %end
                        Successors{iNode, 2} = [Successors{iNode, 2}, cost];
                        Predecessors{newNodeNumber, 2} = [Predecessors{newNodeNumber, 2}, cost];
                    end
                else
                    Successors{iNode, 1} = [Successors{iNode}, newNodeNumber];
                    Predecessors{newNodeNumber, 1} = [Predecessors{newNodeNumber, 1}, iNode];
                        %if ix ~= 0 && iy ~= 0
                            %cost = sqrt((xNode-newX)^2+(yNode-newY)^2+(zNode-newZ)^2);
                        %else
                        %end
                    Successors{iNode, 2} = [Successors{iNode, 2}, cost];
                    Predecessors{newNodeNumber, 2} = [Predecessors{newNodeNumber, 2}, cost];
                end
            end

        end

    end

    % G, RHS
    G = inf(1, nNodes);
    RHS = inf(1, nNodes);

    %% dynamic obsts
    Model.NewObsts.count = 0;

    %% save model
    Model.sLast = Model.Robot.startNode;
    Model.Predecessors = Predecessors;
    Model.Successors = Successors;
    Model.RHS = RHS;
    Model.G = G;
    Model.km = 0;

    %% plot model
    % plotModel(model);

end
