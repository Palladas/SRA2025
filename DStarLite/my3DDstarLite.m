function [Model, Path] = my3DDstarLite(Model)
    % DstarLite algorithm

    % initialization
    [G, RHS, Open] = initializeDstarLite(Model);

    t = 1;
    Model.Robot.obstDist = 2; 
    Path.nodeNumbers = [Model.Robot.startNode];

    % update start_key
    Start.nodeNumber = Model.Robot.startNode;
    Start.key = min(G(Start.nodeNumber), RHS(Start.nodeNumber)) * [1; 1];
    Start.cord = nodes2coords(Start.nodeNumber, Model);
    currentDir = deg2rad(Model.Robot.dir);

    % compute shortest path
    [G, RHS, Open, Start] = computeShortestPath(G, RHS, Open, Start, Model);
    currPath = genCurrPath(Start,Model,G,currentDir);
    x = currPath.coords(:,1);
    y = currPath.coords(:,2);
    z = zeros(length(x));
    for i = 1:length(x)
        z(i) = Model.Alti(y(i),x(i));
    end


    %% main procedure

    % if G(Start.nodeNumber)=inf -> then there is no known path

    Color = 'g';
    newObstColor = [0.8500 0.3250 0.0980];

    if isfield(Model, 'NewObsts')

            for iNewObst = 1:Model.NewObsts.count
                    plot(Model.NewObsts.x(iNewObst), Model.NewObsts.y(iNewObst), 'o', 'MarkerSize', 5, ...
                        'MarkerFaceColor', [0.7 0.1 0.1], 'MarkerEdgeColor', [0.9 1 1]);
            end

    end

    robotPlot = plot3(Start.cord(1), Start.cord(2), Model.Alti(Start.cord(2), Start.cord(1)),'o', 'MarkerFaceColor', Color, 'MarkerEdgeColor', Color(1, :), 'MarkerSize', 8);


    pathPlot = plot3(x,y,z, 'b', 'LineWidth', 2);

    frame = getframe(gcf);
    img = frame2im(frame);
    [img, cmap] = rgb2ind(img, 256);
    imwrite(img, cmap, 'animation.gif', 'gif', 'LoopCount', Inf, 'DelayTime', 1);

    while Start.nodeNumber ~= Model.Robot.targetNode

        % move robot to next node
        sucNodes = Model.Successors{Start.nodeNumber, 1};

        switch Model.expandMethod
            case 'heading'
                dTheta = turnCost(Start.nodeNumber, sucNodes, Model, currentDir);
                [~, sortInds] = sortrows([G(sucNodes) + Model.Successors{Start.nodeNumber, 2}; abs(dTheta)]');
                Start.nodeNumber = sucNodes(sortInds(1));
                currentDir = currentDir + dTheta(sortInds(1));
            case 'random'
                [~, minInd] = min(G(sucNodes) + Model.Successors{Start.nodeNumber, 2});
                Start.nodeNumber = sucNodes(minInd);
        end

        Start.coords = nodes2coords(Start.nodeNumber, Model);

        % move to Start.nodeNumber and add Start.nodeNumber to Path
        Path.nodeNumbers(end + 1) = Start.nodeNumber;
        t = t + 1;

        % check for update in edge costs (obstacles)
        [Open, RHS, Model] = checkForUpdate(Open, RHS, Model, G, t, Start);

        % compute shortest path
       [G, RHS, Open, Start] = computeShortestPath(G, RHS, Open, Start, Model);
       currPath = genCurrPath(Start,Model,G,currentDir);

       set(robotPlot, 'XData', Start.coords(1))
       set(robotPlot, 'YData', Start.coords(2))
       set(robotPlot, 'ZData', Model.Alti(Start.coords(2), Start.coords(1)))
       
       x = currPath.coords(:,1);
       y = currPath.coords(:,2);
       z = zeros(length(x));
       for i = 1:length(x)
           z(i) = Model.Alti(y(i),x(i));
       end
       
       %plot(x, y, 'b', 'LineWidth', 2);
       pathPlot = plot3(x,y,z, 'b', 'LineWidth', 2);
       %plot(x(2:end - 1), y(2:end - 1), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 4);

       if isfield(Model, 'NewObsts')

            for iNewObst = 1:Model.NewObsts.count
                if closeTo(Start.coords(1),Start.coords(2),Model.NewObsts.x(iNewObst),Model.NewObsts.y(iNewObst),Model.Robot.obstDist)
                    plot(Model.NewObsts.x(iNewObst), Model.NewObsts.y(iNewObst), 'o', 'MarkerSize', 5, ...
                        'MarkerFaceColor', newObstColor, 'MarkerEdgeColor', newObstColor);
                end

            end

        end


      frame = getframe(gcf);
      img = frame2im(frame);
      [img, cmap] = rgb2ind(img, 256);
      imwrite(img, cmap, 'animation.gif', 'gif', 'WriteMode', 'append', 'DelayTime', 0.2);

      pause(0.2)

    end

    %% optimal paths coordinations, nodes, directions
    Path.coords = nodes2coords(Path.nodeNumbers, Model);
    Path.dirs = nodes2dirs(Path.nodeNumbers, Model);



end
