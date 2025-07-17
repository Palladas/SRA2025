function visAlti(altitudeGrid,terrainGrid)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    altitudeGrid
    terrainGrid
end

arguments (Output)
end
gridSize = length(altitudeGrid);

figure('Name', 'Generated Altitude Map', 'NumberTitle', 'off', 'Color', [0.95 0.95 0.95]);
terrainColor = terrainColors(terrainGrid);

%surf(altitudeGrid,terrainColor)
surf(altitudeGrid)
hold on;
image(altitudeGrid);
colormap(gray); % Or 'jet', 'gray', etc. for elevation
%colorbar;
xlim([0.5, gridSize + 0.5]); % Adjust limits for imagesc
ylim([0.5, gridSize + 0.5]); % Adjust limits for imagesc
zlim([0,max(altitudeGrid,[],'all')]);
set(gca, 'YDir', 'normal');
title('Generated Altitude Map (Z-values)');
xlabel('X-coordinate');
ylabel('Y-coordinate');
fprintf('Altitude map visualized.\n');

end