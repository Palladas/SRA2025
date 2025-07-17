function [altitudeGrid] = altitudeGen(gridSize)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    gridSize
end

arguments (Output)
    altitudeGrid
end

fprintf('Generating altitude grid using Perlin noise...\n');
% Parameters for Perlin Noise
scale = 0.05; % Controls "zoom" level of noise
octaves = 4; % Number of noise layers
persistence = 0.5; % How much each octave contributes
lacunarity = 2.0; % How much larger each octave is

altitudeGrid = zeros(gridSize, gridSize);
for i = 1:gridSize
    for j = 1:gridSize
        x = (j - 1) * scale; % Map grid coords to noise coords
        y = (i - 1) * scale;
        
        noise_value = 0;
        amplitude = 1;
        frequency = 1;
        
        for o = 1:octaves
            % MATLAB's perlin noise is typically 3D, we'll use 2D by fixing z
            noise_value = noise_value + PerlinNoise2D(x * frequency, y * frequency) * amplitude;
            amplitude = amplitude * persistence;
            frequency = frequency * lacunarity;
        end
        altitudeGrid(i, j) = noise_value;
    end
end

% Normalize and scale altitude values (e.g., to a range of 0 to 100)
minAlt = min(altitudeGrid(:));
maxAlt = max(altitudeGrid(:));
altitudeGrid = (altitudeGrid - minAlt) / (maxAlt - minAlt) * 100; % Scale to 0-100 for better visualization and cost impact

fprintf('Altitude grid generated.\n');
end