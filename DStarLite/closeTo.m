function out = closeTo(x1,y1,x2,y2,dist)
%CLOSETO Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    x1
    y1
    x2
    y2
    dist
end

arguments (Output)
    out
end

out = sqrt((x1-x2)^2+(y1-y2)^2)<=dist;
end