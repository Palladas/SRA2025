function val = PerlinNoise2D(x, y)
    persistent p;
    if isempty(p)
        % Initialize a random permutation table
        p = randperm(256); 
        p = [p, p]; % Duplicate for easier wrap-around
    end

    % Unit cube coordinates
    X = floor(x);
    Y = floor(y);

    % Relative coordinates of the point in the cube
    x = x - X;
    y = y - Y;

    % Wrap around to fit the permutation table
    X = mod(X, 256);
    Y = mod(Y, 256);

    % Smooth cubic interpolation curves
    u = fade(x);
    v = fade(y);

    % Hash coordinates of the 8 cube corners
    A = p(X + 1) + Y;
    AA = p(A + 1);
    AB = p(A + 2);
    B = p(X + 2) + Y;
    BA = p(B + 1);
    BB = p(B + 2);

    % Interpolate results of grad function
    val = lerp(v, lerp(u, grad(p(AA + 1), x, y), ...
                          grad(p(BA + 1), x - 1, y)), ...
                  lerp(u, grad(p(AB + 1), x, y - 1), ...
                          grad(p(BB + 1), x - 1, y - 1)));
end

% Helper function for fading (smoother interpolation)
function t = fade(t)
    t = t * t * t * (t * (t * 6 - 15) + 10);
end

% Helper function for gradient dot product
function val = grad(hash, x, y)
    h = bitand(hash, 15);
    % Choose a gradient vector based on the hash value
    % 0-7 for x,y, 8-15 for other combinations
    switch h
        case {0, 4, 8, 12}
            u = x; v = y; % (x, y)
        case {1, 5, 9, 13}
            u = -x; v = y; % (-x, y)
        case {2, 6, 10, 14}
            u = x; v = -y; % (x, -y)
        case {3, 7, 11, 15}
            u = -x; v = -y; % (-x, -y)
        otherwise
            u = 0; v = 0; % Should not happen
    end
    val = u + v;
end

% Helper function for linear interpolation
function val = lerp(t, a, b)
    val = a + t * (b - a);
end