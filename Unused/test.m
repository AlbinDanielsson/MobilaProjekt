clear; clc; close all;

x1 = 0;
y1 = 0;

x2 = 10;
y2 = 2;

[X, Y] = bresenham(x1, y1, x2, y2);


%Returns matrix with all indicies hit by a line going between two points
function [x, y] = bresenham(x1, y1, x2, y2)
    dx = abs(x2 - x1); 
    dy = abs(y2 - y1);

    sx = sign(x2 - x1); 
    sy = sign(y2 - y1);

    x = x1; 
    y = y1;

    if dx > dy
        %Err is used for rounding
        err = dx / 2;

        %Increment x by +/- 1 until we reach the second point
        while x(end) ~= x2
            x = [x; x(end) + sx];

            %Repeat same value of y until we round up to a new value
            err = err - dy;
            if err < 0
                y = [y; y(end) + sy];
                err = err + dx;
            else
                y = [y; y(end)];
            end
        end
    else %same as other case but x,y flipped
        err = dy / 2;
        while y(end) ~= y2
            y = [y; y(end) + sy];
            err = err - dx;
            if err < 0
                x = [x; x(end) + sx];
                err = err + dy;
            else
                x = [x; x(end)];
            end
        end
    end
end