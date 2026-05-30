function map = update_occupancy_grid(map, rx, ry, rtheta, ranges, res, off_x, off_y, max_trust_range)
    % --- Parametrar för Log-Odds ---
    l_occ = 0.8;   % Öka log-odds om lasern träffar ett hinder (Hinder)
    l_free = -0.4; % Minska log-odds för celler lasern passerar igenom (Fri yta)
    
    % Robotens cell-koordinater i matrisen
    robot_cell_x = round(rx * res) + off_x;
    robot_cell_y = round(ry * res) + off_y;
    
    num_angles = length(ranges);
    
    % --- DYNAMISKT OCH KORRIGERAT VINKELSVEP ---
    if num_angles <= 181
        % Intel Lab (180 grader framåt)
        angles = linspace(-pi/2, pi/2, num_angles);
    else
        %Both are 180
        angles = linspace(-pi/2, pi/2, num_angles); 
    end
    
    % Loopa igenom varje enskild laserstråle
    for i = 1:num_angles
        r = ranges(i);
        
        % KRITERIUM: Filtrera bort brus eller mätningar utanför vår betrodda räckvidd
        if r > max_trust_range || r > 80 
            continue;
        end
        
        % Strålens absolut vinkel i världen
        global_angle = rtheta + angles(i);
        
        % Hindrets position i verkligheten (meter)
        hinder_x = rx + r * cos(global_angle);
        hinder_y = ry + r * sin(global_angle);
        
        % Hindrets cell-koordinater i matrisen
        hinder_cell_x = round(hinder_x * res) + off_x;
        hinder_cell_y = round(hinder_y * res) + off_y;
        
        % --- Bresenham's Ray Tracing ---
        [X, Y] = bresenham(robot_cell_x, robot_cell_y, hinder_cell_x, hinder_cell_y);
        
        % Celler längs strålen sätts till FREE
        for j = 1:(length(X)-1)
            if X(j) > 0 && X(j) <= size(map, 2) && Y(j) > 0 && Y(j) <= size(map, 1)
                map(Y(j), X(j)) = map(Y(j), X(j)) + l_free;
            end
        end
        
        % Sista cellen där lasern studsade sätts till OCCUPIED
        if hinder_cell_x > 0 && hinder_cell_x <= size(map, 2) && hinder_cell_y > 0 && hinder_cell_y <= size(map, 1)
            map(hinder_cell_y, hinder_cell_x) = map(hinder_cell_y, hinder_cell_x) + l_occ;
        end
    end
end

function [x, y] = bresenham(x1, y1, x2, y2)
    dx = abs(x2 - x1); dy = abs(y2 - y1);
    sx = sign(x2 - x1); sy = sign(y2 - y1);
    x = x1; y = y1;
    if dx > dy
        err = dx / 2;
        while x(end) ~= x2
            x = [x; x(end) + sx];
            err = err - dy;
            if err < 0
                y = [y; y(end) + sy];
                err = err + dx;
            else
                y = [y; y(end)];
            end
        end
    else
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