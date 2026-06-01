clear; clc; close all;
datasets = {'intel.gfs.log', 'fr-campus-20040714.carmen.gfs.log'};
for d = 1:length(datasets)
    filename = datasets{d};
    fprintf('\nStartar processering av: %s\n', filename);
    if contains(filename, 'intel')
        % Intel Lab
        resolution = 10;        % celler per meter, 10 = 10 cm/cell
        map_width_m = 100;      % kartbredd i meter
        map_height_m = 100;     % karthöjd i meter
        map_width = map_width_m * resolution;
        map_height = map_height_m * resolution;
        offset_x = map_width / 2;
        offset_y = map_height / 2;
        fig_name = 'Occupancy Grid Map - Intel Lab';
        max_trust_range = 20;
        mat_file = 'IntelMaze.mat';
        var_name = 'IntelMaze';
    else
        % Freiburg Campus
        resolution = 1;         % celler per meter
        map_width_m = 400;      % kartbredd i meter
        map_height_m = 400;     % karthöjd i meter
        map_width = map_width_m * resolution;
        map_height = map_height_m * resolution;
        offset_x_m = map_width_m / 2 - 100;
        offset_y_m = map_height_m / 2 + 100;
        offset_x = offset_x_m * resolution;
        offset_y = offset_y_m * resolution;
        fig_name = 'Occupancy Grid Map - Freiburg Campus';
        max_trust_range = 55;
        mat_file = 'FreiburgMaze.mat';
        var_name = 'FreiburgMaze';
    end
    map_log_odds = zeros(map_height, map_width);
    fid = fopen(filename, 'r');
    if fid == -1
        warning('Kunde inte öppna filen: %s', filename);
        continue;
    end
    figure('Name', fig_name);
    colormap(gray);
    count = 0;
    first_robot_cell = [];
    last_robot_cell = [];
    while ~feof(fid)
        line = fgetl(fid);
        if startsWith(line, 'FLASER')
            data = strsplit(line);
            num_readings = str2double(data{2});
            laser_ranges = str2double(data(3 : 2 + num_readings));
            robot_x = str2double(data{3 + num_readings});
            robot_y = str2double(data{4 + num_readings});
            robot_theta = str2double(data{5 + num_readings});
            robot_cell_x = round(robot_x * resolution) + offset_x;
            robot_cell_y = round(robot_y * resolution) + offset_y;
            
            if isempty(first_robot_cell)
                first_robot_cell = [robot_cell_y; robot_cell_x]; % [row; col]
            end
            last_robot_cell = [robot_cell_y; robot_cell_x];
            map_log_odds = update_occupancy_grid( ...
                map_log_odds, robot_x, robot_y, robot_theta, laser_ranges, ...
                resolution, offset_x, offset_y, max_trust_range);
            count = count + 1;
            if mod(count, 100) == 0
                imagesc(flipud(1 - 1./(1 + exp(map_log_odds))));
                axis equal;
                set(gca, 'YDir', 'reverse');
                title(sprintf('Genererar %s... Tidssteg: %d', filename, count));
                drawnow;
            end
        end
    end
    fclose(fid);
    imagesc(flipud(1 - 1./(1 + exp(map_log_odds))));
    axis equal;
    set(gca, 'YDir', 'reverse');
    title(['Färdig karta: ', filename]);
    
    % Spara och exportera den färdiga, rensade och flippade planeringskartan
    make_planning_maze( ...
        map_log_odds, ...
        first_robot_cell, ...
        last_robot_cell, ...
        mat_file, ...
        var_name, ...
        resolution, ...
        offset_x, ...
        offset_y);
    fprintf('Sparade %s\n', mat_file);
end
fprintf('\nBåda kartorna har genererats och exporterats!\n');

function make_planning_maze(map_log_odds, start_cell, goal_cell, out_file, var_name, resolution, offset_x, offset_y)
    % Skapar en path-planning-kompatibel Maze med samma storlek som grid map:
    %   Maze.map        : 0 = fri yta, inf = hinder eller okänt
    %   Maze.start      : [col; row] (Anpassad till din planner.m [X; Y])
    %   Maze.goal       : [col; row] (Anpassad till din planner.m [X; Y])
    
    free_threshold = 0.35;
    inflate_cells = 1;
    crop_padding = 20; % Säkerhetsmarginal med extra celler utanför de fria ytorna
    
    % 1. Skapa den råa matrisen utifrån sannolikheter (allt okänt/vägg = inf till att börja med)
    prob_occ = 1 ./ (1 + exp(-map_log_odds));
    free = prob_occ < free_threshold;
    
    planning_map = inf(size(map_log_odds));
    planning_map(free) = 0; % <--- Här sätts allt till 0 eller inf
    
    % 2. BERÄKNA BOUNDS UTIFRÅN DEN FÄRDIGA PLANNING_MAP
    % Vi letar efter alla celler som har tilldelats fri yta (värde 0)
    [free_rows, free_cols] = find(planning_map == 0);
    
    if isempty(free_rows)
        min_y = 1; max_y = size(planning_map, 1);
        min_x = 1; max_x = size(planning_map, 2);
    else
        % Vi expanderar rektangeln med crop_padding utanför de fria cellerna
        % Detta säkerställer att väggarna precis utanför kommer med!
        min_y = max(min(free_rows) - crop_padding, 1);
        max_y = min(max(free_rows) + crop_padding, size(planning_map, 1));
        min_x = max(min(free_cols) - crop_padding, 1);
        max_x = min(max(free_cols) + crop_padding, size(planning_map, 2));
    end
    
    % 3. Skär ut rektangeln ur vår tilldelade planning_map
    cropped_map = planning_map(min_y:max_y, min_x:max_x);
    
    % Justera start- och målkoordinaterna baserat på den nya rektangelns startpunkt
    start_cropped = [start_cell(1) - min_y + 1; start_cell(2) - min_x + 1];
    goal_cropped  = [goal_cell(1) - min_y + 1;  goal_cell(2) - min_x + 1];
    
    % 4. FLIPPA MATRISEN OCH KOORDINATERNA HÄR (Så att allt är rättvänt för run_lab3)
    cropped_map = flipud(cropped_map);
    start_cropped(1) = size(cropped_map, 1) - start_cropped(1) + 1;
    goal_cropped(1)  = size(cropped_map, 1) - goal_cropped(1) + 1;
    
    %Make obstacles bigger, to have a margin
    cropped_map = inflate_obstacles(cropped_map, inflate_cells);
    
    % Garantera och tvinga att start och mål alltid är öppna (0)
    cropped_map(start_cropped(1), start_cropped(2)) = 0;
    cropped_map(goal_cropped(1), goal_cropped(2)) = 0;
    
    % 6. Spara i .mat med formatet [X; Y] -> [kolumn; rad] för din planner.m
    Maze.map = cropped_map;
    Maze.start = [start_cropped(2); start_cropped(1)]; 
    Maze.goal  = [goal_cropped(2);  goal_cropped(1)];
    
    Maze.resolution = resolution;
    Maze.offset = [offset_y; offset_x];
    
    eval([var_name ' = Maze;']);
    save(out_file, var_name);
end

%Makes all obstacles bigger, by radius pixels in all directions
function inflated = inflate_obstacles(map, radius)
    inflated = map;
    if radius <= 0
        return;
    end
    [obs_r, obs_c] = find(isinf(map));
    for k = 1:length(obs_r)
        %+/-radius in all directions
        for dr = -radius:radius
            for dc = -radius:radius
                rr = obs_r(k) + dr;
                cc = obs_c(k) + dc;
                %Boundary check
                if rr >= 1 && rr <= size(map, 1) && cc >= 1 && cc <= size(map, 2)
                    inflated(rr, cc) = inf;
                end
            end
        end
    end
end

%Grid, robot, lasers, resolution, offset, max range
function map = update_occupancy_grid(map, rx, ry, rtheta, ranges, res, off_x, off_y, max_trust_range)
    l_occ = 0.8;   
    l_free = -0.4; 
    robot_cell_x = round(rx * res) + off_x;
    robot_cell_y = round(ry * res) + off_y;

    %Loop from -pi/2 to pi/2
    num_angles = length(ranges);
    angles = linspace(-pi/2, pi/2, num_angles); 
    for i = 1:num_angles

        %If no obstacle seen in the max range, go to the next angle
        r = ranges(i);
        if r > max_trust_range || r > 80 
            continue;
        end

        %Get the cell of the seen obstacle
        global_angle = rtheta + angles(i);
        hinder_x = rx + r * cos(global_angle);
        hinder_y = ry + r * sin(global_angle);
        hinder_cell_x = round(hinder_x * res) + off_x;
        hinder_cell_y = round(hinder_y * res) + off_y;
        
        %All cells on the way to the obstacle should be empty
        [X, Y] = bresenham(robot_cell_x, robot_cell_y, hinder_cell_x, hinder_cell_y);
        for j = 1:(length(X)-1)
            %Checking map bounds
            if X(j) > 0 && X(j) <= size(map, 2) && Y(j) > 0 && Y(j) <= size(map, 1)
                map(Y(j), X(j)) = map(Y(j), X(j)) + l_free; %Those celss more likely to be free
            end
        end
        if hinder_cell_x > 0 && hinder_cell_x <= size(map, 2) && hinder_cell_y > 0 && hinder_cell_y <= size(map, 1)
            map(hinder_cell_y, hinder_cell_x) = map(hinder_cell_y, hinder_cell_x) + l_occ; %hinder cell more likely to be occupied
        end
    end
end

%Returns two vectors with all indicies hit by a line going between two points
function [x, y] = bresenham(x1, y1, x2, y2)
    dx = abs(x2 - x1); 
    dy = abs(y2 - y1);

    sx = sign(x2 - x1); 
    sy = sign(y2 - y1);

    x = x1; 
    y = y1;

    if dx > dy
        %Err is used for rounding
        % = 0.5 dx since we round upwards
        err = dx / 2;

        %Increment x by +/- 1 until we reach the second point
        while x(end) ~= x2
            x = [x; x(end) + sx];

            %Repeat same value of y until we round up to a new value
            err = err - dy;
            if err < 0
                y = [y; y(end) + sy];
                err = err + dx; % 1.5 dx, 2.5dx...
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