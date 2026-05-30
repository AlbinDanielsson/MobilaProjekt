clear; clc; close all;
datasets = {'intel.gfs.log', 'fr-campus-20040714.carmen.gfs.log'};
for d = 1:length(datasets)
    filename = datasets{d};
    fprintf('\nStartar processering av: %s\n', filename);
    resolution = 10;
    if contains(filename, 'intel')
        map_width = 1000;
        map_height = 1000;
        offset_x = map_width / 2;
        offset_y = map_height / 2;
        fig_name = 'Occupancy Grid Map - Intel Lab';
        max_trust_range = 20.5;
        mat_file = 'IntelMaze.mat';
        var_name = 'IntelMaze';
    else
        map_width = 4000;
        map_height = 4000;
        offset_x = map_width / 2 - 1000;
        offset_y = map_height / 2 + 1000;
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
                imagesc(flipud(1 - 1./(1+exp(map_log_odds))));
                axis equal;
                set(gca, 'YDir', 'reverse');
                title(sprintf('Genererar %s... Tidssteg: %d', filename, count));
                drawnow;
            end
        end
    end
    fclose(fid);
    imagesc(flipud(1 - 1./(1+exp(map_log_odds))));
    axis equal;
    set(gca, 'YDir', 'reverse');
    title(['Färdig karta: ', filename]);
    
    if contains(filename, 'intel')
        intel_map = map_log_odds;
    else
        freiburg_map = map_log_odds;
    end
    
    % FIX: Nu skickar vi med resolution, offset_x och offset_y till funktionen!
    make_planning_maze(map_log_odds, first_robot_cell, last_robot_cell, mat_file, var_name, resolution, offset_x, offset_y);
    
    fprintf('Sparade %s\n', mat_file);
end
fprintf('\nBåda kartorna har genererats och exporterats!\n');

% =========================================================================
% LOKALA FUNKTIONER
% =========================================================================

function make_planning_maze(map_log_odds, start_cell, goal_cell, out_file, var_name, resolution, offset_x, offset_y)
    free_threshold = 0.65;
    inflate_cells = 1;
    prob_occ = 1 ./ (1 + exp(-map_log_odds));
    free = prob_occ < free_threshold;
    planning_map = inf(size(map_log_odds));
    planning_map(free) = 0;
    planning_map = inflate_obstacles(planning_map, inflate_cells);
    start = start_cell(:);
    goal = goal_cell(:);
    start(1) = min(max(start(1), 1), size(planning_map, 1));
    start(2) = min(max(start(2), 1), size(planning_map, 2));
    goal(1) = min(max(goal(1), 1), size(planning_map, 1));
    goal(2) = min(max(goal(2), 1), size(planning_map, 2));
    planning_map(start(1), start(2)) = 0;
    planning_map(goal(1), goal(2)) = 0;
    Maze.map = planning_map;
    
    % KORRIGERING: Spara som [X; Y] dvs [kolumn; rad] så din planner läser rätt axel
    Maze.start = [start(2); start(1)]; 
    Maze.goal = [goal(2); goal(1)];
    
    Maze.resolution = resolution;
    Maze.offset = [offset_y; offset_x];
    eval([var_name ' = Maze;']);
    save(out_file, var_name);
end

function inflated = inflate_obstacles(map, radius)
    inflated = map;
    if radius <= 0
        return;
    end
    [obs_r, obs_c] = find(isinf(map));
    for k = 1:length(obs_r)
        for dr = -radius:radius
            for dc = -radius:radius
                rr = obs_r(k) + dr;
                cc = obs_c(k) + dc;
                if rr >= 1 && rr <= size(map, 1) && cc >= 1 && cc <= size(map, 2)
                    inflated(rr, cc) = inf;
                end
            end
        end
    end
end

function map = update_occupancy_grid(map, rx, ry, rtheta, ranges, res, off_x, off_y, max_trust_range)
    l_occ = 0.8;   
    l_free = -0.4; 
    robot_cell_x = round(rx * res) + off_x;
    robot_cell_y = round(ry * res) + off_y;
    num_angles = length(ranges);
    
    if num_angles <= 181
        angles = linspace(-pi/2, pi/2, num_angles);
    else
        angles = linspace(-pi/2, pi/2, num_angles); 
    end
    
    for i = 1:num_angles
        r = ranges(i);
        if r > max_trust_range || r > 80 
            continue;
        end
        global_angle = rtheta + angles(i);
        hinder_x = rx + r * cos(global_angle);
        hinder_y = ry + r * sin(global_angle);
        hinder_cell_x = round(hinder_x * res) + off_x;
        hinder_cell_y = round(hinder_y * res) + off_y;
        
        [X, Y] = bresenham(robot_cell_x, robot_cell_y, hinder_cell_x, hinder_cell_y);
        for j = 1:(length(X)-1)
            if X(j) > 0 && X(j) <= size(map, 2) && Y(j) > 0 && Y(j) <= size(map, 1)
                map(Y(j), X(j)) = map(Y(j), X(j)) + l_free;
            end
        end
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