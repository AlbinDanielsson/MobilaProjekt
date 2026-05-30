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

    % Spara original-log-odds också om ni vill kunna finjustera senare
    if contains(filename, 'intel')
        intel_map = map_log_odds;
    else
        freiburg_map = map_log_odds;
    end

    % Skapa path-planning-kompatibel .mat-fil
    make_planning_maze( ...
        map_log_odds, ...
        first_robot_cell, ...
        last_robot_cell, ...
        mat_file, ...
        var_name);

    fprintf('Sparade %s\n', mat_file);
end

fprintf('\nBåda kartorna har genererats och exporterats!\n');


function make_planning_maze(map_log_odds, start_cell, goal_cell, out_file, var_name)
    % Parametrar att justera
    max_grid_size = 1000;      % inte 80x80; max ungefär 300x300
    occ_threshold = 0.65;     % celler över detta blir hinder
    crop_padding = 30;        % marginal runt utforskat område
    inflate_cells = 1;        % gör hinder lite tjockare

    prob = 1 - 1 ./ (1 + exp(map_log_odds));

    known = abs(map_log_odds) > 0.01;
    occupied = prob > occ_threshold;

    % Croppa bort tomma delar av den stora kartan
    relevant = known | occupied;

    if ~isempty(start_cell)
        relevant(start_cell(1), start_cell(2)) = true;
    end

    if ~isempty(goal_cell)
        relevant(goal_cell(1), goal_cell(2)) = true;
    end

    [rows, cols] = find(relevant);

    r1 = max(min(rows) - crop_padding, 1);
    r2 = min(max(rows) + crop_padding, size(map_log_odds, 1));
    c1 = max(min(cols) - crop_padding, 1);
    c2 = min(max(cols) + crop_padding, size(map_log_odds, 2));

    occupied_crop = occupied(r1:r2, c1:c2);

    start_crop = start_cell - [r1; c1] + 1;
    goal_crop = goal_cell - [r1; c1] + 1;

    % Skala ner till rimlig storlek för A*/Dijkstra
    [h, w] = size(occupied_crop);
    scale = max(1, ceil(max(h, w) / max_grid_size));

    new_h = ceil(h / scale);
    new_w = ceil(w / scale);

    planning_map = zeros(new_h, new_w);

    for rr = 1:new_h
        for cc = 1:new_w
            r_start = (rr - 1) * scale + 1;
            r_end = min(rr * scale, h);

            c_start = (cc - 1) * scale + 1;
            c_end = min(cc * scale, w);

            block = occupied_crop(r_start:r_end, c_start:c_end);

            if any(block(:))
                planning_map(rr, cc) = inf;
            end
        end
    end

    start = ceil(start_crop / scale);
    goal = ceil(goal_crop / scale);

    start(1) = min(max(start(1), 1), new_h);
    start(2) = min(max(start(2), 1), new_w);
    goal(1) = min(max(goal(1), 1), new_h);
    goal(2) = min(max(goal(2), 1), new_w);

    planning_map = inflate_obstacles(planning_map, inflate_cells);

    % Säkerställ att start och mål inte råkar hamna i hinder efter skalning
    planning_map(start(1), start(2)) = 0;
    planning_map(goal(1), goal(2)) = 0;

    Maze.map = planning_map;
    
    % Din planner kräver [X; Y] vilket motsvarar [kolumn; rad]
    Maze.start = [start(2); start(1)]; 
    Maze.goal = [goal(2); goal(1)];
    
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