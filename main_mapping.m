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

        max_grid_size = 1000;   % Intel: max storlek för Maze
    else
        map_width = 4000;
        map_height = 4000;
        offset_x = map_width / 2 - 1000;
        offset_y = map_height / 2 + 1000;
        fig_name = 'Occupancy Grid Map - Freiburg Campus';
        max_trust_range = 55;

        mat_file = 'FreiburgMaze.mat';
        var_name = 'FreiburgMaze';

        max_grid_size = 1000;   % Freiburg: skalas ner till max 1000
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
    % Skapar en path-planning-kompatibel Maze med samma storlek som grid map:
    %   Maze.map   : 0 = fri yta, inf = hinder eller okänt
    %   Maze.start : [row; col]
    %   Maze.goal  : [row; col]

    free_threshold = 0.35;
    inflate_cells = 1;

    % Log-odds till sannolikhet för upptaget.
    % map_log_odds < 0 => fri yta
    % map_log_odds = 0 => okänt
    % map_log_odds > 0 => hinder
    prob_occ = 1 ./ (1 + exp(-map_log_odds));

    % Endast tydligt fri yta blir körbar.
    % Allt annat, alltså hinder och okänt, blir blockerad yta.
    free = prob_occ < free_threshold;

    planning_map = inf(size(map_log_odds));
    planning_map(free) = 0;

    planning_map = inflate_obstacles(planning_map, inflate_cells);

    start = start_cell(:);
    goal = goal_cell(:);

    % Säkerställ att start och mål ligger inom kartan
    start(1) = min(max(start(1), 1), size(planning_map, 1));
    start(2) = min(max(start(2), 1), size(planning_map, 2));

    goal(1) = min(max(goal(1), 1), size(planning_map, 1));
    goal(2) = min(max(goal(2), 1), size(planning_map, 2));

    % Start och mål måste vara körbara
    planning_map(start(1), start(2)) = 0;
    planning_map(goal(1), goal(2)) = 0;

    Maze.map = planning_map;
    Maze.start = start;
    Maze.goal = goal;

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