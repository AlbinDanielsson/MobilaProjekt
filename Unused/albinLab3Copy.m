clear;
clc;
close all;

algos = {'Astar'};

fprintf('%s | %s | %s | %s | %s | %s\n', ...
    'MAP', 'CASE', 'ALGO', 'PUSHES', 'POPS', 'TIME(s)');

for map_number = 1:2

    if map_number == 1
        load('IntelMaze.mat');

        Maze = IntelMaze;
        Maze.goal = [274; 410];     % [row; col]
        Maze.start = [550; 581];    % [row; col]

        name = 'Intel Lab';

    else
        load('FreiburgMaze.mat');

        Maze = FreiburgMaze;
        Maze.goal = [197; 115];     % [row; col]
        Maze.start = [354; 242];    % [row; col]

        name = 'Freiburg Campus';
    end

    map = Maze.map;
    s_pos = Maze.start(:);
    g_pos = Maze.goal(:);

    for case_type = 1:1
        if case_type == 1
            case_lbl = 'Known';
        else
            case_lbl = 'Unknown';
        end

        for a = 1:length(algos)
            algo = algos{a};

            if strcmp(algo, 'DStarLite')
                [path, psh, pp, t] = dstar_lite_planner(map, s_pos, g_pos, case_type == 1);
            else
                if case_type == 1
                    [path, psh, pp, t] = planner(map, s_pos, g_pos, algo);
                else
                    [path, psh, pp, t] = run_unknown_standard(map, s_pos, g_pos, algo);
                end
            end

            fprintf('%s | %s | %s | %d | %d | %f\n', ...
                name, case_lbl, algo, psh, pp, t);

            plot_planning_result(map, s_pos, g_pos, path, name, case_lbl, algo);
        end
    end

    fprintf('------------------------------------------------------------------------\n');
end


function [traj, t_push, t_pop, total_t] = run_unknown_standard(act_map, s_pos, g_pos, type)
    tic;

    t_push = 0;
    t_pop = 0;

    s_pos = s_pos(:);
    g_pos = g_pos(:);

    known_map = zeros(size(act_map));
    curr = s_pos;

    traj = curr.'; % Nx2, där varje rad är [row col]

    [c_path, psh, pp, ~] = planner(known_map, curr, g_pos, type);
    c_path = normalize_path(c_path);

    t_push = t_push + psh;
    t_pop = t_pop + pp;

    while ~isempty(c_path) && ~isequal(curr, g_pos)
        replanned = false;

        % Kolla 8 grannar runt roboten
        for d_row = -1:1
            for d_col = -1:1
                look_row = curr(1) + d_row;
                look_col = curr(2) + d_col;

                if look_row < 1 || look_col < 1 || ...
                   look_row > size(act_map, 1) || ...
                   look_col > size(act_map, 2)
                    continue;
                end

                if isinf(act_map(look_row, look_col)) && ~isinf(known_map(look_row, look_col))
                    known_map(look_row, look_col) = inf;
                    replanned = true;
                end
            end
        end

        if replanned
            [c_path, psh, pp, ~] = planner(known_map, curr, g_pos, type);
            c_path = normalize_path(c_path);

            t_push = t_push + psh;
            t_pop = t_pop + pp;

            if isempty(c_path)
                break;
            end
        end

        % Ta ett steg framåt längs pathen
        if size(c_path, 1) > 1
            curr = c_path(2, :).';
            traj = [traj; curr.'];
            c_path(1, :) = [];
        else
            break;
        end
    end

    total_t = toc;
end


function plot_planning_result(map, s_pos, g_pos, path, name, case_lbl, algo)
    path = normalize_path(path);

    figure('Name', sprintf('%s - %s - %s', name, case_lbl, algo));

    % 1 = hinder/okänt, 0 = fri yta
    imagesc(isinf(map));
    colormap(gray);
    hold on;

    % path är [row col], men plot vill ha x=col, y=row
    if ~isempty(path)
        plot(path(:, 2), path(:, 1), 'g-', 'LineWidth', 2.5);
    end

    plot(s_pos(2), s_pos(1), 'sb', ...
        'MarkerFaceColor', 'b', ...
        'MarkerSize', 10, ...
        'Color', 'b');

    plot(g_pos(2), g_pos(1), 'sy', ...
        'MarkerFaceColor', 'y', ...
        'MarkerSize', 10, ...
        'Color', 'y');

    title(sprintf('%s [%s Case] - %s', name, case_lbl, algo));

    axis equal;
    axis([1 size(map, 2) 1 size(map, 1)]);
    set(gca, 'YDir', 'normal');

    hold off;
end


function path = normalize_path(path)
    if isempty(path)
        return;
    end

    % Om path kommer som 2xN, gör om till Nx2.
    if size(path, 1) == 2 && size(path, 2) ~= 2
        path = path.';
    end

    % Säkerställ exakt två kolumner: [row col]
    if size(path, 2) ~= 2
        error('Path måste vara Nx2 eller 2xN.');
    end
end