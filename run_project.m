clear; 
clc; 
close all;
% algorithm names
algos = {'Astar', 'RRTstar'};

fprintf('%s | %s | %s | %s | %s | %s | %s\n', 'MAP', 'CASE', 'ALGO', 'PUSHES', 'POPS','PATH LENGTH', 'TIME(s)');

% Loop over each maze
for map_number = 1:2
    
    % Load the map data
    if map_number == 1
        load('IntelMaze.mat'); 
        map = IntelMaze.map; IntelMaze.start = [291, 46]; IntelMaze.goal = [33, 225];
        s_pos = IntelMaze.start; g_pos = IntelMaze.goal; name = 'Intel Lab';
    else
        load('FreiburgMaze.mat'); 
        map = FreiburgMaze.map; FreiburgMaze.start = [197, 87]; FreiburgMaze.goal = [48, 149];
        s_pos = FreiburgMaze.start; g_pos = FreiburgMaze.goal; name = 'Freiburg Campus';
    end
    
    % known environment and unknown environment
    for case_type = 1:1
        if case_type == 1, case_lbl = 'Known'; else, case_lbl = 'Unknown'; end
        
        % run all algorithms
        for a = 1:length(algos)
            algo = algos{a};
            
            if strcmp(algo, 'DStarLite')
                [path, psh, pp, t] = dstar_lite_planner(map, s_pos, g_pos, (case_type == 1));
            elseif strcmp(algo, 'RRTstar')
                [path, psh, pp, t] = rrt_star_planner(map, s_pos, g_pos);
            else
                if case_type == 1
                    % Planner for A* or dijkstra for known
                    [path, psh, pp, t] = planner(map, s_pos, g_pos, algo);
                else
                    % Unknown planner A* or dijkstra
                    [path, psh, pp, t] = run_unknown_standard(map, s_pos, g_pos, algo);
                end
            end
           
            % Caluclate euclidean distance of the path
            path_length = 0;
                if ~isempty(path)
                    for i = 1:(size(path, 1) - 1)
                        p1 = path(i, :);
                        p2 = path(i+1, :);
                        step_dist = sqrt((p2(1) - p1(1))^2 + (p2(2) - p1(2))^2);
                        path_length = path_length + step_dist;
                    end
                end
            
            % Output
            fprintf('%s | %s | %s | %d | %d | %f | %f\n', name, case_lbl, algo, psh, pp,path_length, t);
            
            
            % Plotting
            figure('Name', sprintf('%s - %s - %s', name, case_lbl, algo));
            
            imagesc(map == 0); 
            colormap(gray); 
            hold all;
            
            if ~isempty(path)
                plot(path(:,2), path(:,1), 'g-', 'LineWidth', 2.5); 
            end
            
            plot(s_pos(2), s_pos(1), 'sb', 'MarkerFaceColor', 'b', 'MarkerSize', 10, 'color', 'b');
            plot(g_pos(2), g_pos(1), 'sy', 'MarkerFaceColor', 'y', 'MarkerSize', 10, 'color', 'y');
            
            title(sprintf('%s [%s Case] - %s', name, case_lbl, algo));
            axis equal; 
            axis([1 size(map,1) 1 size(map,2)]);          
            hold off;
            
        end
    end
    fprintf('------------------------------------------------------------------------\n');
end