function [path, pushes, pops, exec_time] = planner(map, startPos, goalPos, algoType)
    tic; % timer
    pushes = 0; pops = 0;
    [rows, cols] = size(map);
    
    g = inf(rows, cols);          
    f = inf(rows, cols);          
    parentX = zeros(rows, cols);  % To backtrack the path
    parentY = zeros(rows, cols);
    closed_set = false(rows, cols); 
    open_set = false(rows, cols);   % Nodes in queue
    
    % Start node (cost 0)
    g(startPos(1), startPos(2)) = 0;
    if strcmp(algoType, 'Astar')
        f(startPos(1), startPos(2)) = sqrt((startPos(1)-goalPos(1))^2 + (startPos(2)-goalPos(2))^2); % Euclidean distance
    else
        f(startPos(1), startPos(2)) = 0; % For dijkstra heuristic = 0
    end
    open_set(startPos(1), startPos(2)) = true;
    pushes = pushes + 1;
    
    % Grid directions: [dx, dy, step_cost]
    dirs = [1,0,1; -1,0,1; 0,1,1; 0,-1,1; 1,1,sqrt(2); 1,-1,sqrt(2); -1,1,sqrt(2); -1,-1,sqrt(2)];
    found = false;
    
    while any(open_set(:))
        % Find cell in open set with lowest f value
        open_indices = find(open_set);
        [~, min_idx] = min(f(open_indices));
        [curr_x, curr_y] = ind2sub([rows, cols], open_indices(min_idx));
        
        open_set(curr_x, curr_y) = false;
        closed_set(curr_x, curr_y) = true;
        pops = pops + 1;
        
        % If goal is reached stop searching
        if curr_x == goalPos(1) && curr_y == goalPos(2)
            found = true;
            break;
        end
        
        % Explore the 8 neighbors
        for i = 1:8
            nx = curr_x + dirs(i,1);
            ny = curr_y + dirs(i,2);
            cost = dirs(i,3);
            
            % Check map bounds, not a obstacle, and not already closed
            if nx >= 1 && nx <= rows && ny >= 1 && ny <= cols && map(nx,ny) ~= inf && ~closed_set(nx,ny)
                temp_g = g(curr_x, curr_y) + cost;
                
                if temp_g < g(nx, ny)
                    g(nx, ny) = temp_g;
                    parentX(nx, ny) = curr_x;
                    parentY(nx, ny) = curr_y;
                    
                    if strcmp(algoType, 'Astar')
                        h = sqrt((nx-goalPos(1))^2 + (ny-goalPos(2))^2);
                    else
                        h = 0;
                    end
                    f(nx, ny) = g(nx, ny) + h;
                    
                    % Push to queue if new
                    if ~open_set(nx, ny)
                        open_set(nx, ny) = true;
                        pushes = pushes + 1;
                    end
                end
            end
        end
    end
    
    % Backtrack from goal to start to generate path
    path = [];
    if found
        cx = goalPos(1); cy = goalPos(2);
        while cx ~= 0 && cy ~= 0 % 0 since parentX and parentY is 0,0 for start position
            path = [cx, cy; path];
            nx = parentX(cx, cy);
            ny = parentY(cx, cy);
            cx = nx; cy = ny;
        end
    end
    exec_time = toc;
end