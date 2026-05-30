function [path, pushes, pops, exec_time] = dstar_lite_planner(actual_map, startPos, goalPos, is_known)
    tic;
    pushes = 0; pops = 0;
    [rows, cols] = size(actual_map);
    
    if is_known
        known_map = actual_map; 
    else 
        known_map = zeros(rows, cols); 
    end
    
    g = inf(rows, cols);
    rhs = inf(rows, cols);
    rhs(goalPos(1), goalPos(2)) = 0;
    
    U = [goalPos(1), goalPos(2)]; % Priority queue
    pushes = pushes + 1;
    
    % Robot pos
    s_start = startPos; 
    
    dirs = [1,0,1; -1,0,1; 0,1,1; 0,-1,1; 1,1,sqrt(2); 1,-1,sqrt(2); -1,1,sqrt(2); -1,-1,sqrt(2)];
    
    % Calculate key values
    function k = calc_key(x, y)
        min_g_rhs = min(g(x,y), rhs(x,y));
        h = sqrt((x-s_start(1))^2 + (y-s_start(2))^2); % Current distance to robot
        k = [min_g_rhs + h, min_g_rhs];
    end
    
    % Caluclate new rhs value
    function update_vertex(ux, uy)
        if ~(ux == goalPos(1) && uy == goalPos(2))
            if known_map(ux, uy) == inf
                rhs(ux, uy) = inf; % Infinite rhs value for obstacles
            else
                min_rhs = inf;
                for d = 1:8
                    nx = ux + dirs(d,1); 
                    ny = uy + dirs(d,2); 
                    cost = dirs(d,3);
                    if nx >= 1 && nx <= rows && ny >= 1 && ny <= cols && known_map(nx,ny) ~= inf
                        min_rhs = min(min_rhs, cost + g(nx, ny));
                    end
                end
                rhs(ux, uy) = min_rhs;
            end
        end
        
        % Remove from queue
        idx = find(U(:,1) == ux & U(:,2) == uy);
        if ~isempty(idx), U(idx, :) = []; end
        
        % Add back if inconsistent
        if abs(g(ux, uy) - rhs(ux, uy)) > 1e-5 % Floating point errors could have been an issue (because of square root 2 in diagonal) so i added this.
            U = [U; ux, uy]; 
            pushes = pushes + 1;
        end
    end

    function compute_shortest_path()
        while ~isempty(U)
            % Find lowest key in queue
            min_k = [inf, inf]; min_idx = 0;
            for k = 1:size(U, 1)
                key = calc_key(U(k,1), U(k,2));
                if key(1) < min_k(1) - 1e-5 || (abs(key(1) - min_k(1)) < 1e-5 && key(2) < min_k(2) - 1e-5)
                    min_k = key; 
                    min_idx = k;
                end
            end
            
            start_k = calc_key(s_start(1), s_start(2));
            
            min_k_small = false;
            if min_k(1) < start_k(1) - 1e-5
                min_k_small = true;
            elseif abs(min_k(1) - start_k(1)) < 1e-5 && min_k(2) < start_k(2) - 1e-5
                min_k_small = true;
            end
            
            % Termination condition
            if ~min_k_small && abs(rhs(s_start(1), s_start(2)) - g(s_start(1), s_start(2))) < 1e-5
                break;
            end
            
            cx = U(min_idx, 1); 
            cy = U(min_idx, 2);
            U(min_idx, :) = []; 
            pops = pops + 1;
            
            k_old = min_k; 
            k_new = calc_key(cx, cy);
            if k_old(1) < k_new(1) - 1e-5 || (abs(k_old(1) - k_new(1)) < 1e-5 && k_old(2) < k_new(2) - 1e-5)
                U = [U; cx, cy]; pushes = pushes + 1;
            elseif g(cx, cy) > rhs(cx, cy) + 1e-5
                g(cx, cy) = rhs(cx, cy);
                for d = 1:8
                    nx = cx + dirs(d,1); ny = cy + dirs(d,2);
                    if nx >= 1 && nx <= rows && ny >= 1 && ny <= cols, update_vertex(nx, ny); end
                end
            else
                g(cx, cy) = inf; 
                update_vertex(cx, cy);
                for d = 1:8
                    nx = cx + dirs(d,1); ny = cy + dirs(d,2);
                    if nx >= 1 && nx <= rows && ny >= 1 && ny <= cols, update_vertex(nx, ny); end
                end
            end
        end
    end

    compute_shortest_path();
    path = s_start;
    
    % Motion loop
    while ~(s_start(1) == goalPos(1) && s_start(2) == goalPos(2))
        
        % Scan before moving
        if ~is_known
            changed = false;
            for d = 1:8
                nx = s_start(1) + dirs(d,1); 
                ny = s_start(2) + dirs(d,2);
                if nx >= 1 && nx <= rows && ny >= 1 && ny <= cols
                    if actual_map(nx, ny) == inf && known_map(nx, ny) ~= inf
                        known_map(nx, ny) = inf; 
                        changed = true; 
                        update_vertex(nx, ny); 
                    end
                end
            end

            if changed
                update_vertex(s_start(1), s_start(2));
                compute_shortest_path();
            end
        end
        
        % Travel to lowest cost neighbour
        best_n = s_start; min_v = inf;
        for d = 1:8
            nx = s_start(1) + dirs(d,1); ny = s_start(2) + dirs(d,2); cost = dirs(d,3);
            if nx >= 1 && nx <= rows && ny >= 1 && ny <= cols && known_map(nx,ny) ~= inf
                if cost + g(nx, ny) < min_v
                    min_v = cost + g(nx, ny); 
                    best_n = [nx, ny]; 
                end
            end
        end
        
        if isequal(best_n, s_start), break; end % Break we are stuck
        s_start = best_n; 
        path = [path; s_start];
        
    end
    exec_time = toc;
end