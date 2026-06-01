function [path, pushes, pops, exec_time] = rrt_star_planner(map, startPos, goalPos)
    tic;
    pushes = 0; pops = 0;
    [rows, cols] = size(map);
    
    % Parameters
    max_iter = 20000;         % Max antal samplingar
    step_size = 5;          % Hur långt ett trädgren-steg får vara (i celler)
    search_radius = 40;      % Radie för RRT* Rewiring/Neighbor search
    goal_threshold = 12;     % Hur nära målet vi måste komma för att godkännas
    
    % Initiera trädet
    % Varje rad i 'nodes': [X, Y, Cost, Parent_Index]
    nodes = [startPos(1), startPos(2), 0, 0];
    pushes = pushes + 1;
    
    found_goal = false;
    best_goal_idx = 0;
    min_goal_cost = inf;
    
    for iter = 1:max_iter
        % 1. Sampla en slumpmässig punkt (med 10% chans att sikta direkt på målet)
        if rand() < 0.10
            x_rand = goalPos(1);
            y_rand = goalPos(2);
        else
            x_rand = randi([1, rows]);
            y_rand = randi([1, cols]);
        end
        
        % 2. Hitta närmaste noden i trädet
        dists_to_rand = sqrt((nodes(:,1) - x_rand).^2 + (nodes(:,2) - y_rand).^2);
        [~, nearest_idx] = min(dists_to_rand);
        x_near = nodes(nearest_idx, 1);
        y_near = nodes(nearest_idx, 2);
        
        % 3. Ta ett kontrollerat steg mot den samplade punkten
        theta = atan2(y_rand - y_near, x_rand - x_near);
        x_new = round(x_near + step_size * cos(theta));
        y_new = round(y_near + step_size * sin(theta));
        
        % Kontrollera kartans gränser
        if x_new < 1 || x_new > rows || y_new < 1 || y_new > cols
            continue;
        end
        
        % 4. Kollisionskontroll för den nya grenen
        if check_collision(map, x_near, y_near, x_new, y_new)
            continue; 
        end
        
        % 5. STAR-OPTIMERING Del A: Choose Parent (Hitta bästa föräldern)
        % Beräkna exakta avstånd från ALLA noder till det faktiska nya steget (x_new, y_new)
        dists_to_new = sqrt((nodes(:,1) - x_new).^2 + (nodes(:,2) - y_new).^2);
        neighbor_indices = find(dists_to_new <= search_radius);
        
        % FIX 1: Utgå från det exakta avståndet till x_new, inte till x_rand!
        min_cost = nodes(nearest_idx, 3) + dists_to_new(nearest_idx); 
        best_parent = nearest_idx;
        
        for i = 1:length(neighbor_indices)
            n_idx = neighbor_indices(i);
            cost_via_neighbor = nodes(n_idx, 3) + dists_to_new(n_idx);
            
            if cost_via_neighbor < min_cost
                if ~check_collision(map, nodes(n_idx, 1), nodes(n_idx, 2), x_new, y_new)
                    min_cost = cost_via_neighbor;
                    best_parent = n_idx;
                end
            end
        end
        
        % Lägg till den nya optimerade noden i trädet
        nodes = [nodes; x_new, y_new, min_cost, best_parent];
        pushes = pushes + 1;
        new_node_idx = size(nodes, 1);
        
        % 6. STAR-OPTIMERING Del B: Rewire (Koppla om existerande noder)
        for i = 1:length(neighbor_indices)
            n_idx = neighbor_indices(i);
            new_cost_for_neighbor = min_cost + dists_to_new(n_idx);
            
            if new_cost_for_neighbor < nodes(n_idx, 3)
                if ~check_collision(map, x_new, y_new, nodes(n_idx, 1), nodes(n_idx, 2))
                    % Beräkna hur mycket kostnaden sjunker för den här noden
                    cost_difference = nodes(n_idx, 3) - new_cost_for_neighbor;
                    
                    nodes(n_idx, 3) = new_cost_for_neighbor;
                    nodes(n_idx, 4) = new_node_idx; % Byt förälder
                    pops = pops + 1;
                    
                    % FIX 2: Propagera kostnadsminskningen ner till grannens alla barn!
                    nodes = propagate_cost(nodes, n_idx, cost_difference);
                end
            end
        end
        
        % 7. Kolla om vi har nått målet
        dist_to_goal = sqrt((x_new - goalPos(1))^2 + (y_new - goalPos(2))^2);
        if dist_to_goal <= goal_threshold
            if ~check_collision(map, x_new, y_new, goalPos(1), goalPos(2))
                found_goal = true;
                total_cost = min_cost + dist_to_goal;
                if total_cost < min_goal_cost
                    min_goal_cost = total_cost;
                    best_goal_idx = new_node_idx;
                end
            end
        end
    end
    
    % 8. Backtracka trädet för att generera den slutgiltiga snygga pathen
    path = [];
    if found_goal
        path = [goalPos(1), goalPos(2)];
        curr_idx = best_goal_idx;
        
        while curr_idx ~= 0
            path = [nodes(curr_idx, 1), nodes(curr_idx, 2); path];
            curr_idx = nodes(curr_idx, 4); 
            pops = pops + 1;
        end
    else
        warning('RRT* lyckades inte hitta målet inom max_iter.');
    end
    
    exec_time = toc;
end

% Rekursiv hjälpfunktion för att uppdatera kostnader nedåt i trädets grenar
function nodes = propagate_cost(nodes, parent_idx, cost_diff)
    % Hitta alla noder som har den omkopplade noden som förälder
    child_indices = find(nodes(:, 4) == parent_idx);
    for i = 1:length(child_indices)
        c_idx = child_indices(i);
        nodes(c_idx, 3) = nodes(c_idx, 3) - cost_diff; % Sänk kostnaden
        nodes = propagate_cost(nodes, c_idx, cost_diff); % Gå djupare
    end
end

% Lokalt hjälp-funktion för kollisionskontroll (Bresenham)
function collision = check_collision(map, x1, y1, x2, y2)
    collision = false;
    dx = abs(x2 - x1); dy = abs(y2 - y1);
    sx = sign(x2 - x1); sy = sign(y2 - y1);
    x = x1; y = y1;
    
    if dx > dy
        err = dx / 2;
        while x ~= x2
            if map(x, y) == inf
                collision = true; return;
            end
            x = x + sx; err = err - dy;
            if err < 0
                y = y + sy; err = err + dx;
            end
        end
    else
        err = dy / 2;
        while y ~= y2
            if map(x, y) == inf
                collision = true; return;
            end
            y = y + sy; err = err - dx;
            if err < 0
                x = x + sx; err = err + dy;
            end
        end
    end
    if map(x2, y2) == inf
        collision = true;
    end
end