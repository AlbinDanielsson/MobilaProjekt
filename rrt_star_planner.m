function [path, pushes, pops, exec_time] = rrt_star_planner(map, startPos, goalPos)
    tic;
    pushes = 0; pops = 0;
    [rows, cols] = size(map);
    
    % Konfigurera parametrar för RRT* utifrån matrisstorlek
    max_iter = 10000;         % Max antal samplingar
    step_size = 25;          % Hur långt ett trädgren-steg får vara (i celler)
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
        
        % 2. Hitta närmaste noden i trädet som redan existerar
        dists = sqrt((nodes(:,1) - x_rand).^2 + (nodes(:,2) - y_rand).^2);
        [~, nearest_idx] = min(dists);
        x_near = nodes(nearest_idx, 1);
        y_near = nodes(nearest_idx, 2);
        
        % 3. Ta ett kontrollerat steg i riktning mot den samplade punkten
        theta = atan2(y_rand - y_near, x_rand - x_near);
        x_new = round(x_near + step_size * cos(theta));
        y_new = round(y_near + step_size * sin(theta));
        
        % Kontrollera att den nya punkten ligger inom kartan
        if x_new < 1 || x_new > rows || y_new < 1 || y_new > cols
            continue;
        end
        
        % 4. Kollisionskontroll för den tänkta nya grenen (Bresenham-check)
        if check_collision(map, x_near, y_near, x_new, y_new)
            continue; % Avbryt om grenen skär en vägg (inf)
        end
        
        % 5. STAR-OPTIMERING Del A: Hitta bästa föräldern (Choose Parent)
        % Leta efter grannar inom sökradien
        neighbor_dists = sqrt((nodes(:,1) - x_new).^2 + (nodes(:,2) - y_new).^2);
        neighbor_indices = find(neighbor_dists <= search_radius);
        
        min_cost = nodes(nearest_idx, 3) + dists(nearest_idx); % Kostnad via närmaste nod
        best_parent = nearest_idx;
        
        for i = 1:length(neighbor_indices)
            n_idx = neighbor_indices(i);
            cost_via_neighbor = nodes(n_idx, 3) + neighbor_dists(n_idx);
            
            if cost_via_neighbor < min_cost
                % Kolla om linjen mellan grannen och nya punkten är kollisionsfri
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
        
        % 6. STAR-OPTIMERING Del B: Koppla om existerande noder (Rewire)
        for i = 1:length(neighbor_indices)
            n_idx = neighbor_indices(i);
            new_cost_for_neighbor = min_cost + neighbor_dists(n_idx);
            
            % Om vägen via vår nyligen tillagda nod är kortare än grannens nuvarande
            if new_cost_for_neighbor < nodes(n_idx, 3)
                if ~check_collision(map, x_new, y_new, nodes(n_idx, 1), nodes(n_idx, 2))
                    nodes(n_idx, 3) = new_cost_for_neighbor;
                    nodes(n_idx, 4) = new_node_idx; % Byt förälder till den nya noden
                    pops = pops + 1;
                end
            end
        end
        
        % 7. Kolla om vi har nått målet
        dist_to_goal = sqrt((x_new - goalPos(1))^2 + (y_new - goalPos(2))^2);
        if dist_to_goal <= goal_threshold
            % Säkerställ en sista ren linje ända fram till exakta målkoordinaten
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
        % Lägg till det exakta målet först
        path = [goalPos(1), goalPos(2)];
        curr_idx = best_goal_idx;
        
        while curr_idx ~= 0
            path = [nodes(curr_idx, 1), nodes(curr_idx, 2); path];
            curr_idx = nodes(curr_idx, 4); % Gå till föräldern
            pops = pops + 1;
        end
    else
        warning('RRT* lyckades inte hitta målet inom max_iter. Prova att öka max_iter eller sänka inflate_cells.');
    end
    
    exec_time = toc;
end

% Lokalt hjälp-funktion för supersnabb kollisionskontroll längs en trädgren
function collision = check_collision(map, x1, y1, x2, y2)
    collision = false;
    % Använd Bresenhams linjealgoritm för att scanna sträckan cell-för-cell
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
    % Kontrollera även slutpunkten
    if map(x2, y2) == inf
        collision = true;
    end
end