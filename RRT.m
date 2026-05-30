clc; clear;

load FreiburgMaze.mat
Maze = FreiburgMaze;

Maze.goal = [197; 115];
Maze.start = [354; 242];

path = zeros(2, 0);
showMap(Maze, path);

[path, nodes] = planPathRRTStar(Maze);

fprintf('RRT* nodes: %d\n', size(nodes, 2));
showMap(Maze, path);


function [path, nodes] = planPathRRTStar(Maze)
    start = Maze.start(:);
    goal = Maze.goal(:);

    if isinf(Maze.map(start(1), start(2)))
        error('Start ligger i hinder eller okänt område.');
    end

    if isinf(Maze.map(goal(1), goal(2)))
        error('Goal ligger i hinder eller okänt område.');
    end

    maxIterations = 20000;
    stepSize = 3500;
    goalRadius = 45;
    rewireRadius = 80;
    goalBias = 0.10;

    % nodes:
    % row, col, cost, parentIndex
    nodes = [start(1); start(2); 0; 0];

    goalNodeIndex = 0;

    freeCells = find(Maze.map == 0);
    [freeRows, freeCols] = ind2sub(size(Maze.map), freeCells);

    for iter = 1:maxIterations

        % Slumpa punkt, ibland direkt mot goal
        if rand < goalBias
            sample = goal;
        else
            k = randi(length(freeRows));
            sample = [freeRows(k); freeCols(k)];
        end

        % Hitta närmaste nod
        diffs = nodes(1:2, :) - sample;
        dists = sqrt(sum(diffs.^2, 1));
        [~, nearestIndex] = min(dists);

        nearest = nodes(1:2, nearestIndex);
        newPoint = steer(nearest, sample, stepSize);

        if ~isFreePoint(Maze.map, newPoint)
            continue
        end

        if ~isCollisionFree(Maze.map, nearest, newPoint)
            continue
        end

        % Välj bästa parent bland närliggande noder
        diffs = nodes(1:2, :) - newPoint;
        nearDists = sqrt(sum(diffs.^2, 1));
        nearIndices = find(nearDists <= rewireRadius);

        bestParent = nearestIndex;
        bestCost = nodes(3, nearestIndex) + distance(nearest, newPoint);

        for i = 1:length(nearIndices)
            idx = nearIndices(i);
            candidate = nodes(1:2, idx);

            if isCollisionFree(Maze.map, candidate, newPoint)
                candidateCost = nodes(3, idx) + distance(candidate, newPoint);

                if candidateCost < bestCost
                    bestCost = candidateCost;
                    bestParent = idx;
                end
            end
        end

        % Lägg till ny nod
        newIndex = size(nodes, 2) + 1;
        nodes(:, newIndex) = [newPoint; bestCost; bestParent];

        % Rewire: försök förbättra närliggande noder via nya noden
        for i = 1:length(nearIndices)
            idx = nearIndices(i);

            if idx == bestParent
                continue
            end

            candidate = nodes(1:2, idx);
            newCost = bestCost + distance(newPoint, candidate);

            if newCost < nodes(3, idx) && isCollisionFree(Maze.map, newPoint, candidate)
                nodes(3, idx) = newCost;
                nodes(4, idx) = newIndex;
            end
        end

        % Kolla om vi nått målet
        if distance(newPoint, goal) <= goalRadius && isCollisionFree(Maze.map, newPoint, goal)
            goalCost = bestCost + distance(newPoint, goal);
            goalNodeIndex = size(nodes, 2) + 1;
            nodes(:, goalNodeIndex) = [goal; goalCost; newIndex];
            fprintf('Goal hittades vid iteration %d\n', iter);
            break
        end

        if mod(iter, 1000) == 0
            fprintf('Iteration %d, nodes %d\n', iter, size(nodes, 2));
        end
    end

    if goalNodeIndex == 0
        error('RRT* hittade ingen väg. Testa fler iterationer, större stepSize eller annat start/mål.');
    end

    % Bygg path baklänges från goal
    path = nodes(1:2, goalNodeIndex);
    parent = nodes(4, goalNodeIndex);

    while parent ~= 0
        path = [nodes(1:2, parent), path];
        parent = nodes(4, parent);
    end
end


function newPoint = steer(fromPoint, toPoint, stepSize)
    direction = toPoint - fromPoint;
    dist = sqrt(sum(direction.^2));

    if dist <= stepSize
        newPoint = round(toPoint);
    else
        direction = direction / dist;
        newPoint = round(fromPoint + direction * stepSize);
    end

    newPoint = newPoint(:);
end


function d = distance(a, b)
    d = sqrt(sum((a - b).^2));
end


function ok = isFreePoint(map, point)
    r = point(1);
    c = point(2);

    ok = r >= 1 && c >= 1 && ...
         r <= size(map, 1) && c <= size(map, 2) && ...
         map(r, c) == 0;
end


function ok = isCollisionFree(map, p1, p2)
    [rows, cols] = bresenham(p1(1), p1(2), p2(1), p2(2));

    ok = true;

    for i = 1:length(rows)
        r = rows(i);
        c = cols(i);

        if r < 1 || c < 1 || r > size(map, 1) || c > size(map, 2) || isinf(map(r, c))
            ok = false;
            return
        end
    end
end


function [rows, cols] = bresenham(r1, c1, r2, c2)
    r1 = round(r1);
    c1 = round(c1);
    r2 = round(r2);
    c2 = round(c2);

    dr = abs(r2 - r1);
    dc = abs(c2 - c1);

    sr = sign(r2 - r1);
    sc = sign(c2 - c1);

    rows = r1;
    cols = c1;

    if dc > dr
        err = dc / 2;
        while cols(end) ~= c2
            cols(end + 1, 1) = cols(end) + sc;
            err = err - dr;

            if err < 0
                rows(end + 1, 1) = rows(end) + sr;
                err = err + dc;
            else
                rows(end + 1, 1) = rows(end);
            end
        end
    else
        err = dr / 2;
        while rows(end) ~= r2
            rows(end + 1, 1) = rows(end) + sr;
            err = err - dc;

            if err < 0
                cols(end + 1, 1) = cols(end) + sc;
                err = err + dr;
            else
                cols(end + 1, 1) = cols(end);
            end
        end
    end
end


function showMap(Maze, path)
    figure;

    display_map = ones(size(Maze.map));
    display_map(isinf(Maze.map)) = 0;

    imagesc(display_map);
    colormap(gray);
    axis equal;
    axis tight;
    set(gca, 'YDir', 'normal');
    hold on;

    plot(Maze.start(2), Maze.start(1), 'bs', ...
        'MarkerFaceColor', 'b', ...
        'MarkerSize', 10);

    plot(Maze.goal(2), Maze.goal(1), 'ys', ...
        'MarkerFaceColor', 'y', ...
        'MarkerSize', 10);

    if ~isempty(path)
        plot(path(2, :), path(1, :), 'g-', 'LineWidth', 2);
        plot(path(2, :), path(1, :), 'gs', ...
            'MarkerFaceColor', 'g', ...
            'MarkerSize', 3);
    end

    title('RRT* Path Planning Map');
    hold off;
end