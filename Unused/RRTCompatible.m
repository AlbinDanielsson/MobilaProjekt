clc; clear; close all;

% Kompatibel med kartor skapade av make_planning_maze:
% Maze.map   indexeras som map(row, col)


load FreiburgMaze.mat
Maze = FreiburgMaze;

Maze.start = [176; 147];
Maze.goal   = [165; 365];

% load IntelMaze.mat
% Maze = IntelMaze;

path = zeros(2, 0);
showMap(Maze, path);

[path, nodes] = planPathRRTStar(Maze);

fprintf('RRT* nodes: %d\n', size(nodes, 2));
showMap(Maze, path);


function [path, nodes] = planPathRRTStar(Maze)
    start = Maze.start(:);  % [x; y] = [col; row]
    goal = Maze.goal(:);    % [x; y] = [col; row]

    if ~isFreePoint(Maze.map, start)
        error('Start ligger i hinder, okant omrade eller utanfor kartan.');
    end

    if ~isFreePoint(Maze.map, goal)
        error('Goal ligger i hinder, okant omrade eller utanfor kartan.');
    end

    maxIterations = 20000;
    stepSize = 350;
    goalRadius = 45;
    rewireRadius = 80;
    goalBias = 0.10;

    % nodes: x, y, cost, parentIndex
    nodes = [start(1); start(2); 0; 0];

    goalNodeIndex = 0;

    [freeRows, freeCols] = find(Maze.map == 0);

    for iter = 1:maxIterations
        if rand < goalBias
            sample = goal;
        else
            k = randi(length(freeRows));
            sample = [freeCols(k); freeRows(k)];
        end

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

        newIndex = size(nodes, 2) + 1;
        nodes(:, newIndex) = [newPoint; bestCost; bestParent];

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
        error('RRT* hittade ingen vag. Testa fler iterationer, storre stepSize eller annat start/mal.');
    end

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
    col = point(1);
    row = point(2);

    ok = row >= 1 && col >= 1 && ...
         row <= size(map, 1) && col <= size(map, 2) && ...
         map(row, col) == 0;
end


function ok = isCollisionFree(map, p1, p2)
    [cols, rows] = bresenham(p1(1), p1(2), p2(1), p2(2));

    ok = true;

    for i = 1:length(rows)
        row = rows(i);
        col = cols(i);

        if row < 1 || col < 1 || ...
           row > size(map, 1) || col > size(map, 2) || ...
           isinf(map(row, col))
            ok = false;
            return
        end
    end
end


function [x, y] = bresenham(x1, y1, x2, y2)
    x1 = round(x1);
    y1 = round(y1);
    x2 = round(x2);
    y2 = round(y2);

    dx = abs(x2 - x1);
    dy = abs(y2 - y1);

    sx = sign(x2 - x1);
    sy = sign(y2 - y1);

    x = x1;
    y = y1;

    if dx > dy
        err = dx / 2;

        while x(end) ~= x2
            x(end + 1, 1) = x(end) + sx;
            err = err - dy;

            if err < 0
                y(end + 1, 1) = y(end) + sy;
                err = err + dx;
            else
                y(end + 1, 1) = y(end);
            end
        end
    else
        err = dy / 2;

        while y(end) ~= y2
            y(end + 1, 1) = y(end) + sy;
            err = err - dx;

            if err < 0
                x(end + 1, 1) = x(end) + sx;
                err = err + dy;
            else
                x(end + 1, 1) = x(end);
            end
        end
    end
end


function showMap(Maze, path)
    figure;

    displayMap = ones(size(Maze.map));
    displayMap(isinf(Maze.map)) = 0;

    imagesc(displayMap);
    colormap(gray);
    axis equal;
    axis tight;
    set(gca, 'YDir', 'normal');
    hold on;

    plot(Maze.start(1), Maze.start(2), 'bs', ...
        'MarkerFaceColor', 'b', ...
        'MarkerSize', 10);

    plot(Maze.goal(1), Maze.goal(2), 'ys', ...
        'MarkerFaceColor', 'y', ...
        'MarkerSize', 10);

    if ~isempty(path)
        plot(path(1, :), path(2, :), 'g-', 'LineWidth', 2);
        plot(path(1, :), path(2, :), 'gs', ...
            'MarkerFaceColor', 'g', ...
            'MarkerSize', 3);
    end

    title('RRT* Path Planning Map');
    hold off;
end