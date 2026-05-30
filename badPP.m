clc; clear;

load FreiburgMaze.mat
Maze = FreiburgMaze;
Maze.goal = [197; 115];
Maze.start = [354; 242];
tic
%load IntelMaze.mat
%Maze = IntelMaze;
%Maze.goal = [274; 410];
%Maze.start = [550; 581];

aStar = 1; % 1 = A*, 0 = Dijkstra

path = zeros(2, 0);
showMap(Maze, path);
[path, pushes, pops] = planPath(Maze, aStar);

fprintf('pushes %d, pops %d\n', pushes, pops);
showMap(Maze, path);

function [path, pushes, pops] = planPath(Maze, aStar)
    start = Maze.start;
    goal = Maze.goal;

    openQueue = zeros(5, 0); % row, col, cost, parent row, parent col
    closedQueue = [start(1); start(2); aStar * heuristic(start, goal); 0; 0];

    pushes = 0;
    pops = 0;
    foundGoal = false;

    target = closedQueue(:, end);

    while ~foundGoal
        for relRow = -1:1
            for relCol = -1:1

                if relCol == 0 && relRow == 0
                    continue
                end

                lookAt = [target(1) + relRow; target(2) + relCol];

                if lookAt(1) < 1 || lookAt(2) < 1 || ...
                   lookAt(1) > size(Maze.map, 1) || ...
                   lookAt(2) > size(Maze.map, 2)
                    continue
                end

                if isinf(Maze.map(lookAt(1), lookAt(2)))
                    continue
                end

                if any(all(closedQueue(1:2, :) == lookAt, 1))
                    continue
                end

                isInOpen = any(all(openQueue(1:2, :) == lookAt, 1));

                old_g = target(3) - aStar * heuristic(target(1:2), goal);
                step_cost = abs(relRow) + abs(relCol);
                new_g = old_g + step_cost;
                new_f = new_g + aStar * heuristic(lookAt, goal);

                if ~isInOpen
                    pushes = pushes + 1;
                    newNode = [lookAt; new_f; target(1:2)];
                    openQueue = [openQueue, newNode];
                else
                    ind = find(all(openQueue(1:2, :) == lookAt, 1), 1);
                    if new_f < openQueue(3, ind)
                        openQueue(3, ind) = new_f;
                        openQueue(4:5, ind) = target(1:2);
                    end
                end
            end
        end

        if isempty(openQueue)
            error('Ingen väg hittades. Start och mål är troligen separerade av hinder.');
        end

        [~, bestInd] = min(openQueue(3, :));
        movedNode = openQueue(:, bestInd);
        openQueue(:, bestInd) = [];

        pops = pops + 1;
        closedQueue = [closedQueue, movedNode];
        target = movedNode;

        if isequal(movedNode(1:2), goal)
            foundGoal = true;
        end
    end

    path = closedQueue(1:2, end);
    nextParent = closedQueue(4:5, end);

    while ~isequal(nextParent, [0; 0])
        ind = find(all(closedQueue(1:2, :) == nextParent, 1), 1, 'last');
        path = [closedQueue(1:2, ind), path];
        nextParent = closedQueue(4:5, ind);
    end
end

toc
function y = heuristic(a, b)
    y = abs(a(1) - b(1)) + abs(a(2) - b(2));
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

    title('Path Planning Map');
    hold off;
end