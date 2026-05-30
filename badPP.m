clear all
clc

load IntelMaze.mat
Maze = IntelMaze;

% Alternativt:
% load FreiburgMaze.mat
% Maze = FreiburgMaze;

start = Maze.start;

seenObstacles = zeros(size(Maze.map));
seenObstacles = Maze.map; % kommentera bort för okänd karta

aStar = 1; % 1 = A*, 0 = Dijkstra

traversedPath = zeros(2, 0);
arrived = false;
pushes = 0;
pops = 0;

showMap(IntelMaze, traversedPath);

while arrived == false
%init
openQueue = zeros(5,0); %x, y, cost, parent x, parent y
startH = aStar * heuristic(start, IntelMaze.goal);
closedQueue = [start(1); start(2); startH; 0; 0;];
path = zeros(2, 0);

foundGoal = false;

%A*
target = closedQueue(:, end);
while(foundGoal == false)

    %Look at the adjacent nodes to most recent node in closedQueue
    for relRow = -1:1:1
        for relCol = -1:1:1

            %Skip the targer itself
            if relCol == 0 && relRow == 0
                continue
            end

            lookAt = [target(1) + relRow; target(2) + relCol];

            if lookAt(1) < 1 || lookAt(2) < 1 || ...
                lookAt(1) > size(Maze.map, 1) || lookAt(2) > size(Maze.map, 2)
                continue
            end

            %See if the node is already visited and also not an obstacle
            if ~any(all(closedQueue(1:2, :) == lookAt, 1)) &&...
                    seenObstacles(lookAt(1), lookAt(2)) == 0

                isInOpen = false;
                if  any(all(openQueue(1:2, :) == lookAt, 1))
                    isInOpen = true;
                end

                %calculate cost
                cost = target(3) - aStar * heuristic(target(1:2), IntelMaze.goal);
                %cost = cost + sqrt(relRow^2 + relCol^2);
                cost = cost + abs(relCol) + abs(relRow);
                cost = cost + aStar * heuristic(lookAt(1:2), IntelMaze.goal);

                if isInOpen == false
                    %add node to openQueue
                    pushes = pushes + 1;
                    newNode = [lookAt; cost; target(1:2)];
                    ind = find(openQueue(3, :) > newNode(3), 1);
                    if isempty(ind)
                        openQueue = [openQueue, newNode]; %new biggest
                    else
                        openQueue = [openQueue(:, 1:ind-1), newNode, openQueue(:, ind:end)];
                    end
                else
                    %Update the cost and parent
                    ind = find(all(openQueue(1:2, :) == lookAt, 1), 1);
                    if cost < openQueue(3, ind)
                        openQueue(3, ind) = cost;
                        openQueue(4:5, ind) = target(1:2);
                    end
                end
            end
        end
    end

    %move the lowest cost node from open queue to closed queue
    pops = pops + 1;
    movedNode = openQueue(:, 1);
    closedQueue = [closedQueue, movedNode];
    openQueue = openQueue(:, 2:end);

    target = movedNode;

    %Once the goal is added stop
    if movedNode(1:2)' == IntelMaze.goal
        foundGoal = true;
    end
end

%create path from closed queue
path = [path, closedQueue(1:2, end)];
nextParent = closedQueue(4:5, end);

for i = numel(closedQueue(1, :)) - 1: -1:1
    if closedQueue(1:2, i) == nextParent
        %add to path
        path = [path, closedQueue(1:2, i)];
        nextParent = closedQueue(4:5, i);
    end
end

%See if/where we hit an obstacle
path = fliplr(path);
firstObstacleInd = 0;
for i = 1:1:numel(path(1, :))
    if IntelMaze.map(path(1, i), path(2, i)) ~= 0
        seenObstacles(path(1, i), path(2, i)) = 1;
        start = traversedPath(:, end);
        break
    else
        traversedPath = [traversedPath, path(:, i)];
    end
end

if traversedPath(:, end) == IntelMaze.goal
    arrived = true;
end
end

fprintf('pushes %d, pops %d', pushes, pops)
showMap(IntelMaze, traversedPath);

%% Functions

%Manhattan
function y = heuristic(a, b)
    y = abs(a(1) - b(1)) + abs(a(2) - b(2));
end

function showMap(Maze, path)
    figure;
    
    % Gör en bildmatris:
    % 1 = fri yta / okänt, 0 = hinder
    display_map = ones(size(Maze.map));
    display_map(isinf(Maze.map)) = 0;

    imagesc(display_map);
    colormap(gray);
    axis equal;
    axis tight;
    set(gca, 'YDir', 'normal');
    hold on;

    % Start och mål
    plot(Maze.start(2), Maze.start(1), 'bs', ...
        'MarkerFaceColor', 'b', ...
        'MarkerSize', 10);

    plot(Maze.goal(2), Maze.goal(1), 'ys', ...
        'MarkerFaceColor', 'y', ...
        'MarkerSize', 10);

    % Path
    if ~isempty(path)
        plot(path(2, :), path(1, :), 'g-', ...
            'LineWidth', 2);

        plot(path(2, :), path(1, :), 'gs', ...
            'MarkerFaceColor', 'g', ...
            'MarkerSize', 3);
    end

    title('Path Planning Map');
    hold off;
end