source('util.m');
source('planning.m');

I = imread('IGVCmap.jpg');
map = im2bw(I, 0.7);  % Convert to 0 - 1 image
map = 1 - flipud(map)'; % Convert to 0 free, 1 occupied and flip.
% map = zeros(100, 100);
[M, N] = size(map);  % Map size
map_min = [1 1];
map_max = [M N];

% robot start position
dxy = 0.1;
pos_start = [40 5 pi];
pos_end = [50 10];


function path = backtrack_path(closed_set, nodes, start, finish)
    done = 0;
    cur = finish;
    curC = find(closed_set(:,1) == finish);
    prev =  closed_set(curC,2);
    i = 2;

    path = [];
    while (!done)
        if (prev == start)
            done = 1;
        end

        if (cur ~= start && cur ~= finish)
            path = [path; nodes(cur, :)];
        end

        figure(1);
        hold on;
        plot([nodes(prev,1) nodes(cur,1)], [nodes(prev,2) nodes(cur,2)], 'g', 'LineWidth', 2)
        cur = prev;
        curC = find(closed_set(:, 1) == cur);
        prev = closed_set(curC, 2);
    end
    Cend = find(closed_set(:,1) == finish);
    plen = closed_set(Cend,4);

    % return shortest path
    node_start = nodes(start, :);
    node_finish = nodes(finish, :);
    path = [node_start; path; node_finish];
end

function [spath,sdist] = shortestpath(nodes, edges, start, finish)
%SHORTESTPATH Find shortest path using A-star search
% Inputs:
%   nodes: list of n node locations in 2D
%   edges: nXn connectivity of nodes (1 = connected), symmetric only uses
%           upper triangle
%   start: index of start node
%   finish: index of finish node

    % Find edge lengths
    n = length(nodes);
    dists= zeros(n,n);
    for i = 1:n
        for j = i:n
            if (edges(i,j))
                dx = nodes(i, 1) - nodes(j, 1);
                dy = nodes(i, 2) - nodes(j, 2);
                dists(i,j) = sqrt(dx^2 + dy^2);
                % dists(i,j) = norm(nodes(i,:)-nodes(j,:));
                dists(j,i) = dists(i,j);
            end
        end
    end

    % Initialize open set (node, backtrack, lower bound cost, current cost)
    dmax = 0;
    for i = 1:n
        for j = i:n
            dx = nodes(i, 1) - nodes(j, 1);
            dy = nodes(i, 2) - nodes(j, 2);
            d = sqrt(dx^2 + dy^2);

            if (d > dmax)
                dmax = d;
            end
        end
    end
    % dmax = norm(nodes(start,:)-nodes(finish,:));
    O = [start 0 dmax 0];

    % Initialize closed set (same as open set)
    C = [];
    done = 0;
    t = 0;

    % Main algorithm
    while (~done)
        % Check if open set is empty
        if (length(O(:,1))==0)
            spath = [];
            sdist = 0;
            return;
        end
        % Find best node in open set
        [val, best] = min(O(:,3));
        bestnode = O(best,:);

        % Move best to closed set
        C = [C; bestnode];

        % Check end condition
        if (bestnode(1)==finish)
            done = 1;
            continue;
        end

        % Get all neighbours of best node
        neigh = find(edges(bestnode(1),:)==1);

        % Process each neighbour
        for i=1:length(neigh)
            found = find(C(:,1)==neigh(i),1);
            if (length(found)==1)
                continue;
            end
            dx = nodes(neigh(i), 1) - nodes(finish, 1);
            dy = nodes(neigh(i), 2) - nodes(finish, 2);
            dtogo = sqrt(dx^2 + dy^2);
            % dtogo = norm(nodes(neigh(i),:)-nodes(finish,:));
            dcur = bestnode(4)+dists(bestnode(1),neigh(i));
            found = find(O(:,1)==neigh(i),1);
            if (length(found)==0)
                O = [O; neigh(i) bestnode(1) dtogo+dcur dcur];
            else
                if (dcur < O(found,4))
                    O(found,:) = [neigh(i) bestnode(1) dtogo+dcur dcur];
                end
            end
        end
        O = O([1:best-1 best+1:end],:);
    end

    % Find final path through back tracing
    done = 0;
    cur = finish;
    curC = find(C(:,1)==finish);
    prev =  C(curC,2);
    spath = [cur];
    sdist = 0;
    while (~done)
        if (prev == start)
            done = 1;
        end
        cur = prev;
        curC = find(C(:,1)==cur);
        prev = C(curC,2);
        spath = [cur, spath];
        sdist = sdist+dists(spath(1),spath(2));
    end
end


% function path = shortest_path_custom(mode, map, nodes, start, finish, dmax, e, D)
%     % open set (node, backtrack, lower bound cost, current cost)
%     O = [start 0 dmax 0];
%
%     % closed set (node, backtrack, lower bound cost, current cost)
%     C = [];
%
%     % check mode
%     if (mode == 1)
%         % A-star
%         disp('Running A-star mode');
%     elseif (mode == 2)
%         % Dijkstra
%         disp('Running Dijkstra mode');
%     else
%         disp('Invalid mode!');
%     end
%
%     % find shortest path
%     t = 0;
%     done = 0;
%
%     while (!done)
%         t = t + 1;
%         % Find best node in open set
%         [val, best] = min(O(:,3));
%         bestnode = O(best,:);
%
%         % Move best to closed set
%         C = [C; bestnode];
%
%         % Check end condition
%         if (bestnode(1) == finish)
%             done = 1;
%             continue;
%         end
%
%         % Get all neighbours of best node
%         neigh = find(e(bestnode(1),:) == 1);
%
%         % Process each neighbour
%         disp('processing neighbour');
%         ok = 0;
%         for i = 1:length(neigh)
%             % if neighbour is in closed set, skip
%             found = find(C(:,1) == neigh(i), 1);
%             if (length(found) == 1)
%                 continue;
%             end
%
%             % if line to neighbour collides, skip
%             x0 = nodes(neigh(i), 1);
%             y0 = nodes(neigh(i), 2);
%             x1 = nodes(finish, 1);
%             y1 = nodes(finish, 2);
%
%             collide = omap_collision(map, x0, y0, x1, y1);
%             % node_start = nodes(neigh(i), :)
%             % node_finish = nodes(finish, :)
%             % collide
%
%             if collide == 0
%                 % calculate distance between neighbour and node
%                 dx = nodes(neigh(i), 1) - nodes(finish, 1);
%                 dy = nodes(neigh(i), 2) - nodes(finish, 2);
%                 dtogo = sqrt(dx^2 + dy^2);
%                 dcur = bestnode(4) + D(bestnode(1), neigh(i));
%                 found = find(O(:,1) == neigh(i),1);
%
%                 % if neighbour is not in open set, add it
%                 if (length(found) == 0)
%                     if (mode == 1)
%                         % Astar
%                         O = [O; neigh(i) bestnode(1) dtogo+dcur dcur];
%                     elseif (mode == 2)
%                         % Dijkstra
%                         O = [O; neigh(i) bestnode(1) dcur dcur];
%                     end
%
%                 % if neighbour is in open set, check if new route is better
%                 else
%                     if (dcur < O(found, 4))
%                         if (mode == 1)
%                             % Astar
%                             O(found, :) = [neigh(i) bestnode(1) dtogo+dcur dcur];
%                         elseif (mode == 2)
%                             % Dijkstra
%                             O(found, :) = [neigh(i) bestnode(1) dcur dcur];
%                         end
%                     end
%                 end
%
%                 ok = 1;
%             end
%
%         end
%
%         if ok == 0
%             disp('No solutions');
%             % nodes(i, :)
%             % hold on;
%             % plot(nodes(i, 1), nodes(i, 2), 'ro.');
%             % drawnow;
%             return;
%         end
%
%         % remove best node from open set
%         O = O([1:best-1 best+1:end],:);
%
%         % plot active nodes for this step
%         plot_active_nodes(1, nodes, bestnode, neigh, C);
%         drawnow;
%     end
%
%     % find and plot final path through backtracking
%     disp('backtracking results');
%     path = backtrack_path(C, nodes, start, finish);
% end



% plot
% sample map
nb_samples = 1000;
[samples, milestones] = sample_map(
    nb_samples,
    map,
    map_min,
    map_max,
    pos_start(1:2),
    pos_end
);
plot_samples(1, samples, milestones);
plot_map(1, map, pos_start, pos_end, 0.1);


% create graph, find furthest distance between nodes and search
[edges, dists] = create_closest_edges(map, milestones, 40);
% milestones(edges, :)
% edges(1, 1)
% for i = 1:length(edges)
%     milesones(i, :)
% end
dmax = calculate_furthest_distance_betweeen_nodes(milestones);
[spath, sdist] = shortestpath(milestones, edges, 1, 2);
for i = 1:length(spath) - 1
    plot(
        milestones(spath(i:i + 1), 1),
        milestones(spath(i:i + 1), 2),
        'g--',
        'LineWidth',
        3
    );
end
% spath
% path = shortest_path_custom(1, map, milestones, 1, 2, dmax, edges, dists);
% path = shortest_path(1, milestones, 1, 2, dmax, edges, dists)
print -djpg -color planning.jpg

drawnow;
pause;
