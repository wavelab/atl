function [samples, milestones] = sample_map(nb_samples, map, xR, xMin, pos_start, pos_end)
    samples_x = int32(xR(1) * rand(nb_samples, 1) + xMin(1));
    samples_y = int32(xR(2) * rand(nb_samples, 1) + xMin(2));
    samples = [samples_x, samples_y];

    milestones = [pos_start(1:2); pos_end;];
    for i = 1:length(samples)
        sample = samples(i, :);

        if map(sample(1), sample(2)) == 0
            map(sample(1), sample(2));
            milestones = [milestones; sample];
        end
    end
end

function [dmax, start, finish] = calculate_furthest_distance_betweeen_nodes(nodes)
    dmax = 0;
    start = 0;
    finish = 0;
    n = length(nodes);

    for i = 1:n
        for j = i:n
            d = norm(nodes(i,:) - nodes(j,:));

            if (d > dmax)
                dmax = d;
                start = i;
                finish = j;
            end
        end
    end
end

function [node, index] = closest_node(nodes, x, y)
    index = 0;
    i = length(nodes);
    random_node = nodes(i, :);
    dx = x - random_node(1);
    dy = y - random_node(2);
    dmin = sqrt(dx^2 + dy^2);

    for i = 1:length(nodes)
        dx = x - nodes(i, 1);
        dy = y - nodes(i, 2);
        d = sqrt(dx^2 + dy^2);

        if (d != 0 && d < dmin)
            dmin = d;
            index = i;
        end
    end

    node = nodes(index, :);
    index = index;
end

function [edges, dists] = create_closest_edges(nodes)
    p = 5;
    n = length(nodes);
    edges = sparse(zeros(n, n));
    dists = sparse(zeros*ones(n, n));

    for i = 1:n
        % calculate distances between nodes
        for j = 1:n
            dx = nodes(i, 1) - nodes(j, 1);
            dy = nodes(i, 2) - nodes(j, 2);
            d(j) = sqrt(dx^2 + dy^2);
        end

        % sort calculated distances between nodes
        [d2, indicies] = sort(d);

        % create edges to p closest nodes
        for j = 1:p
            if (i != indicies(j))
                edges(i, indicies(j)) = 1;
                edges(indicies(j), i) = 1;
                dists(i, indicies(j)) = d(indicies(j));
                dists(indicies(j), i) = d(indicies(j));
            end
        end
    end
end

function plot_nodes(fig_index, nodes, start, finish)
    figure(fig_index);
    hold on;
    n = length(nodes);

    plot(nodes(:, 1), nodes(:, 2), 'ko');
    for i = 1:n
        for j = i:n
            if (e(i, j)==1)
                plot([nodes(i, 1) nodes(j, 1)], [nodes(i, 2) nodes(j, 2)], 'k');
            end
        end
    end
    plot(nodes(start, 1), nodes(start, 2), 'bo', 'MarkerSize', 6, 'LineWidth', 2);
    plot(nodes(finish, 1), nodes(finish, 2), 'ro', 'MarkerSize', 6, 'LineWidth', 2);
end

function plot_active_nodes(fig_index, nodes, best_node, neigh, C)
    figure(1);
    hold on;
    plot(nodes(C(:,1), 1), nodes(C(:,1), 2), 'ko','MarkerSize',6,'LineWidth',2);
    plot(nodes(best_node(1), 1), nodes(best_node(1), 2), 'go','MarkerSize',6,'LineWidth',2);

    for i = 1:length(neigh)
        plot(nodes(neigh(i), 1), nodes(neigh(i), 2), 'mo');
        plot(
            [nodes(best_node(1), 1) nodes(neigh(i), 1)],
            [nodes(best_node(1), 2) nodes(neigh(i), 2)],
            'm'
        );
    end
    drawnow;
end

function shortest_path(mode, nodes, start, finish, dmax, e, D)
    % open set (node, backtrack, lower bound cost, current cost)
    O = [start 0 dmax 0];

    % closed set (node, backtrack, lower bound cost, current cost)
    C = [];

    % check mode
    if (mode == 1)
        % A-star
        disp('Running A-star mode');
    elseif (mode == 2)
        % Dijkstra
        disp('Running Dijkstra mode');
    else
        disp('Invalid mode!');
    end

    % find shortest path
    t = 0;
    done = 0;

    while (!done)
        t = t + 1;
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
        neigh = find(e(bestnode(1),:) == 1);

        % Process each neighbour
        for i = 1:length(neigh)
            % if neighbour is in closed set, skip
            found = find(C(:,1) == neigh(i), 1);
            if (length(found) == 1)
                continue;
            end
            dtogo = norm(nodes(neigh(i),:)-nodes(finish,:));
            dcur = bestnode(4)+D(bestnode(1),neigh(i));
            found = find(O(:,1)==neigh(i),1);

            % if neighbour is not in open set, add it
            if (length(found) == 0)
                if (mode == 1)
                    % Astar
                    O = [O; neigh(i) bestnode(1) dtogo+dcur dcur];
                elseif (mode == 2)
                    % Dijkstra
                    O = [O; neigh(i) bestnode(1) dcur dcur];
                end

            % if neighbour is in open set, check if new route is better
            else
                if (dcur < O(found, 4))
                    if (mode == 1)
                        % Astar
                        O(found,:) = [neigh(i) bestnode(1) dtogo+dcur dcur];
                    elseif (mode == 2)
                        % Dijkstra
                        O(found,:) = [neigh(i) bestnode(1) dcur dcur];
                    end
                end
            end
        end

        % remove best node from open set
        O = O([1:best-1 best+1:end],:);

        % plot active nodes for this step
        plot_active_nodes(1, nodes, bestnode, neigh, C);
    end

    % find and plot final path through back tracing
    done = 0;
    cur = finish;
    curC = find(C(:,1) == finish);
    prev =  C(curC,2);
    i = 2;

    path = [];
    while (!done)
        if (prev == start)
            done = 1;
        end

        if (cur != start && cur != finish)
            path = [path; nodes(cur, :)];
        end

        figure(1);
        hold on;
        plot([nodes(prev,1) nodes(cur,1)], [nodes(prev,2) nodes(cur,2)], 'g', 'LineWidth', 2)
        cur = prev;
        curC = find(C(:, 1) == cur);
        prev = C(curC, 2);
    end
    Cend = find(C(:,1) == finish)
    plen = C(Cend,4)

    % return shortest path
    node_start = nodes(start, :)
    node_finish = nodes(finish, :)
    path = [node_start; path; node_finish]
end

function astar(nodes, start, finish, dmax, e, D)
    shortest_path(1, nodes, start, finish, dmax, e, D);
end

function dijkstra(nodes, start, finish, dmax, e, D)
    shortest_path(2, nodes, start, finish, dmax, e, D);
end
