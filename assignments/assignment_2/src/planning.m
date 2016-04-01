function [samples, milestones] = sample_map(nb_samples, map, map_min, map_max, pos_start, pos_end)
    map_range = map_max - map_min;
    samples_x = int32(map_range(1) * rand(nb_samples, 1) + map_min(1));
    samples_y = int32(map_range(2) * rand(nb_samples, 1) + map_min(2));
    samples = [samples_x, samples_y];

    milestones = [
        pos_start(1:2) / 0.1;
        pos_end / 0.1
    ];
    for i = 1:length(samples)
        sample = samples(i, :);

        if map(sample(1), sample(2)) == 0
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
            dx = nodes(i, 1) - nodes(j, 1);
            dy = nodes(i, 2) - nodes(j, 2);
            d = sqrt(dx^2 + dy^2);

            if (d > dmax)
                dmax = d;
                start = i;
                finish = j;
            end
        end
    end
end

function dmax = calculate_max_edge_distance(nodes)
    dmax = 0;
    start = 0;
    finish = 0;
    n = length(nodes);

    for i = 1:n
        for j = i:n
            dx = nodes(i, 1) - nodes(j, 1);
            dy = nodes(i, 2) - nodes(j, 2);
            d = sqrt(dx^2 + dy^2);

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

        if (d ~= 0 && d < dmin)
            dmin = d;
            index = i;
        end
    end

    node = nodes(index, :);
    index = index;
end

function edges = connect_edges(nodes, nb_edges)
    n = length(nodes);
    edges = zeros(n, n);
    dists = zeros*ones(n, n);

    for i = 1:n
        % calculate distances between nodes
        for j = 1:n
            dx = nodes(j, 1) - nodes(i, 1);
            dy = nodes(j, 2) - nodes(i, 2);
            d(j) = sqrt(dx^2 + dy^2);
        end

        % sort calculated distances between nodes
        [d2, indicies] = sort(d);

        % create edges to p closest nodes
        for j = 1:min(nb_edges, length(indicies))
            if (i ~= indicies(j))
                edges(i, indicies(j)) = 1;
                edges(indicies(j), i) = 1;
            end
        end
    end
end

function edges = connect_edges_omap(map, nodes, nb_edges)
    n = length(nodes);
    edges = zeros(n, n);
    dists = zeros*ones(n, n);

    for i = 1:n
        % calculate distances between nodes
        for j = 1:n
            dx = nodes(j, 1) - nodes(i, 1);
            dy = nodes(j, 2) - nodes(i, 2);
            d(j) = sqrt(dx^2 + dy^2);
        end

        % sort calculated distances between nodes
        [d2, indicies] = sort(d);
        length(indicies)

        % create edges to p closest nodes
        for j = 1:min(nb_edges, length(indicies))
            x0 = nodes(i, 1);
            y0 = nodes(i, 2);
            x1 = nodes(indicies(j), 1);
            y1 = nodes(indicies(j), 2);
            collide = omap_collision(map, x0, y0, x1, y1);

            if collide == 0
                edges(i, indicies(j)) = 1;
                edges(indicies(j), i) = 1;
                % plot([x0, x1], [y0, y1], 'g-');
                % drawnow;
            else
                edges(i, indicies(j)) = 0;
                edges(indicies(j), i) = 0;
                % plot([x0, x1], [y0, y1], 'r-');
                % drawnow;
            end
        end
    end
end

function collide = omap_collision(map, x0, y0, x1, y1)
    loop = 1;
    collide = 0;
    bline = bresenham_line_setup(x0, y0, x1, y1);
    [x_max, y_max] = size(map);

    if (x0 == x1 && y0 == y1)
        collide = 0;
        return;
    end

    while (loop)
        % bline.x0
        % bline.y0
        % disp(' ');
        % disp(' ');
        [bline, loop] = bresenham_line_step(bline);

        if bline.x0 > x_max || bline.x0 < 0
            collide = 0;
            return;

        elseif bline.y0 > y_max || bline.y0 < 0
            collide = 0;
            return;

        elseif map(bline.x0, bline.y0) == 1
            collide = 1;
            loop = 0;
            return;
        end
        % plot(bline.x0, bline.y0, 'r.--');
        % drawnow;
    end
end

function dists = calculate_edge_distances(nodes, edges)
    n = length(nodes);
    dists= zeros(n,n);

    for i = 1:n
        for j = i:n
            if (edges(i, j))
                dx = nodes(i, 1) - nodes(j, 1);
                dy = nodes(i, 2) - nodes(j, 2);
                dists(i, j) = sqrt(dx^2 + dy^2);
                dists(j, i) = dists(i, j);
            end
        end
    end
end

function [spath, sdist] = backtrack_path(nodes, closed_set, dists, start, finish)
    done = 0;
    cur = finish;
    curC = find(closed_set(:, 1) == finish);
    prev =  closed_set(curC, 2);
    spath = [cur];
    sdist = 0;

    while (~done)
        if (prev == start)
            done = 1;
        end

        cur = prev;
        curC = find(closed_set(:,1) == cur);
        prev = closed_set(curC,2);
        spath = [cur, spath];
        sdist = sdist + dists(spath(1), spath(2));
    end

    spath = [nodes(spath, :)];
end

function [spath, sdist] = shortest_path(mode, nodes, edges, start, finish)
    % check mode
    if (mode == 1)
        % A-star
        disp('Running A-star mode');
    elseif (mode == 2)
        % Dijkstra
        disp('Running Dijkstra mode');
    else
        disp('Invalid mode!');
        return;
    end

    % setup
    done = 0;
    dists = calculate_edge_distances(nodes, edges);
    dmax = calculate_max_edge_distance(nodes);
    open_set = [start 0 dmax 0];  % (node, backtrack, lbound cost, curr cost)
    closed_set = [];  % (node, backtrack, lbound cost, curr cost)

    while (!done)
        % Find best node in open set
        [val, best] = min(open_set(:,3));
        bestnode = open_set(best,:);

        % Move best to closed set
        closed_set = [closed_set; bestnode];

        % Check end condition
        if (length(open_set) == 0)
            disp('No solution!');
            return;
        elseif (bestnode(1) == finish)
            done = 1;
            continue;
        end

        % Get all neighbours of best node
        neigh = find(edges(bestnode(1),:) == 1);

        % Process each neighbour
        for i = 1:length(neigh)
            % if neighbour is in closed set, skip
            found = find(closed_set(:,1) == neigh(i), 1);
            if (length(found) == 1)
                continue;
            end
            dx = nodes(neigh(i), 1) - nodes(finish, 1);
            dy = nodes(neigh(i), 2) - nodes(finish, 2);
            dtogo = sqrt(dx^2 + dy^2);
            dcur = bestnode(4) + dists(bestnode(1), neigh(i));
            found = find(open_set(:,1) == neigh(i),1);

            % if neighbour is not in open set, add it
            if (length(found) == 0)
                if (mode == 1)
                    % Astar
                    open_set = [open_set; neigh(i) bestnode(1) dtogo+dcur dcur];
                elseif (mode == 2)
                    % Dijkstra
                    open_set = [open_set; neigh(i) bestnode(1) dcur dcur];
                end

            % if neighbour is in open set, check if new route is better
            else
                if (dcur < open_set(found, 4))
                    if (mode == 1)
                        % Astar
                        open_set(found, :) = [neigh(i) bestnode(1) dtogo+dcur dcur];
                    elseif (mode == 2)
                        % Dijkstra
                        open_set(found, :) = [neigh(i) bestnode(1) dcur dcur];
                    end
                end
            end
        end

        % remove best node from open set
        open_set = open_set([1:best-1 best+1:end], :);

        % plot active nodes for this step
        % plot_active_nodes(1, nodes, bestnode, neigh, closed_set);
    end

    % find and plot final path through back tracing
    [spath, sdist] = backtrack_path(nodes, closed_set, dists, start, finish);
end

function [spath, sdist] = astar(nodes, edges, start, finish)
    [spath, sdist] = shortest_path(1, nodes, edges, start, finish);
end

function path = dijkstra(nodes, edges, start, finish)
    [spath, sdist] = shortest_path(2, nodes, edges, start, finish);
end

function bline = bresenham_line_setup(x0, y0, x1, y1)
    % diff in x and y
    dx = abs(x1 - x0);
    dy = abs(y1 - y0);

    % how to increment in x and y based on (x0, y0) and (x1, y1)
    sx = 0;
    if x0 < x1
        sx = 1;
    else
        sx = -1;
    end

    sy = 0;
    if y0 < y1
        sy = 1;
    else
        sy = -1;
    end

    % error and incrementers
    err = 2 * dy - dx;
    inc1 =  2 * dy;
    inc2 =  2 * dy - 2 * dx;

    % struct holding all variables
    bline = struct(
        'x0', x0,
        'y0', y0,
        'x1', x1,
        'y1', y1,
        'dx', dx,
        'dy', dy,
        'sx', sx,
        'sy', sy,
        'inc1', inc1,
        'inc2', inc2,
        'err', err
    );
end

function [bline, ok] = bresenham_line_step(bline)
    if bline.x0 == bline.x1 && bline.y0 == bline.y1
        ok = 0;
        return;
    end

    bline.x0 = bline.x0 + bline.sx;
    if bline.err < 0
        bline.err = bline.err + bline.inc1;
    else
        bline.err = bline.err + bline.inc2;
        bline.y0 = bline.y0 + bline.sy;
    end
    ok = 1;
end

function bresenham_line(x0, y0, x1, y1)
    bline = bresenham_line_setup(x0, y0, x1, y1);

    while (1)
        [bline, ok] = bresenham_line_step(bline);
        if ok == 0
            return;
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
