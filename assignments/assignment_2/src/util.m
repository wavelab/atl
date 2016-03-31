function p = closest_point(point, edge)
    x0 = point(1);
    y0 = point(2);
    x1 = edge(1);
    y1 = edge(2);
    x2 = edge(3);
    y2 = edge(4);

    % pre-check
    if x1 == x2
        p = [x1, y0];
        return;
    elseif y1 == y2
        p = [x0, y1];
        return;
    end

    % find closest point
    m1 = (y2 - y1) / (x2 - x1);
    m2 = -1 / m1;
    x = (m1 * x1 - m2 * a + b - y1) / (m1 - m2);
    y = m2 * (x - a) + b;

    % check results
    if x < min(x1, x2)
        x = min(x1, x2);
    elseif x > max(x1, x2)
        x = max(x1, x2);
    end
    if y < min(y1, y2)
        y = min(y1, y2);
    elseif y > max(y1, y2)
        y = max(y1, y2);
    end

    % answer
    p = [x, y];
end

function dist = point_edge_dist(p, edge)
    % NOTE: CALCULATES PERPENDICULAR DISTANCE ONLY
    %       for euclidean distance use distancePointEdge()
    x0 = p(1);
    y0 = p(2);
    x1 = edge(1);
    y1 = edge(2);
    x2 = edge(3);
    y2 = edge(4);

    n = ((y2 - y1) * x0) - ((x2 - x1) * y0) + (x2 * y1) - (y2 * x1);
    d = sqrt((y2 - y1)^2 + (x2 - x1)^2);

    dist = abs(n) / d;
end

function p = dist_point_on_edge(d, edge)
    lp1 = edge(1);
    lp2 = edge(2);

    % calculate unit vector
    v = lp2 - lp1;
    u = v / norm(v);

    % calculate distant point along edge
    p = lp1 + d * u;
end

function angle = atan3(y, x)
    angle = atan2(y, x);

    if y < 0
        angle = angle + 2 * pi;
    end
end

function dist = dist_between_points(x, y)
    diff = x - y;
    dist = norm(diff, 2);
end

function wp_reached = waypoint_reached(position, waypoint, threshold)
    wp_reached = 0;

    if dist_between_points(position, waypoint) < threshold
        wp_reached = 1;
        return;
    else
        wp_reached = 0;
        return;
    end
end

function drawbox(fig_index, x, y, h, scale)
    % function drawbox(x,y,h,scale,fig)
    % This function plots a box at position x,y heading h and size scale on
    % figure number fig.


    % car outline
    box = [-1 -0.5; 1 -0.5; 1 0.5; -1 0.5; -1 -0.5];

    % size scaling
    box = scale*box;

    % rotation matrix
    R = [cos(h) -sin(h); sin(h) cos(h)];
    box = (R*box')';

    % centre
    box(:,1) = box(:,1)+x;
    box(:,2) = box(:,2)+y;

    % plot
    figure(fig);
    plot(box(:,1), box(:,2), 'b','LineWidth', 2);
    axis equal
end

function plot_waypoints(fig_index, waypoints)
    figure(fig_index);
    hold on;

    subplot(3, 1, 1);
    plot(waypoints(:, 1), waypoints(:, 2), 'ro-');
end

function plot_trajectory(fig_index, x_store, carrot_store, t)
    figure(fig_index);
    hold on;

    subplot(3, 1, 1);

    % plot trajectory
    plot(x_store(1, :), x_store(2, :), 'bo');

    % plot carrot
    plot(carrot_store(1, :), carrot_store(2, :), 'go');

    % plot parameters
    title('Trajectory');
    xlabel('x-position (meters)');
    ylabel('y-position (meters)');
    % axis([4 32 4 12]);
    % axis([20 30 5 15]);
    axis equal;
end

function plot_animation(fig_index, x_store, carrot_store, t)
    figure(fig_index);
    hold on;

    for i = 1:length(t)
        % plot trajectory
        plot(x_store(1, i), x_store(2, i), 'bo');

        % plot carrot
        plot(carrot_store(1, i), carrot_store(2, i), 'go');

        % draw box
        if (mod(i, 5) == 0)
            drawbox(fig_index, x_store(1, i), x_store(2, i), x_store(3, i), 0.3);
        end

        % plot parameters
        axis equal;
        drawnow;
    end
end

function plot_controller(fig_index, c_store, t)
    figure(fig_index);
    hold on;

    subplot(3, 1, 2);
    head_err_deg = zeros(1, length(t));
    for i = 1:length(t)
        err = rad2deg(c_store(2, i));
        head_err_deg(i) = err;
    end
    plot(t, head_err_deg(:), 'ro-');
    title('Heading Error');
    xlabel('t (seconds)');
    ylabel('error (degrees)');
end

function plot_samples(fig_index, samples, milestones)
    figure(fig_index);
    hold on;

    % plot(samples(:, 1), samples(:, 2), 'r.');
    plot(milestones(:, 1), milestones(:, 2), 'bo');
end

function plot_map(fig_index, map, pos_start, pos_end, dxy)
    figure(fig_index);
    hold on;

    colormap('gray');
    imagesc(1 - map');

    % plot
    plot(pos_start(1) / dxy, pos_start(2) / dxy, 'ro', 'MarkerSize',10, 'LineWidth', 3);
    plot(pos_end(1) / dxy, pos_end(2) / dxy, 'gx', 'MarkerSize',10, 'LineWidth', 3 );

    % plot parameters
    axis equal
    pause
end
