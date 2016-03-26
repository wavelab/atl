source('src/drawbox.m');


% simulation parameters
t_end = 8;
dt = 0.1;  % 10 Hz
t = 0:dt:t_end;

% waypoints
rect_x = 5;
rect_y = 10;
rect_w = 20;
rect_h = 5;

wp_1 = [rect_x, rect_y];
wp_2 = [rect_x + rect_w, rect_y];
wp_3 = [rect_x + rect_w, rect_y - rect_h];
wp_4 = [rect_x, rect_y - rect_h];

waypoints = [wp_1; wp_2; wp_3; wp_4; wp_1];

% model parameters
L = 0.3;
stddev_x = 0.02;
stddev_y = 0.02;
stddev_theta = deg2rad(1.0);
v_t = 3;

% bicycle states
delta_max = deg2rad(25); % max steering angle
x_t = [
    5;  % x
    12;  % y
    deg2rad(0);  % theta
];
x_store = zeros(length(x_t), length(t));
x_store(:, 1) = x_t;

% carrot states
carrot_t = [
    0;  % x
    0;  % y
];
carrot_store = zeros(length(carrot_t), length(t));
carrot_store(:, 1) = carrot_t;

% control parameters
k = 2.0;

% controller states
c_t = [
    % 10;  % crosstrack error
    % x_t(3);  % heading error
    0;
    0;
];
c_store = zeros(length(c_t), length(t));

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


function delta_t = calculate_steering_angle(c_t, k, v_t, delta_max)
    atan2(k * c_t(1), v_t)
    if (atan2(k * c_t(1), v_t) < 0)
        disp('FLIP2');
        delta_t = c_t(2) + (atan2(k * c_t(1), v_t) + 2 * pi);
    else
        delta_t = c_t(2) + atan2(k * c_t(1), v_t);
    end
    % delta_t
    % delta_t = atan2(k * c_t(1), v_t);

    % make sure steering angle does not exceed allowed
    if (delta_t > delta_max)
        delta_t = delta_max;
    elseif (delta_t < -delta_max)
        delta_t = -delta_max;
    end
end

function x = bicycle_update(x_prev, v_t, L, psi_t, delta_t, dt)
    x = [
        x_prev(1) + v_t * cos(psi_t - delta_t) * dt;
        x_prev(2) + v_t * sin(psi_t - delta_t) * dt;
        x_prev(3) + (v_t * tan(deg2rad(delta_t)) / L) * dt;
    ];
end

function new_carrot = calculate_new_carrot(x_t, r, wp_start, wp_end)
    % cross-track point on trajectory
    pt_on_line = closest_point(x_t, [wp_start, wp_end]);

    % calculate unit vector of trajectory
    % allows us to traverse distance r along the trajectory
    v = wp_end - wp_start;
    u = v / norm(v);

    % update carrot position
    new_carrot = pt_on_line + r * u;
end

function wp_reached = waypoint_reached(carrot_t, wp_start, wp_end)
    wp_reached = 0;

    % check x-axis
    if carrot_t(1) < min(wp_start(1), wp_end(1))
        carrot_t(1) = min(wp_start(1), wp_end(1));
        wp_reached = 1;
    elseif carrot_t(1) > max(wp_start(1), wp_end(1))
        carrot_t(1) = max(wp_start(1), wp_end(1));
        wp_reached = 1;
    end

    % check y-axis
    if carrot_t(2) < min(wp_start(2), wp_end(2))
        carrot_t(2) = min(wp_start(2), wp_end(2));
        wp_reached = 1;
    elseif carrot_t(2) > max(wp_start(2), wp_end(2))
        carrot_t(2) = max(wp_start(2), wp_end(2));
        wp_reached = 1;
    end
end

function [carrot_t, wp_index] = carrot_update(x_t, carrot_prev, r, waypoints, wp_index)
    wp_start = waypoints(wp_index, :);
    wp_end = waypoints(wp_index + 1, :);

    % calculate new carrot
    carrot_t = calculate_new_carrot(x_t, r, wp_start, wp_end);

    % check if carrot has reached waypoint
    wp_reached = waypoint_reached(carrot_t, wp_start, wp_end);
    if wp_reached == 1
        disp('new waypoint');

        % update waypoint index
        wp_index += 1;
        if wp_index == length(waypoints)
            wp_index = 1;
        end

        % recalculate carrot
        wp_start = waypoints(wp_index, :)
        wp_end = waypoints(wp_index + 1, :)
        carrot_t = calculate_new_carrot(x_t, r, wp_start, wp_end);

        % check x-axis
        if carrot_t(1) < min(wp_start(1), wp_end(1))
            carrot_t(1) = min(wp_start(1), wp_end(1));
        elseif carrot_t(1) > max(wp_start(1), wp_end(1))
            carrot_t(1) = max(wp_start(1), wp_end(1));
        end

        % check y-axis
        if carrot_t(2) < min(wp_start(2), wp_end(2))
            carrot_t(2) = min(wp_start(2), wp_end(2));
        elseif carrot_t(2) > max(wp_start(2), wp_end(2))
            carrot_t(2) = max(wp_start(2), wp_end(2));
        end
    end
end

function c_t = controller_update(c_prev, L, v_t, delta_t, dt)
    % controller state derivatives
    cd = [
        v_t * sin(c_prev(2) - delta_t);
        -(v_t * sin(delta_t)) / L;
    ];

    % controller state update
    c_t = [
        c_prev(1) + cd(1) * dt;
        c_prev(2) + cd(2) * dt;
    ];
end

function plot_trajectory(fig_index, x_store, carrot_store, t)
    figure(fig_index);
    hold on;

    for i = 1:2:length(t)
        % plot trajectory
        plot(x_store(1, 1:i), x_store(2, 1:i), 'bo');

        % plot carrot
        plot(carrot_store(1, i), carrot_store(2, i), 'go');

        % draw box
        % if (mod(i, 5) == 0)
        %     drawbox(x_store(1, i), x_store(2, i), x_store(3, i), 0.3, fig_index);
        % end

        % plot parameters
        axis([4 32 4 12]);
        axis equal;
        drawnow;
    end
end

function plot_rectangle(fig_index, p1, p2, p3, p4)
    figure(fig_index);
    hold on;

    points = [p1; p2; p3; p4; p1];
    plot(points(:, 1), points(:, 2), 'r--');
end

% simulation
wp_index = 1;
for i = 2:length(t)
    disp(' ');
    % % control heading
    delta_t = calculate_steering_angle(c_t, k, v_t, delta_max);

    % % update bicycle
    noise = [
        normrnd(0, stddev_x);
        normrnd(0, stddev_y);
        normrnd(0, stddev_theta);
    ];
    % x_t = bicycle_update(x_t, v_t, L, c_t(2), delta_t, dt) + noise;
    x_t = bicycle_update(x_t, v_t, L, c_t(2), delta_t, dt);
    x_store(:, i) = x_t;

    % update carrot
    [carrot_t, wp_index] = carrot_update(x_t, carrot_t, 0.2, waypoints, wp_index);
    carrot_store(:, i) = carrot_t;
    carrot_t

    % update controller
    %% adjust crosstrack error
    wp_start = waypoints(wp_index, :);
    wp_end = waypoints(wp_index + 1, :);
    c_t(1) = point_edge_dist(x_t, [wp_start, wp_end]);
    %% adjust heading error
    pt_on_line = closest_point(x_t, [wp_start, wp_end]);
    u = [carrot_t(1) - pt_on_line(1); carrot_t(2) - pt_on_line(2); 0];
    v = [carrot_t(1) - x_t(1); carrot_t(2) - x_t(2); 0];
    if (atan2(norm(cross(u,v)), dot(u,v)) < 0.0)
        disp('FLIP');
        c_t(2) = -atan2(norm(cross(u,v)), dot(u,v)) + 2 * pi;
    else
        c_t(2) = -atan2(norm(cross(u,v)), dot(u,v));
    end
    % c_t(2) = -atan2(norm(cross(u,v)), dot(u,v));
    % atan2(norm(cross(u,v)), dot(u,v))
    % rad2deg(c_t(2));

    c_t = controller_update(c_t, L, v_t, delta_t, dt);
end

% plot animation
plot_rectangle(1, wp_1, wp_2, wp_3, wp_4);
plot_trajectory(1, x_store, carrot_store, t);
pause

% distancePointEdge([0, 10], [wp_1, wp_2])
% perp_point_edge_dist([6, -16], [wp_1, wp_2])
% print -djpg -color bicycle_motion_20s.jpg
% pause
