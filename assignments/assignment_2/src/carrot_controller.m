source('src/drawbox.m');
source('src/util.m');


% simulation parameters
t_end = 1;
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
    24;  % x
    10;  % y
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
c_store(:, 1) = c_t;


function delta_t = calculate_steering_angle(c_t, k, v_t, delta_max)
   delta_t = c_t(2) + atan2(k * c_t(1), v_t);

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
        psi_t - delta_t;
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

function dist = dist_between_points(x, y)
    diff = x - y;
    dist = norm(diff, 2);
end

function [carrot_t, wp_index] = carrot_update(x_t, carrot_prev, r, waypoints, wp_index)
    wp_start = waypoints(wp_index, :);
    wp_end = waypoints(wp_index + 1, :);
    dist_from_wp_start = dist_between_points(wp_start', x_t(1:2));

    % calculate new carrot
    carrot_t = calculate_new_carrot(x_t, r, wp_start, wp_end);

    % check if carrot has reached waypoint
    wp_reached = waypoint_reached(carrot_t, wp_start, wp_end, r);
    if wp_reached == 1
        disp('new waypoint');

        % update waypoint index
        wp_index += 1;
        if wp_index == length(waypoints)
            wp_index = 1;
        end

        % recalculate carrot
        new_wp_start = waypoints(wp_index, :);
        new_wp_end = waypoints(wp_index + 1, :);
        diff = transpose(new_wp_start) - x_t(1:2);
        adj_r = r - norm(diff, 2);
        adj_r
        carrot_t = calculate_new_carrot(x_t, adj_r, new_wp_start, new_wp_end);

    % adjust carrot if cornering
    elseif dist_from_wp_start <= r
        adj_r = r - dist_from_wp_start;
        adj_r
        carrot_t = calculate_new_carrot(x_t, adj_r, wp_start, wp_end);

    end

    % limit carrot x component
    if carrot_t(1) < min(wp_start(1), wp_end(1))
        carrot_t(1) = min(wp_start(1), wp_end(1));
    elseif carrot_t(1) > max(wp_start(1), wp_end(1))
        carrot_t(1) = max(wp_start(1), wp_end(1));
    end

    % limit carrot y component
    if carrot_t(2) < min(wp_start(2), wp_end(2))
        carrot_t(2) = min(wp_start(2), wp_end(2));
    elseif carrot_t(2) > max(wp_start(2), wp_end(2))
        carrot_t(2) = max(wp_start(2), wp_end(2));
    end
    carrot_t
end

function c_t = controller_update(c_prev, L, v_t, delta_t, dt)
    % controller state derivatives
    c_deriv = [
        v_t * sin(c_prev(2) - delta_t);
        (-v_t * sin(delta_t)) / L;
    ];

    % controller state update
    c_t = [
        c_prev(1) + c_deriv(1) * dt;
        c_prev(2) + c_deriv(2) * dt;
    ];
end

% simulation
wp_index = 1;
for i = 2:length(t)
    % control heading
    delta_t = calculate_steering_angle(c_t, k, v_t, delta_max);

    % update bicycle
    noise = [
        normrnd(0, stddev_x);
        normrnd(0, stddev_y);
        normrnd(0, stddev_theta);
    ];
    % x_t = bicycle_update(x_t, v_t, L, c_t(2), delta_t, dt) + noise;
    x_t = bicycle_update(x_t, v_t, L, c_t(2), delta_t, dt);
    x_store(:, i) = x_t;

    % update carrot
    [carrot_t, wp_index] = carrot_update(x_t, carrot_t, 1, waypoints, wp_index);
    carrot_store(:, i) = carrot_t;

    % update controller
    wp_start = waypoints(wp_index, :);
    wp_end = waypoints(wp_index + 1, :);
    %% adjust crosstrack error
    c_t(1) = point_edge_dist(x_t, [wp_start, wp_end]);
    %% adjust heading error
    pt_on_line = closest_point(x_t, [wp_start, wp_end]);
    v = [
        carrot_t(1) - pt_on_line(1);
        carrot_t(2) - pt_on_line(2);
        0
    ];
    u = [
        carrot_t(1) - x_t(1);
        carrot_t(2) - x_t(2);
        0
    ];
    c_t(2) = -atan3(norm(cross(u,v)), dot(u,v));
    % c_t = controller_update(c_t, L, v_t, delta_t, dt);
    c_store(:, i) = c_t;
end

% plot animation
plot_waypoints(1, waypoints);
plot_trajectory(1, x_store, carrot_store, t);
plot_controller(1, c_store, t);
pause
