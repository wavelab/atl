function new_carrot = calculate_new_carrot(x_t, r, wp_start, wp_end)
    % cross-track point on trajectory
    pt_on_line = closest_point(x_t, [wp_start, wp_end]);

    % calculate unit vector of trajectory
    % allows us to traverse distance r along the trajectory
    v = double(wp_end - wp_start);
    u = v / norm(v);

    % update carrot position
    new_carrot = pt_on_line + r * u';
    new_carrot = [new_carrot(1); new_carrot(2)];
end

function [carrot_t, wp_index] = carrot_update(x_t, carrot_prev, r, waypoints, wp_index)
    wp_start = waypoints(wp_index, :);
    wp_end = waypoints(wp_index + 1, :);
    dist_from_wp_start = dist_between_points(transpose(wp_start), x_t(1:2));

    % calculate new carrot
    carrot_t = calculate_new_carrot(x_t, r, wp_start, wp_end);

    % update waypoint if reached waypoint
    if waypoint_reached(x_t(1:2), transpose(wp_end), 0.2) == 1
    % if waypoint_reached(carrot_t(1:2), transpose(wp_end), 0.2) == 1
        % update waypoint index
        wp_index += 1;
        if wp_index == length(waypoints)
            wp_index = 1;
        end

        % recalculate carrot
        new_wp_start = waypoints(wp_index, :);
        new_wp_end = waypoints(wp_index + 1, :);
        carrot_t = calculate_new_carrot(x_t, r, new_wp_start, new_wp_end);
    end

    % % limit carrot x component
    % if carrot_t(1) < min(wp_start(1), wp_end(1))
    %     carrot_t(1) = min(wp_start(1), wp_end(1));
    % elseif carrot_t(1) > max(wp_start(1), wp_end(1))
    %     carrot_t(1) = max(wp_start(1), wp_end(1));
    % end
    %
    % % limit carrot y component
    % if carrot_t(2) < min(wp_start(2), wp_end(2))
    %     carrot_t(2) = min(wp_start(2), wp_end(2));
    % elseif carrot_t(2) > max(wp_start(2), wp_end(2))
    %     carrot_t(2) = max(wp_start(2), wp_end(2));
    % end
end

function delta_t = calculate_delta(x_t, carrot_t, delta_max)
    theta = x_t(3);

    % calculate angle between carrot and bicycle
    x = (carrot_t(1) - x_t(1));
    y = (carrot_t(2) - x_t(2));
    angle_of_vec = atan3(y, x);  % returns only +ve angle

    % limit delta_t to pi and -pi only
    delta_t = -(theta - angle_of_vec);
    delta_t = mod(delta_t + pi, 2 * pi) - pi;

    % limit delta_t to steering angle max
    if (delta_t > delta_max)
        delta_t = delta_max;
    elseif (delta_t < -delta_max)
        delta_t = -delta_max;
    end
end

function x = bicycle_update(x_prev, v_t, L, delta_t, dt)
    x = [
        x_prev(1) + v_t * cos(x_prev(3)) * dt;
        x_prev(2) + v_t * sin(x_prev(3)) * dt;
        x_prev(3) + ((v_t * tan(delta_t)) / L) * dt;
    ];
end

% simulation
wp_index = 1;
delta_t = 0;

for i = 1:length(t)
    % update carrot
    [carrot_t, wp_index] = carrot_update(x_t, carrot_t, 2, waypoints, wp_index);
    carrot_store(:, i) = carrot_t;

    % update steering
    delta_t = calculate_delta(x_t, carrot_t, delta_max);

    % update bicycle
    x_t = bicycle_update(x_t, v_t, L, delta_t, dt);
    x_store(:, i + 1) = x_t;
end