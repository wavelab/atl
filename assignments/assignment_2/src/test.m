source('util.m');
% v = [1; 0; 0];
% u = [0; 10; 0];
%
% y = norm(cross(u, v))
% x = dot(u, v)
% angle = rad2deg(atan2(y, x))


% x = 1;
% y = -1;
%

% function angle = atan3(y, x)
%     angle = atan2(y, x);
%
%     % if x <= 0 && y < 0
%     %     angle = angle + 2 * pi;
%     % elseif x > 0 && y < 0
%     %     angle = angle + 2 * pi;
%     % end
%     if y < 0
%         angle = angle + 2 * pi;
%     end
% end
% %
% % rad2deg(atan3(y, x))
%
% % wp_start = [0; 0; 0];
% % wp_end = [0; -1; 0];
% % wp_heading = wp_end - wp_start;
% % angle = rad2deg(atan3(wp_heading(2), wp_heading(1)))
%
%
% angle = rad2deg(atan3(0, 1))  % 0 degrees
% angle = rad2deg(atan3(1, 1))  % 45 degrees
% angle = rad2deg(atan3(1, 0))  % 90 degrees
% angle = rad2deg(atan3(0, -1))  % 180 degrees
% angle = rad2deg(atan3(-1, -1))  % 225 degrees
% angle = rad2deg(atan3(-1, 0))  % 270 degrees
% angle = rad2deg(atan3(-1, 1))  % 315 degrees

% function delta_t = calculate_delta(x_t, carrot_t, delta_max)
%     theta = x_t(3);
%     R = [
%         cos(theta), -sin(theta);
%         sin(theta), cos(theta);
%     ];
%     carrot_robot = R * carrot_t - x_t(1:2);
%
%     delta_t = atan2(carrot_robot(2), carrot_robot(1));
%     if (delta_t > delta_max)
%         delta_t = delta_max;
%     elseif (delta_t < -delta_max)
%         delta_t = -delta_max;
%     end
% end
%
% rad2deg(calculate_delta([0; 0; 0], [1; 1], deg2rad(30)))
% rad2deg(calculate_delta([0; 0; 0], [-1; 1], deg2rad(30)))
% rad2deg(calculate_delta([0; 0; 0], [1; -1], deg2rad(30)))
% rad2deg(calculate_delta([0; 0; 0], [-1; -1], deg2rad(30)))


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

map_min = [0, 0];
map_max = [100, 100];
nb_samples = 10;
map_range = map_max - map_min;
samples_x = int32(map_range(1) * rand(nb_samples, 1) + map_min(1));
samples_y = int32(map_range(2) * rand(nb_samples, 1) + map_min(2));
samples = [samples_x, samples_y];

figure(1);
hold on;
plot(samples(:, 1), samples(:, 2), "ro")
drawnow;

n1 = samples(1, :)
n2 = samples(2, :)

bline = bresenham_line_setup(n1(1), n1(2), n2(1), n2(2))
ok = 1;
x_max = 100;
y_max = 100;

while ok
    [bline, ok] = bresenham_line_step(bline);

    if bline.x0 > x_max || bline.x0 < 0
        collide = 0;
        return;

    elseif bline.y0 > y_max || bline.y0 < 0
        collide = 0;
        return;
    end

    plot(bline.x0, bline.y0, 'r.-');
    drawnow;
end

pause;
