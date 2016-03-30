source('src/util.m');
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

function delta_t = calculate_delta(x_t, carrot_t, delta_max)
    theta = x_t(3);
    R = [
        cos(theta), -sin(theta);
        sin(theta), cos(theta);
    ];
    carrot_robot = R * carrot_t - x_t(1:2);

    delta_t = atan2(carrot_robot(2), carrot_robot(1));
    if (delta_t > delta_max)
        delta_t = delta_max;
    elseif (delta_t < -delta_max)
        delta_t = -delta_max;
    end
end

rad2deg(calculate_delta([0; 0; 0], [1; 1], deg2rad(30)))
rad2deg(calculate_delta([0; 0; 0], [-1; 1], deg2rad(30)))
rad2deg(calculate_delta([0; 0; 0], [1; -1], deg2rad(30)))
rad2deg(calculate_delta([0; 0; 0], [-1; -1], deg2rad(30)))
