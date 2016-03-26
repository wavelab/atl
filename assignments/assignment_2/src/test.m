% v = [0; 1; 0];
% u = [1; 1; 0];
%
% y = norm(cross(u, v))
% x = dot(u, v)
% angle = atan2(y, x);

x = 1;
y = -1;


function angle = atan3(y, x)
    angle = atan2(y, x);

    if x < 0 && y < 0
        angle = angle + 2 * pi;
    elseif x > 0 && y < 0
        angle = 2 * pi + angle;
    end
end

rad2deg(atan3(y, x))
