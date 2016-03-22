% clear environment
clear;
clc;
close all;


% simulation parameters
t_end = 60;
dt = 0.1;  % 10 Hz
t = 0:dt:t_end;
x_t = [
    0;  % x
    10;  % y
    -10;  % steering angle
    0;  % crosstrack error
    0  % vehicle heading
];
x_store = zeros(5, length(t));

% model parameters
L = 0.3;
stddev_x = 0.02;
stddev_y = 0.02;
stddev_theta = 1.0;

% input commands
v_t = 3;

% control parameters
k = 2.0;


function r = deg2rad(degrees)
    r = degrees * pi / 180;
end

function x = g(x_prev, v_t, L, delta_t, dt)
    x = [
        x_prev(1) + v_t * cos(deg2rad(x_prev(3) - delta_t)) * dt;
        x_prev(2) + v_t * sin(deg2rad(x_prev(3) - delta_t)) * dt;
        x_prev(3) + (v_t * tan(deg2rad(delta_t)) / L) * dt;
        x_prev(2);
        x_prev(5) + ((-v_t * sin(delta_t)) / L) * dt
    ];
    x
end

% simulation
for i = 1:length(t)
    % control heading
    delta_t = x_t(5) + atan2(k * x_t(4), v_t);

    % motion update
    % noise = [
    %     normrnd(0, stddev_x);
    %     normrnd(0, stddev_y);
    %     normrnd(0, deg2rad(stddev_theta));
    %     0;
    %     0
    % ];
    % x_t = g(x_t, v_t, L, delta_t, dt) + noise;
    x_t = g(x_t, v_t, L, delta_t, dt);
    x_store(:, i + 1) = x_t;
end


% plot animation
figure(1);
clf;

for i = 1:2:length(t)
    plot(x_store(1, 1:i), x_store(2, 1:i), 'bx');
    axis equal;
    drawnow;
end
% print -djpg -color bicycle_motion_20s.jpg
% pause
