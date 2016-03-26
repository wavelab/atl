% clear environment
clear;
clc;
close all;


% simulation parameters
t_end = 20;
dt = 0.1;  % 10 Hz
t = 0:dt:t_end;

% model parameters
L = 0.3;
stddev_x = 0.02;
stddev_y = 0.02;
stddev_theta = 1.0;

% wheel input commands
n = length(t);
v_t = 3;
x_t = [0; 0; 0];
x_store = zeros(3, length(t));


function r = deg2rad(degrees)
    r = degrees * pi / 180;
end

function x = g(x_prev, v_t, L, delta_t, dt)
    x = [
        x_prev(1) + v_t * cos(deg2rad(x_prev(3))) * dt;
        x_prev(2) + v_t * sin(deg2rad(x_prev(3))) * dt;
        x_prev(3) + (v_t * tan(deg2rad(delta_t)) / L) * dt;
    ];
end

function plot_animation(fig_index, x_store, t)
    % plot animation
    figure(fig_index);
    clf;

    for i = 1:2:length(t)
        plot(x_store(1, 1:i), x_store(2, 1:i), 'bx');
        axis equal;
        drawnow;
    end
end


% simulation
for i = 1:length(t)
    delta_t = 10 - t(i);
    additive_noise = [
        normrnd(0, stddev_x);
        normrnd(0, stddev_y);
        normrnd(0, deg2rad(stddev_theta));
    ];

    x_t = g(x_t, v_t, L, delta_t, dt) + additive_noise;
    x_store(:, i + 1) = x_t;
end

plot_animation(1, x_store, t);
print -djpg -color bicycle_motion_20s.jpg
