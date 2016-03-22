% clear environment
clear;
clc;
close all;


% simulation parameters
t_end = 15;
dt = 0.01;  % 10 Hz
t = 0:dt:t_end;

% model parameters
r = 0.25;
l = 0.30;
omega_1 = -15.5;
omega_2 = 10.5;
omega_3 = 1.5;
stddev_x = 0.05;
stddev_y = 0.05;
stddev_theta = 0.05;

% wheel input commands
n = length(t);

% J_1, J_2 and x_t (state)
J_1 = [
    0, 1, l;
    -cos(pi / 6), -sin(pi / 6), l;
    cos(pi / 6), -sin(pi / 6), l;
];
J_2 = [
    r, 0.0, 0.0;
    0.0, r, 0.0;
    0.0, 0.0, r;
];
x_t = [
    0;
    0;
    0
];

% simulation
for i = 1:length(t)
    theta = x_t(3, i);
    R = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1;];
    omega = [omega_1; omega_2; omega_3];
    additive_noise = [
        normrnd(0, stddev_x);
        normrnd(0, stddev_y);
        normrnd(0, stddev_theta);
    ];

    x_dt = inv(R) * inv(J_1) * J_2 * omega;
    x_t(:,i + 1) = x_t(:,i) + x_dt * dt;
end


% plot animation
figure(1);
clf;
axis_limits = [ -5 5 -5 5 ];  % x and y-axis limits

for i = 1:2:length(t)
    plot(x_t(1, 1:i), x_t(2, 1:i), 'bx');
    axis equal;
    % axis(axis_limits);
    drawnow;
end
