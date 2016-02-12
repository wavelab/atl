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


function r = deg2rad(degrees)
    r = degrees * pi / 180;
end

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



% simulate robot
function x_t = simulate(J_1, J_2, omega_inputs, t, dt)
    x_t = [ 0; 0; 0 ];
    R = [
        0.05^2, 0, 0;
        0, 0.05^2, 0;
        0, 0, deg2rad(0.5);
    ];
    [RE, Re] = eig(R);

    for i = 1:length(t)
        theta = x_t(3, i);
        R = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1;];
        omega = [omega_inputs(1); omega_inputs(2); omega_inputs(3);];

        e = RE * Re * randn(3, 1);

        x_dt = inv(R) * inv(J_1) * J_2 * omega;
        x_t(:,i + 1) = x_t(:,i) + x_dt * dt + e;
    end
endfunction

% plot animation
function plot_animation(t, x_t)
    figure(1);
    clf;
    axis_limits = [ -5 5 -5 5 ];  % x and y-axis limits

    for i = 1:2:length(t)
        plot(x_t(1, 1:i), x_t(2, 1:i), 'bx');
        axis equal;
        % axis(axis_limits);
        drawnow;
    end
endfunction



% apply pre-defined rotation inputs
% n = length(t);
% omega_1 = -15.5;
% omega_2 = 10.5;
% omega_3 = 1.5;
% omega_inputs = [
%     omega_1 * ones(1, n);
%     omega_2 * ones(1, n);
%     omega_3 * ones(1, n);
% ];
% x_t = simulate(J_1, J_2, omega_inputs, t, dt);
% plot_animation(t, x_t)
% print -djpg -color traverse_predefined.jpg




% % traverse east
% n = length(t);
% omega_1 = 0;
% omega_2 = -1;
% omega_3 = 1;
% omega_inputs = [
%     omega_1 * ones(1, n);
%     omega_2 * ones(1, n);
%     omega_3 * ones(1, n);
% ];
% x_t = simulate(J_1, J_2, omega_inputs, t, dt);
% plot_animation(t, x_t)
% print -djpg -color traverse_east_with_noise.jpg
%
% % traverse west
% n = length(t);
% omega_1 = 0;
% omega_2 = 1;
% omega_3 = -1;
% omega_inputs = [
%     omega_1 * ones(1, n);
%     omega_2 * ones(1, n);
%     omega_3 * ones(1, n);
% ];
% x_t = simulate(J_1, J_2, omega_inputs, t, dt);
% plot_animation(t, x_t)
% print -djpg -color traverse_west_with_noise.jpg
%
% % traverse north east
% n = length(t);
% omega_1 = 1;
% omega_2 = -1;
% omega_3 = 0;
% omega_inputs = [
%     omega_1 * ones(1, n);
%     omega_2 * ones(1, n);
%     omega_3 * ones(1, n);
% ];
% x_t = simulate(J_1, J_2, omega_inputs, t, dt);
% plot_animation(t, x_t)
% print -djpg -color traverse_north_east_with_noise.jpg
%
% % traverse south east
% n = length(t);
% omega_1 = -1;
% omega_2 = 0;
% omega_3 = 1;
% omega_inputs = [
%     omega_1 * ones(1, n);
%     omega_2 * ones(1, n);
%     omega_3 * ones(1, n);
% ];
% x_t = simulate(J_1, J_2, omega_inputs, t, dt);
% plot_animation(t, x_t)
% print -djpg -color traverse_south_east_with_noise.jpg
%
% % traverse in-place
% n = length(t);
% omega_1 = 1;
% omega_2 = 1;
% omega_3 = 1;
% omega_inputs = [
%     omega_1 * ones(1, n);
%     omega_2 * ones(1, n);
%     omega_3 * ones(1, n);
% ];
% x_t = simulate(J_1, J_2, omega_inputs, t, dt);
% plot_animation(t, x_t)
% print -djpg -color traverse_inplace_with_noise.jpg




% circle
% n = length(t);
% theta_0 = 0;
% circle_radius = 1;
% R = [
%     cos(theta_0), -sin(theta_0), 0;
%     sin(theta_0), cos(theta_0), 0;
%     0, 0, 1;
% ];
% x_inertia = [
%     2 * pi * circle_radius / t_end,
%     0,
%     2 * pi / t_end
% ];
% calculated_omega= R * J_1 * inv(J_2) * x_inertia;
% omega_inputs = [
%     calculated_omega(1) * ones(1, n);
%     calculated_omega(2) * ones(1, n);
%     calculated_omega(3) * ones(1, n);
% ];
% x_t = simulate(J_1, J_2, omega_inputs, t, dt);
% plot_animation(t, x_t)
% print -djpg -color circle_with_noise.jpg





% expanding spiral of any size
t_end = 50;
dt = 0.01;  % 10 Hz
t = 0:dt:t_end;
n = length(t);
theta_0 = 0;
circle_radius = 1;
R = [
    0.05^2, 0, 0;
    0, 0.05^2, 0;
    0, 0, deg2rad(0.5);
];
[RE, Re] = eig(R);
R = [
    cos(theta_0), -sin(theta_0), 0;
    sin(theta_0), cos(theta_0), 0;
    0, 0, 1;
];
x_inertia = [
    2 * pi * circle_radius / t_end,
    0,
    2 * pi
];
x_t = [ 0; 0; 0 ];

z = 10;
for i = 1:length(t)
    theta = x_t(3, i);
    R = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1;];
    omega = [
        z;
        -2;
        2;
    ];
    z = 0.999 * z;

    e = RE * Re * randn(3, 1);
    x_dt = inv(R) * inv(J_1) * J_2 * omega;
    x_t(:,i + 1) = x_t(:,i) + x_dt * dt + e;
end
plot_animation(t, x_t)
print -djpg -color traverse_spiral_with_noise.jpg


