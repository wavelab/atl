function [ X_dot ] = Gt(theta,omega_1, omega_2, omega_3)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
l = 0.3;
r = 0.25;
stddev_x = 0.05;
stddev_y = 0.05;
stddev_theta = 0.05;

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
    
    R = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1;];
    omega = [omega_1; omega_2; omega_3];
    additive_noise = [
        normrnd(0, stddev_x);
        normrnd(0, stddev_y);
        normrnd(0, stddev_theta);
    ];
    X_dot = inv(R) * inv(J_1) * J_2 * omega;    

end

