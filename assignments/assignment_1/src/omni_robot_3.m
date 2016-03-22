% clear environment
clear;
clc;
close all;

% creating the motion model
syms theta r w_1 w_2 w_3 x_1 x_2 x_3 l

R = [
    cos(theta), -sin(theta), 0;
    sin(theta), cos(theta), 0;
    0, 0, 1;
];

J_1 = [
    0, 1, l;
    -cos(pi / 6), -sin(pi / 6), l;
    cos(pi / 6), -sin(pi / 6), l;
];

v = [
    x_1;
    x_2;
    x_3
];

g = inv(R) * inv(J_1) * v;
simplify(g)
