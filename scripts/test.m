#!/usr/bin/octave
%
% phi = 30;
% theta = 20;
%
% rot = [
%     1, 0, 0;
%     0, cosd(phi), -1 * sind(phi);
%     -1 * sind(theta), cosd(theta) * sind(phi), cosd(theta) * cosd(phi);
% ];
%
% velocity = [1; 1; 1;];
%
% rot * velocity



% rotation matrix - RzRyRx
psi = 1.0;
phi = 1.0;
theta = 1.0;

r1 = cos(phi) * cos(theta);
r4 = sin(phi) * cos(theta);
r7 = -1 * sin(theta);

r2 = cos(phi) * sin(theta) * sin(psi) - cos(psi) * sin(phi);
r5 = cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi);
r8 = cos(theta) * sin(psi);

r3 = sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta);
r6 = cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi);
r9 = cos(theta) * cos(psi);

rot_m = [r1, r2, r3; r4, r5, r6; r7, r8, r9]
