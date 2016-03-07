#!/usr/bin/octave

phi = 30;
theta = 20;

rot = [
    1, 0, 0;
    0, cosd(phi), -1 * sind(phi);
    -1 * sind(theta), cosd(theta) * sind(phi), cosd(theta) * cosd(phi);
];

velocity = [1; 1; 1;];

rot * velocity

