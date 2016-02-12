function [ G ] = Gt(theta,omega_1, omega_2, omega_3)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
l = 0.3;
r = 0.25;

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
omega = [omega_1; omega_2; omega_3];
A = inv(J_1)*J_2*omega;
    
    dRot = [-sin(theta),  cos(theta),  0;
          -cos(theta),  -sin(theta), 0;
          0,              0              0; ];
    
    dRot_A = dRot*A;
   
        
    G = eye(3);
    G(1:2, 3) = dRot_A( 1:2 );
    
    
end

