% clear environment
clear;
clc;

% model parameters
r = 0.25;
l = 0.30;
omega_1 = -15.5;
omega_2 = 10.5;
omega_3 = 1.5;

% syms theta r w_1 w_2 w_3 x_1 x_2 x_3;
% R = [
%     cos(theta), -sin(theta), 0;
%     sin(theta), cos(theta), 0;
%     0, 0, 1;
% ];
% J_1 = [
%     0, 1, l;
%     -cos(pi / 6), -sin(pi / 6), l;
%     cos(pi / 6), -sin(pi / 6), l;
% ];
% v = [
%     x_1;
%     x_2;
%     x_3
% ];
% g = inv(R) * inv(J_1) * v;


% video object
% makemovie = 1;
% if(makemovie)
%     vidObj = VideoWriter('ekf.avi');
%     vidObj.Quality = 100;
%     vidObj.FrameRate = 8;
%     open(vidObj);
% end

% discrete time step
dt = 0.1;

% initial State
x0 = [0 0 0]';

% prior
mu = [0 0 0]'; % mean (mu)
S = 1 * eye(3);% covariance (Sigma)

% motion model
R = [
    0.1 0 0; 
    0 0.1 0; 
    0 0 0.1
];
[RE, Re] = eig(R);

% measurement model
Q = [
    0.5, 0, 0;
    0, 0.5, 0;
    0, 0, deg2rad(10);
];

% Simulation Initializations
Tf = 10;
T = 0:dt:Tf;
n = length(Ad(1,:));
x = zeros(n,length(T));
x(:,1) = x0;
m = length(Q(:,1));
y = zeros(m,length(T));
mup_S = zeros(n,length(T));
mu_S = zeros(n,length(T));



%% plot results
figure(1);
clf; 
hold on;

%% main loop
for t = 2:length(T)
    % update state
    e = RE * sqrt(Re) * randn(n,1);
    x_1 = mu(1);
    x_2 = mu(2);
    x_3 = mu(3);
    g = [
        (2*x_1*sin(theta))/3 - (x_2*sin(theta))/3 - (x_3*sin(theta))/3 - (3^(1/2)*x_2*cos(theta))/3 + (3^(1/2)*x_3*cos(theta))/3;
        (2*x_1*cos(theta))/3 - (x_2*cos(theta))/3 - (x_3*cos(theta))/3 + (3^(1/2)*x_2*sin(theta))/3 - (3^(1/2)*x_3*sin(theta))/3;
        (x_1 + x_2 + x_3)/(3*l)
    ];
    G = [
        2/3 * sin(theta), 2/3 * cos(theta), 1/(3 * l);
        -cos(theta)/sqrt(3) - cos(theta)/3, -sin(theta)/sqrt(3) - cos(theta)/3, 1/(3 * l);
        cos(theta)/sqrt(3) - sin(theta)/3, -sin(theta)/sqrt(3) - cos(theta)/3, 1/(3 * l);
    ];
    x(:,t) = g + G + e;

    % update measurement
    d = sqrt(Q) * randn(m,1);
    y(:,t) = sqrt(x(1,t)^2 + x(3,t)^2) + d;

    % EKF
    % prediction update
    mup = Ad * mu;
    Sp = Ad * S * Ad' + R;

    % measurement update
    Ht = [1 1 1];
    K = Sp * Ht' * inv(Ht * Sp * Ht'+Q);
    mu = mup + K * (y(:,t)-sqrt(mup(1)^2 + mup(3)^2));
    S = (eye(n) - K * Ht) * Sp;

    % store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    K_S(:,t) = K;

    % Plot results
    figure(1);
    clf; 
    hold on;
    pause(0.001);     
    plot(0,0,'bx', 'MarkerSize', 6, 'LineWidth', 2)
    plot([20 -1],[0 0],'b--')
    plot(x(1,2:t),x(3,2:t), 'ro--')
    plot(mu_S(1,2:t),mu_S(3,2:t), 'bx--')
    mu_pos = [mu(1) mu(3)];
    S_pos = [S(1,1) S(1,3); S(3,1) S(3,3)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    title('True state and belief')
    axis equal
    axis([-1 20 -10 10])
    % if (makemovie) writeVideo(vidObj, getframe(gca)); end

end
% if (makemovie) close(vidObj); end
