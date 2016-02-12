% clear environment
clear;
clc;

% create AVI object
makemovie = 1;
if(makemovie)
    vidObj = VideoWriter('ekf.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
end

% model parameters
r = 0.25;
l = 0.30;
omega_1 = -15.5;
omega_2 = 10.5;
omega_3 = 1.5;
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

% discrete time step
dt = 0.1;

% initial State
x0 = [0 0 0]';

% prior
mu = [0 0 0]'; % mean (mu)
S = 1 * eye(3);% covariance (Sigma)

% motion model
R = [
    0 0 0; 
    0 0 0; 
    0 0 0;
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
n = 3;
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
    G = Gt(mu(3), omega_1, omega_2, omega_3);
    
    Rot = [
        cos(x(3,t-1)), -sin(x(3,t-1)), 0;
        sin(x(3,t-1)), cos(x(3,t-1)), 0; 
        0, 0, 1;
    ];
    omega = [omega_1; omega_2; omega_3];
    g = inv(Rot) * inv(J_1) * J_2 * omega;
    x(:,t) = x(:,t-1) + g * dt + e;
    
    % update measurement
    d = sqrt(Q) * randn(m,1);
    y(:,t) = x(:,t) + d;

    % EKF
    % prediction update
    mup = x(:,t);
    Sp = G * S * G' + R;

    % measurement update
    Ht = eye(3);
    K = Sp * Ht' * inv(Ht * Sp * Ht'+Q);
    mu = mup + K * (y(:,t) - Ht*mup );
    
    S = (eye(n) - K * Ht) * Sp;

    % store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    %K_S(:,t) = K;

    % plot results
    figure(1);
    clf; 
    hold on;
    pause(0.001);     
    
    plot(0,0,'bx', 'MarkerSize', 6, 'LineWidth', 2)
    plot(x(1,2:t),x(2,2:t), 'ro--')
    plot(mu_S(1,2:t),mu_S(2,2:t), 'bx--')
    mu_pos = [mu(1) mu(2)];
    S_pos = [S(1,1) S(1,3); S(3,1) S(3,3)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    title('True state and belief')
    axis equal
    axis([-1 8 -6 3])
    if (makemovie) writeVideo(vidObj, getframe(gca)); end

end
if (makemovie) close(vidObj); end
