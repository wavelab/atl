
clear all
load SV_Pos                         %position of satellites
load SV_Rho                         %pseudorange of satellites
syms x Vx y Vy z Vz                 %position and velocity of the receiver in 3 dimensions,
syms b d                            %clock bias(b),clock drift(d)
Xstate = [x Vx y Vy z Vz b d].';    %state vector

T = 1; %positioning interval
N = 25;%total steps
% Set f
f = [x+T*Vx;
     Vx;
     y+T*Vy;
     Vy;
     z+T*Vz;
     Vz;
     b+T*d
     d];
% Set Q
Sf = 36;Sg = 0.01;sigma=5;         %state transition variance
Qb = [Sf*T+Sg*T*T*T/3 Sg*T*T/2;
	  Sg*T*T/2 Sg*T];
Qxyz = sigma^2 * [T^3/3 T^2/2;
                  T^2/2 T];
Q=blkdiag(Qxyz,Qxyz,Qxyz,Qb);

% Set initial value of X and P     
X = zeros(8,1);
X([1 3 5]) = [-2.168816181271560e+006 
                    4.386648549091666e+006 
                        4.077161596428751e+006];                 %Initial position
X([2 4 6]) = [0 0 0];                                            %Initial velocity
X(7,1) = 3.575261153706439e+006;                                 %Initial clock bias
X(8,1) = 4.549246345845814e+001;                                 %Initial clock drift
P = eye(8)*10;

fprintf('GPS positioning using EKF started\n') 
tic

for ii = 1:N
    % Set g
    syms xs ys zs                                                % symbols for position of satellites                                          
    g_func = sqrt((x-xs)^2+(y-ys)^2+(z-zs)^2) + b;               % pseudorange equation
    g = [];
    for jj=1:4                                                   % pseudorange equations for each satellites
        g = [g ; 
            subs(g_func,[xs ys zs],SV_Pos{ii}(jj,:))]; 
    end

    % Set R
    Rhoerror = 36;                                               % variance of measurement error(pseudorange error)
    R=eye(4) * Rhoerror; 

    % Set Z
    Z = SV_Rho{ii}.';                                            % measurement value

    [X,P] = Extended_KF(f,g,Q,R,Z,X,P,Xstate);
    Pos_KF(:,ii) = X([1 3 5]).';                                 % positioning using Kalman Filter
    Pos_LS(:,ii) = Rcv_Pos_Compute(SV_Pos{ii},SV_Rho{ii});       % positioning using Least Square as a contrast
    
    fprintf('KF time point %d in %d  ',ii,N)
    time = toc;
    remaintime = time * N / ii - time;
    fprintf('Time elapsed: %f seconds, Time remaining: %f seconds\n',time,remaintime)
end

for ii = 1:3
    subplot(3,1,ii)
    plot(1:N,Pos_KF(ii,:)-mean(Pos_KF(ii,:)),'-r')
    hold on
    plot(1:N,Pos_LS(ii,:)-mean(Pos_KF(ii,:)))
    legend('EKF','ILS')
    xlabel('Sampling index')
    ylabel('Position error(meters)')
end
	 