classdef OmniRobot
    % MODEL PROPERTIES
    properties (Constant)
        % length from wheel to center of chassis
        l = 0.3;
        
        % radius of wheel
        r = 0.25;

        % wheel constraints
        J_1 = [
            0, 1, OmniRobot.l;
            -cos(pi / 6), -sin(pi / 6), OmniRobot.l;
            cos(pi / 6), -sin(pi / 6), OmniRobot.l;
        ];
    
        % wheel radius
        J_2 = [
            OmniRobot.r, 0.0, 0.0;
            0.0, OmniRobot.r, 0.0;
            0.0, 0.0, OmniRobot.r;
        ];
    
        % motion disturbance
        R = [
            0.05^2 0 0; 
            0 0.05^2 0; 
            0 0 deg2rad(0.5)^2;
        ];
    
        % measurement disturbance
        Q = [
            0.5^2, 0, 0;
            0, 0.5^2, 0;
            0, 0, deg2rad(10^2);
        ];
    end
    
    % SIMULATION PROPERTIES
    properties (Constant)
        % discrete time step
        dt = 0.1;
        
        % end time (in seconds)
        Tf = 10;

        % initial State
        x_0 = [
            0; 
            0; 
            0;
        ];

        % prior
        mu_0 = [
            0; 
            0; 
            0;
        ];  % mean (mu)  
    end
   
    methods
        function [G] = G(this, theta, omega)
            % constant terms
            A = inv(this.J_1) * this.J_2 * omega;
            
            % differentiated rotation matrix w.r.t x_3 (i.e. theta)
            dRot = [
                -sin(theta), cos(theta), 0;
                -cos(theta), -sin(theta), 0;
                0, 0, 0; 
            ];
            dRot_A = dRot * A;

            % fill in the jacobian matrix G
            G = eye(3);
            G(1:2, 3) = dRot_A(1:2);
        end
        
        function x = g_dot(this, theta, omega)
            Rot = [
                cos(theta), -sin(theta), 0;
                sin(theta), cos(theta), 0; 
                0, 0, 1;
            ];
            x = inv(Rot) * inv(this.J_1) * this.J_2 * omega;
        end
        
        function simulateEKF(this)
            T = 0:this.dt:this.Tf;
            
            % state
            n = length(this.x_0);
            x = zeros(n, length(T));
            x(:,1) = this.x_0;
            
            % motion related
            mu = this.mu_0;
            S = 1 * eye(3);  % covariance (Sigma)
            [RE, Re] = eig(this.R);

            % measurement related
            m = length(this.Q(:, 1));
            y = zeros(m, length(T));
            [DE, De] = eig(this.Q);

            % inputs
            omega_1 = -15.5;
            omega_2 = 10.5;
            omega_3 = 1.5;
           
            % save states
            mup_S = zeros(n, length(T));
            mu_S = zeros(n, length(T));
            St = zeros(n,n,length(T));
          
            % create AVI thisect
            makemovie = 0;
            if(makemovie)
                vidthis = VideoWriter('ekf.avi');
                vidthis.Quality = 100;
                vidthis.FrameRate = 8;
                open(vidthis);
            end       
            
            for t = 2:length(T)
                % update state
                omega = [omega_1; omega_2; omega_3];
                e = RE * sqrt(Re) * randn(n,1);
                G = this.G(mu(3), omega);
                x(:,t) = x(:,t-1) + this.g(x(3, t - 1), omega) * this.dt + e;
                
                % update measurement
                d = DE * sqrt(De) * randn(m, 1);
                y(:,t) = x(:,t) + d + [0; 0; deg2rad(-9.7)];

                % EKF
                % prediction update
                mup = mu + this.g(mu(3), omega) * this.dt;
                Sp = G * S * G' + this.R;

                % measurement update
                Ht = eye(3);
                K = Sp * Ht' * inv(Ht * Sp * transpose(Ht) + this.Q);
                mu = mup + K * (y(:,t) - Ht * mup);
                S = (eye(n) - K * Ht) * Sp;
                

                % store results
                mup_S(:,t) = mup;
                mu_S(:,t) = mu;
                St(:,:,t) = S; 
                
                % plot results
%                 figure(1);
%                 clf; 
%                 hold on;
%                 pause(0.001);     
% 
%                 plot(x(1, 2:t), x(2, 2:t), 'ro--')
%                 plot(mu_S(1, 2:t), mu_S(2, 2:t), 'bx--')
%                 
%                 mu_pos = [mu(1) mu(2)];
%                 S_pos = [S(1,1) S(1,3); S(3,1) S(3,3)];
%                 error_ellipse(S_pos, mu_pos, 0.75);
%                 error_ellipse(S_pos, mu_pos, 0.95);

%                 title('True state and belief')
%                 axis equal
%                 axis([-1 8 -6 3])

                % write plot frame to movie thisect
                if (makemovie) 
                    writeVideo(vidthis, getframe(gca)); 
                end
            end
            plot_results = true;
            if plot_results == true
                this.plot_results(mu_S, x, T, St)
            end
            
        end
        
        function plot_results(~,mu_S, x, T, St)
            % plot the true state and belief for x, y, heading
            figure(1);
            clf; 
            
            subplot(4, 1, 1)
            plot(x(1,:), x(2,:), 'ro--', mu_S(1,:), mu_S(2,:), 'bx--')
            title('True state and belief (Top Down View)');
            xlabel('Displacement in x-direction (m)')
            ylabel('Displacement in y-direction (m)')
            axis equal
            axis([-1.5 10 -8 4])
            
            subplot(4, 1, 2)
            plot(T, x(1,:), 'ro--', T, mu_S(1,:), 'bx--');
            title('True state and belief (x-axis)');
            xlabel('Time (s)')
            ylabel('Displacement (m)')

            subplot(4, 1, 3)
            plot(T, x(2,:), 'ro--', T, mu_S(2,:), 'bx--');
            title('True state and belief (y-axis)');
            xlabel('Time (s)')
            ylabel('Displacement (m)')
            
            subplot(4, 1, 4)
            %heading_plot = mod(x(3,:),2*pi());
            %mu_S_heading_plot = mod(mu_S(3,:),2*pi());
            plot(T, rad2deg(x(3,:)), 'ro--', T, rad2deg(mu_S(3,:)), 'bx--');
            title('True state and belief (y-axis)');
            xlabel('Time (s)')
            ylabel('Heading (degrees)')
            
            %plot error elipse over time
            for t = 2:length(T)
                figure(2);
                %clf;
                hold on;
                pause(0.01);
                %mup = mup_S(:,t);
                mu = mu_S(:,t);
                S = St(:,:,t);
                

                plot(x(1, 2:t), x(2, 2:t), 'ro--')
                plot(mu_S(1, 2:t), mu_S(2, 2:t), 'bx--')

                mu_pos = [mu(1) mu(2)];
                S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
                error_ellipse(S_pos, mu_pos, 0.75);
                error_ellipse(S_pos, mu_pos, 0.95);

                title('True state and belief')
                axis equal
                %axis([-1 8 -6 3])
            end

            
            % close movie thisect
%            if (makemovie) 
%                close(vidthis); 
 %           end
            
        end
            
            
   end
   
end
