% Plotting camera data for report. 
close all;
clear all;
load linear_data.mat;
load 'Dimond_Data.mat'

FPS_Plotting_x = True_y(6:end);

True_y = [True_y; DMocap_y];
True_x = [True_x; DMocap_x];
True_z = [True_z; DMocap_z];

Apr_x = [Apr_x(6:end); DApr_x(2:end)];
Apr_y = [Apr_y(6:end); DApr_y(2:end)];
Apr_z = [Apr_z(6:end); DApr_z(2:end)];

time = (Time - Time(1))/10^9;

dist = sqrt((-True_y(1:end) + 0.09).^2 + ...
            (True_x(1:end) + 0.03).^2 + ...
            (True_z(1:end) - 1.33).^2 );

measured_dist = sqrt((Apr_y).^2 + ...
                (Apr_x).^2 + ...
                (Apr_z).^2 );
            
figure(1) %error plot as funct distance
plot(dist, (dist-measured_dist+0.03), 'bo');
set(gca, 'XMinorTick', 'on', ...
          'YMinorTick', 'on', ...
          'FontWeight', 'bold', ...
          'Fontsize', 12);
xlabel('Euclidean Distance Between AprilTag and Camera [meters]',...
                                    'Fontsize', 15, ...
                                    'FontWeight', 'normal');
                                
ylabel('Measured Euclidean Distance [meters]',...
               'Fontsize', 15, ...
               'FontWeight', 'normal');
axis([0 3.5 -0.2 0.2]);

% figure(2) %ground truth
% error_x = (Apr_x(6:end) - True_y(1:end) + 0.09);
% error_y = (Apr_y(6:end) + True_x(1:end) + 0.02);
% error_z = (Apr_z(6:end) + True_z(1:end) - 1.33);
% plot3(Apr_x(6:end), Apr_y(6:end), Apr_z(6:end), '-bx', ...
%       True_y(1:end) + 0.09, True_x(1:end) + 0.02, -(True_z(1:end) - 1.33) , '--rd')
%  xlabel('x')
%  ylabel('y')
%  zlabel('z')


figure(3)
plot(FPS_Plotting_x, FrameRate(7:26), '-bo')
set(gca, 'XMinorTick', 'on', ...
          'YMinorTick', 'on', ...
          'FontWeight', 'bold', ...
          'Fontsize', 12);
        
xlabel('Euclidean Distance Between AprilTag and Camera [meters]',...
                                    'Fontsize', 15, ...
                                    'FontWeight', 'normal');
ylabel('Frame Rate [Hz]',...
               'Fontsize', 15, ...
               'FontWeight', 'normal');
axis([0 3.5 0 35]);


