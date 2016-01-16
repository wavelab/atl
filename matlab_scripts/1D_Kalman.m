clear all
close all

t = 50;

z = zeros(t,1);
z_hat = zeros(t,1);
m = zeros(t,1);
P = zeros(t,1);

B = [1];
C = [1];
R  = [2];
P(1,1) = [1];
z(1,1) = 0.5*randn(1,1);


alpha = 0.9;
A = alpha;
z_hat_a = z_hat;
z_a = z;
P_a = P;
%part (a)
for i = 2:t
    z_a(i,1) = alpha*z_a(i-1,1)  + randn();
    m(i,1) = z_a(i,1)+2*randn();   
    
    %Kalman(A,B,m,C,R,P_0,z_0)
    [z_hat_a(i,1),P_a(i,1)] = Kalman_Scaler_Case(A,B,m(i,1),C,R,P_a(i-1,1),z_hat(i-1,1));
    
    
end

upperbound_a = z_hat_a + sqrt(P_a);
lowerbound_a = z_hat_a - sqrt(P_a);

figure('name','alpha = 0.9')
plot(1:t,z_a,1:t, z_hat_a,...
         1:t,lowerbound_a,'--k',...
         1:t,m,'*r',...
         1:t,upperbound_a,'--k',...          
         'LineWidth',2,'MarkerSize',10)
%         1:100,upperbound_z2, '--k',...
%         [10, 30], m1, 'kx',...
%         1:100,lowerbound_z2, '--k',...
%         'LineWidth',2,'MarkerSize',10)    
xlabel('Time t','FontSize',15,'fontweight','demi')
ylabel('Measured and Estimated Values','FontSize',15, 'fontweight','demi' )
leg = legend({'z(t)','Kalman Filter Estimate, $\hat{z}(t)$','$\pm \sqrt{P(t|t)}$', 'Measurements, m(t)'},'Interpreter','Latex');
set(leg, 'fontsize', 15,'fontweight','demi')
set(gca,'fontsize',12,'fontweight','demi')


