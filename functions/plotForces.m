function plotForces(t,f)
% plotMotions.m     e.anderlini@ucl.ac.uk     15/09/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is used to plot the thrust, restoring and damping forces of
% the ROV.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Forces:
figure;
subplot(3,1,1);
plot(t,f(:,1),'--');
hold on;
plot(t,f(:,7),'-.');
hold on;
plot(t,f(:,13),'-');
hold off;
ylabel('$f_x$ (N)','Interpreter','Latex');
l = legend('thrust','restoring','damping','Location','Best');
set(l,'Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex');
subplot(3,1,2);
plot(t,f(:,2),'--');
hold on;
plot(t,f(:,3),'-.');
hold on;
plot(t,f(:,14),'-');
hold off;
ylabel('$f_y$ (N)','Interpreter','Latex');
l = legend('thrust','restoring','damping','Location','Best');
set(l,'Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex');
subplot(3,1,3);
plot(t,f(:,3),'--');
hold on;
plot(t,f(:,9),'-.');
hold on;
plot(t,f(:,15),'-');
hold off;
xlabel('Time (s)','Interpreter','Latex');
ylabel('$f_z$ (N)','Interpreter','Latex');
l = legend('thrust','restoring','damping','Location','Best');
set(l,'Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex');

%% Moments:
figure;
subplot(3,1,1);
plot(t,f(:,4),'--');
hold on;
plot(t,f(:,10),'-.');
hold on;
plot(t,f(:,16),'-');
hold off;
ylabel('$m_\phi$ (Nm)','Interpreter','Latex');
l = legend('thrust','restoring','damping','Location','Best');
set(l,'Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex');
subplot(3,1,2);
plot(t,f(:,5),'--');
hold on;
plot(t,f(:,11),'-.');
hold on;
plot(t,f(:,17),'-');
hold off;
ylabel('$m_\theta$ (Nm)','Interpreter','Latex');
l = legend('thrust','restoring','damping','Location','Best');
set(l,'Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex');
subplot(3,1,3);
plot(t,f(:,6),'--');
hold on;
plot(t,f(:,12),'-.');
hold on;
plot(t,f(:,18),'-');
hold off;
xlabel('Time (s)','Interpreter','Latex');
ylabel('$m_\psi$ (Nm)','Interpreter','Latex');
l = legend('thrust','restoring','damping','Location','Best');
set(l,'Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex');

end