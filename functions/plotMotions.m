function plotMotions(t,states)
% plotMotions.m     e.anderlini@ucl.ac.uk     13/09/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is used to plot the translations and rotations in the
% inertial reference frame and the translational and rotational velocities
% in the body-fixed reference frame of the ROV.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Translations and rotations in the inertial reference frame:
figure;
subplot(2,1,1);
plot(t,states(:,1),'--');
hold on;
plot(t,states(:,2),'-.');
hold on;
plot(t,states(:,3),'-');
hold off;
ylabel('Displacement (m)','Interpreter','Latex');
l = legend('$x$','$y$','$z$','Location','Best');
set(l,'Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex')
subplot(2,1,2);
plot(t,rad2deg(states(:,4)),'--','Color',[0.4940,0.1840,0.5560]);
hold on;
plot(t,rad2deg(states(:,5)),'-.','Color',[0.4660,0.6740,0.1880]);
hold on;
plot(t,rad2deg(states(:,6)),'-','Color',[0.3010,0.7450,0.9330]);
hold off;
xlabel('Time (s)','Interpreter','Latex');
ylabel('Rotation ($^\circ$)','Interpreter','Latex');
l = legend('$\phi$','$\theta$','$\psi$','Location','Best');
set(l,'Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex')
set(gcf,'color','w');

%% Translational and rotational velocity in the body-fixed reference frame:
figure;
subplot(2,1,1);
plot(t,states(:,7),'--');
hold on;
plot(t,states(:,8),'-.');
hold on;
plot(t,states(:,9),'-');
hold off;
ylabel('Translational velocity (m/s)','Interpreter','Latex');
l = legend('$u$','$v$','$w$','Location','Best');
set(l,'Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex')
subplot(2,1,2);
plot(t,rad2deg(states(:,10)),'--','Color',[0.4940,0.1840,0.5560]);
hold on;
plot(t,rad2deg(states(:,11)),'-.','Color',[0.4660,0.6740,0.1880]);
hold on;
plot(t,rad2deg(states(:,12)),'-','Color',[0.3010,0.7450,0.9330]);
hold off;
xlabel('Time (s)','Interpreter','Latex');
ylabel('Rotational velocity ($^\circ$/s)','Interpreter','Latex');
l = legend('$p$','$q$','$r$','Location','Best');
set(l,'Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex')
set(gcf,'color','w');

end