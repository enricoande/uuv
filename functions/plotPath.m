function plotPath(states,waypoints)
% plotPath.m     e.anderlini@ucl.ac.uk     15/09/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is used to plot the path of the ROV.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure;
plot3(states(:,1),states(:,2),states(:,3));
hold on;
scatter3(waypoints(:,1),waypoints(:,2),waypoints(:,3),'MarkerEdgeColor',...
    [0.8500,0.3250,0.0980]);
xlabel('$x$ (m)','Interpreter','Latex');
ylabel('$y$ (m)','Interpreter','Latex');
zlabel('$z$ (m)','Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex')
set(gcf,'color','w');

end