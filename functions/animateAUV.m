function animateAUV(t,x,nframes)
% animateAUV.m     e.anderlini@ucl.ac.uk     15/09/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is used to plot the path of the ROV.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Keep only the desired frames for the animation:
n = floor(length(t)/nframes);
t_anim = zeros(n,1);
x_anim = zeros(n,6);
for i=1:n
    t_anim(i) = t((i-1)*nframes+1);
    x_anim(i,:) = x((i-1)*nframes+1,1:6);
end


%%
figure;
for i=1:n
    scatter3(x_anim(i,1),x_anim(i,2),x_anim(i,3));
    hold on;
end

plot3(x_anim(:,1),x_anim(:,2),x_anim(:,3),'--','Color',[0.8500,0.3250,0.0980]);
hold off;
xlabel('$x$ (m)','Interpreter','Latex');
ylabel('$y$ (m)','Interpreter','Latex');
zlabel('$z$ (m)','Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex')
set(gcf,'color','w');

end