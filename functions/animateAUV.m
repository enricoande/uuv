function animateAUV(t,x,nframes,l)
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
% Compute the position of the three extremities of the triad:
p = zeros(n,3,3);
for i=1:n
    for j=1:3
        p(i,:,j) = x_anim(i,1:3)+rotation(x_anim(i,4:6),);
    end
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