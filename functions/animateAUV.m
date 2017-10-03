function animateAUV(t,x,nframes,l,fspeed)
% animateAUV.m     e.anderlini@ucl.ac.uk     15/09/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is used to plot the path of the ROV.
%
% Input:
% t:        time series vector;
% x:        states matrix;
% nframes:  no. frames per animation step;
% l:        length of the arms of the moving triad;
% fspeed:   speed of the time frame - be careful, select even number.
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
arm = [l,0,0;0,l,0;0,0,l];
p = zeros(n,3,3);
for i=1:n
    for j=1:3
        p(i,:,j) = x_anim(i,1:3)+rotation(x_anim(i,4:6),arm(:,j))';
    end
end
% Compute the time step of the animation:
dt = (t_anim(2)-t_anim(1))/fspeed;
% Invert the z-axis to make animation more intuitive:
p(:,3,:) = -p(:,3,:);
x_anim(:,3) = -x_anim(:,3);

%% Animate the triad's motion:
figure;
% tmpAspect=daspect();
% daspect(tmpAspect([1 1 1]));
xlabel('$x$ (m)','Interpreter','Latex');
ylabel('$y$ (m)','Interpreter','Latex');
zlabel('$z$ (m)','Interpreter','Latex');
set(gca,'TickLabelInterpreter','Latex')
set(gcf,'color','w');
for i=1:n
    plot3(x_anim(:,1),x_anim(:,2),x_anim(:,3),'--','Color',...
    [0.4940,0.1840,0.5560]);
    hold on;
    plot3([x_anim(i,1),p(i,1,1)],[x_anim(i,2),p(i,2,1)],[x_anim(i,3),...
        p(i,3,1)],'Color',[0,0.4470,0.7410]);
    hold on;
    plot3([x_anim(i,1),p(i,1,2)],[x_anim(i,2),p(i,2,2)],[x_anim(i,3),...
        p(i,3,2)],'Color',[0.8500,0.3250,0.0980]);
    hold on;
    plot3([x_anim(i,1),p(i,1,3)],[x_anim(i,2),p(i,2,3)],[x_anim(i,3),...
        p(i,3,3)],'Color',[0.4660,0.6740,0.1880]);
    hold on;
    scatter3(x_anim(i,1),x_anim(i,2),x_anim(i,3),'filled',...
         'MarkerFaceColor',[0.4940,0.1840,0.5560]);
%     scatter3(x_anim(i,1),x_anim(i,2),x_anim(i,3),'filled',...
%         'MarkerFaceColor',[0,0.4470,0.7410]);
%     hold on;
%     scatter3(p(i,1,1),p(i,2,1),p(i,3,1),'filled',...
%         'MarkerFaceColor',[0.9290,0.6940,0.1250]);
%     hold on;
%     scatter3(p(i,1,2),p(i,2,2),p(i,3,2),'filled',...
%         'MarkerFaceColor',[0.4940,0.1840,0.5560]);
%     hold on;
%     scatter3(p(i,1,3),p(i,2,3),p(i,3,3),'filled',...
%         'MarkerFaceColor',[0.4660,0.6740,0.1880]);
    hold off;
    daspect([1 1 1]);
    pause(dt);
end

end