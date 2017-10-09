waypoints = [0,0,0;
             2,0,0;
             2,4,0;
             2,4,2];
         
trj = Trajectory('minimum_snap',waypoints,20);

t = 0:0.01:20;
n = length(t);
pos = zeros(3,n);

for i=1:n
    trj = trj.trajectory_generation(t(i));
    pos(:,i) = trj.des_pos;
end

figure;
plot3(pos(1,:),pos(2,:),pos(3,:));