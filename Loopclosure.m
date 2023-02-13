clear all
load('hw4_data.mat')
trajectory = gtsam.Values;    %Estimated poses
groundTruth = gtsam.Values;   %Actual Gt 3d
for ii = 1:length(traj3)
    traj = gtsam.Pose3(traj3{ii});
    gt = gtsam.Pose3(poses3_gt{ii});
    key = gtsam.symbol('x',ii);
    trajectory.insert(key,traj)
    groundTruth.insert(key,gt)
end
figure(1);
gtsam.plot3DTrajectory(trajectory,'*-r',[]);
hold on
gtsam.plot3DTrajectory(groundTruth,'*-b',[]);
view ([0,-1,0]) ; 
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
title('Estimated Trajectory vs Ground Truth')
legend('Estimated trajectory','Ground Truth')

%Factor graph
graph = gtsam.NonlinearFactorGraph;
%Model and init position
noiseModel = gtsam.noiseModel.Diagonal.Sigmas([(1e-3)*ones(1,3),0.1*ones(1,3)]');
origin = gtsam.Pose3(eye(4));
key1 = gtsam.symbol('x',1);
graph.add(gtsam.PriorFactorPose3(key1,origin,noiseModel))
for ii = 1:length(traj3)-1
    key1 = gtsam.symbol('x',ii);
    key2 = gtsam.symbol('x',ii+1);
    pos  = gtsam.Pose3(dpose{ii});
    graph.add(gtsam.BetweenFactorPose3(key1,key2,pos,noiseModel))
end

optimizer = gtsam.LevenbergMarquardtOptimizer(graph,trajectory);
result = optimizer.optimizeSafely();
marginals = gtsam.Marginals(graph,result);
figure(2)
gtsam.plot3DTrajectory(groundTruth,'*-b',[]);
hold on
gtsam.plot3DTrajectory(result,'*-r',[],[],marginals);
view ([0,-1,0]) ; 
axis equal tight
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
title('Optimized trajectory vs Ground Truth')

R21 = [0.330571768  0.0494690228    -0.942483486;
          0.0138000518 0.998265226     0.057237196;
          0.943679959  -0.0319273223   0.329315626];
      
t211=[-24.1616858; -0.0747429903; 275.434963];

key11 = gtsam.symbol('x',4);
key22 = gtsam.symbol('x',43);

pose12 = gtsam.Pose3([R21,t211;[0 0 0 1]]);
graph.add(gtsam.BetweenFactorPose3(key11,key22,pose12,noiseModel));
optimizer1 = gtsam.LevenbergMarquardtOptimizer(graph,trajectory);
result1 = optimizer1.optimizeSafely();
marginals1 = gtsam.Marginals(graph,result1);
figure(3)
gtsam.plot3DTrajectory(groundTruth,'*-b',[]);
hold on
gtsam.plot3DTrajectory(result1,'*-r',[],[],marginals1);
view ([0,-1,0]) ; 
axis equal tight
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
title('Optimized trajectory with loop clouser vs Ground Truth')


OL = gtsam.utilities.extractPose3(result);
CL = gtsam.utilities.extractPose3(result1);
GT = gtsam.utilities.extractPose3(groundTruth);

OL = OL(:,10:12);
CL = CL(:,10:12);
GT = GT(:,10:12);

for ii = 1:length(OL)
    OL_DIST(ii) = norm(OL(ii,:)-GT(ii,:));
    CL_DIST(ii) = norm(CL(ii,:)-GT(ii,:));
end

figure(4)
plot(1:1:length(OL),OL_DIST,'o-')
hold on
plot(1:1:length(OL),CL_DIST,'o-')
title('Loop clouser vs Open loop');
legend('Open Loop Error','Closed Loop Error');
xlabel('Time')
ylabel('Error [m]')
     
    
