function [ P, pointCloud ] = scaleScene( P, pointCloud, scaleSceneBy )
%scaleScene Scales an entire 3D scene generated from SLAM a ratio defined by scaleSceneBy

% First, scale the point cloud
pointCloud(2:4,:) = scaleSceneBy*pointCloud(2:4,:);

% Second, scale the motion parameters - rotation portion should be constant, need to scale camera centers/translation vector
for i = 1:length(P)
    P{i}(:,4) = P{i}(:,4)*scaleSceneBy;
end


end

