function [ pointCloud, distThreshold, center ] = initPcCleanup( pointCloud )
%initPcCleanup % Calculate the center of all the points - then the StdDev of all the point locations. 
%Remove any points with a depth g.t. sigma*sdev from the center
%   Detailed explanation goes here

sig = 2;
center = [mean(pointCloud(2,:)) mean(pointCloud(3,:)) mean(pointCloud(4,:))];
sdev = [std(pointCloud(2,:)) std(pointCloud(3,:)) std(pointCloud(4,:))];
wCtr = 0;
while sdev(3)/center(3) > .2 && wCtr < 5
    pcStdOutlier = find(abs(pointCloud(4,:)) > abs(center(3)) + sig*abs(sdev(3)));
    keepIdx = true(1,size(pointCloud,2));
    keepIdx(pcStdOutlier) = false;
    pointCloud = pointCloud(:,keepIdx);
    center = [mean(pointCloud(2,:)) mean(pointCloud(3,:)) mean(pointCloud(4,:))];
    sdev = [std(pointCloud(2,:)) std(pointCloud(3,:)) std(pointCloud(4,:))];
    wCtr = wCtr + 1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gather some stats on the point cloud
distFromCenter = pdist2(center,pointCloud(2:4,:)','euclidean'); % Calculate the distance of each pre-existing point from the center
maxDist = max(distFromCenter); % Extract the max distance point
distThreshold = 2*maxDist; % Set the distance threshold as a multiple x the maximum distance point from the center



end

