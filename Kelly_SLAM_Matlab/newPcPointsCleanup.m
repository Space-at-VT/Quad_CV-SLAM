function [ Xw, globalIndex ] = newPcPointsCleanup( Xw, distThreshold, center, globalIndex )
%newPcPointsCleanup Cleans up new points being added to the point cloud
% Clean up bad points - calling them bad under 2 circumstances:
% 1 - bad if negative depth
negIdx = find( Xw(3,:) <= 0 ); % is the depth negative?
if size(negIdx,2) > 0 % If there are negative depths
    % remove them from the points that are going to get allocated to
    % pointCloud
    tmp = true(size(Xw,2),1);
    tmp(negIdx) = false;
    Xw = Xw(:,tmp); % Get rid of that 3d point 
    globalIndex = globalIndex(tmp);
%     f1pts = f1pts(tmp,:);
%     f2pts = f2pts(tmp,:);
end

% 2 - if the distance from the center of the point cloud is greater than distThreshold
newPtDist = pdist2(double(center),double(Xw(1:3,:))','euclidean')'; % Calculate the distance of each new point from the center
test1 = find(newPtDist > distThreshold); % Find any points that are greater than the distance threshold away from the center

if size(test1,1) > 0
    tmp = true(size(Xw,2),1);
    tmp(test1) = false;      
    Xw = Xw(:,tmp); % Remove the 3D points outside the specified range (in test1)
    globalIndex = globalIndex(tmp);
%     f1pts = f1pts(keep,:);
%     f2pts = f2pts(keep,:);
end

end

