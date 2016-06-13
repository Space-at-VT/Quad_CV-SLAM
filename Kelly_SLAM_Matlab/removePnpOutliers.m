function [ pointCloud, visiblePoints ] = removePnpOutliers( pointCloud, pcIndex, pnpIdxInliers, visiblePoints, frameCtr )
%removePnpOutliers Removes outliers from the visible tracked points and the point cloud 
%after the RANSAC pnp process results in a vector of outliers


numOutliers = size(pointCloud(2:4,pcIndex)',1) - size(pnpIdxInliers,1);
numOutStr = num2str(numOutliers);
pcPtStr = num2str(size(pointCloud(2:4,pcIndex)',1));
pnpInStr = num2str(size(pnpIdxInliers,1));

dispStr = strcat('Frame: ', num2str(frameCtr), '. Num 3D points matched for PnP: ', pcPtStr, '. Num inliers in PnP RANSAC: ', pnpInStr, '. Num  outliers: ', numOutStr, '.');
disp(dispStr);
if size(pnpIdxInliers,1) < 4
    error('There aren''t enough PnP inliers to calculate a solution!');
end
if size(pnpIdxInliers,1)/size(pointCloud(2:4,pcIndex)',1) < .25
    error('There are way too many PnP outliers compared to inliers, something is wrong')
end

% if size(pnpIdxInliers,1) < 10
%     error('There are way too many PnP outliers compared to inliers, something is wrong')
% end

% Find the global point numbers of the outliers
pnpOutliers = true(size(pointCloud(2:4,pcIndex),2),1);
pnpOutliers(pnpIdxInliers) = false;
pcNumIndex = find(pcIndex == true);
pcOutlierIndex = pcNumIndex(pnpOutliers);
globalOutliers = pointCloud(1,pcOutlierIndex)';

% Delete the global outliers from the visible tracked points
visibleInliers = not(ismember(visiblePoints(:,1),globalOutliers));
visiblePoints = visiblePoints(visibleInliers,:);
% ptLifeInliers = ptLifeInliers(visibleInliers,:);

% Delete the global outliers from the point cloud
tmp = true(size(pointCloud,2),1);
tmp(pcOutlierIndex) = false;
pointCloud = pointCloud(:,tmp);

end

