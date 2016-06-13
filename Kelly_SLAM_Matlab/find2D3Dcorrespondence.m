function [ has3dIndex, pcIndex ] = find2D3Dcorrespondence( pointCloud, visiblePoints )
%find2D3Dcorrespondence Finds visible 2D points that already have a 3d location

global3dIndex = pointCloud(1,:)'; % Find list of global 3D points
global2dVisible = visiblePoints(:,1);% Find global index of currently visible 2D points
has3dIndex = ismember(global2dVisible,global3dIndex); % Index for 2D visible points that have a 3D point
pcIndex = ismember(global3dIndex,global2dVisible); % Index for matching 3D points in pointCloud


end

