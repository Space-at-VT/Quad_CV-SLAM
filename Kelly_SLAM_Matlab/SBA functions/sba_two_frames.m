function [ pts3D, P2return, info  ] = sba_two_frames( f1pts, f2pts, pointCloudMatches, P1, P2, K1, ncon, mcon, numIter )
%sba_two_frames Runs Sparse Bundle Adjustment on two frames and updates the 3D points and second camera projection matrix
%   [ pts3D, P2return  ] = sba_two_frames( f1pts, f2pts, pointCloudMatches, P1, P2, K1 )
%   f1pts and f2pts - 2D locations of points in images 1 and 2. Format for each is N x 2.
%
%   pointCloudMatches - the 3D locations of the points. Needs to be the same length as
%   f1inliers and f2inliers. Format N x 3.
% 
%   P1 and P2 - initial camera projection matrix estimates
% 
%   K1 - camera calibration matrix
%   
%   ncon - the number of 3D points whose parameters should not be modified (the first X number of points passed to the function won't be modified)
% 
%   mcon - the number of images whose parameters should not be modified (ie, if 1 - leave the first camera where it started)
% 
%   Usage: [pts3Df, P2f] = sba_two_frames(f1pts(:,2:3), f2pts(:,2:3), pointCloud(2:4,:)',P{1},P{2},K1);

invK = inv(K1);
size1 = size(f1pts,1);
size2 = size(f2pts,1);
size3 = size(pointCloudMatches,1);

if size1 ~= size2 || size1 ~= size3 || size2 ~= size3
    error('Error! Incorrect input to two frames bundle adjustment script. f1pts, f2pts, and pointCloudMatches should all have the same number of points.');
end


iR1 = invK*P1(:,1:3);
iR2 = invK*P2(:,1:3);
iq1 = quaternion(iR1);
iq2 = quaternion(iR2);
it1 = invK*P1(:,4);
it2 = invK*P2(:,4);

r0 = [iq1' iq2'];
cams = [iq1(2:4)' it1'; iq2(2:4)' it2'];

tmp = [f1pts, f2pts];
pts2D = reshape(tmp',1,size(tmp,1)*size(tmp,2));
% pts3D = [f1pts(:,2:3) ones(size(f1pts,1),1)];
pts3D = pointCloudMatches;

% figure; plot3(pts3D(:,1),pts3D(:,2),pts3D(:,3),'b.'); hold on; axis equal;
% plot3(cams(:,5),cams(:,6),cams(:,7),'r.','MarkerSize',20);


ncams = 2;
% r0 = [1 0 0 0 1 0 0 0]; % 4 element quaternion for first and second camera pose, in order. initial pose is unknown.
% cams=zeros(ncams, 6); % Each row is the initial camera pose estimation - in this case, initializing at the same location with zero rotation and translation

npts = size(f1pts,1); % the number of 3D points
% ncon = 0; % the number of 3D points whose parameters should not be modified
% mcon = 1; % the number of images whose parameters should not be modified (1 - leave the first camera where it started)
vmask = ones(npts,2); %nxm matrix specifying the points visible in each camera. A nonzero element at position (i, j) indicates that point i is visible in image j.
pnp=size(pts3D, 2); % number of parameters for each 3D point
cnp=size(cams, 2); % the number of parameters for each camera
p0=[reshape(cams', 1, ncams*cnp) reshape(pts3D', 1, npts*pnp)];
mnp = 2; % the number of parameters for each image projections
opts=[1E-03, 1E-12, 1E-12, 1E-12, 0.0]; % options vector
cal = [K1(1,:), K1(2,2), K1(2,3)]; % calibration parameters

p0=double(p0);
pts2D = double(pts2D);
% numIter = 5;

% motion & structure BA

[ret, p, info]=sba(npts, ncon, ncams, mcon, vmask, p0, cnp, pnp, pts2D, mnp, 'projRTS', 'jacprojRTS', numIter, 1, opts, 'motstr', r0, cal);

% structure only BA
% [ret, p, info]=sba(npts, 0, ncams, 1, spmask, p0, cnp, pnp, pts2D, 2, 'projRTS', 'jacprojS', 100, 1, opts, 'str', r0, cal);


% sba finished; retrieve the refined motion & structure from the output vector
cams=reshape(p(1:ncams*cnp), cnp, ncams)';
% derive the full (local) unit quaternions from their vector parts and combine with the initial rotation
cams=[zeros(ncams, 1), cams]; % augment with a leading column
for i=1:ncams
  % note that the cams column indices below are incremented by one to account for the extra column above!
  qrl=[sqrt(1 - (cams(i, 2)^2 + cams(i, 3)^2 + cams(i, 4)^2)), cams(i, 2), cams(i, 3), cams(i, 4)];
  qr0=r0((i-1)*4+1:i*4);
  % tmp=qrl*qr0;
  tmp=[qrl(1)*qr0(1) - dot(qrl(2:4), qr0(2:4)),  qrl(1)*qr0(2:4) + qr0(1)*qrl(2:4) + cross(qrl(2:4), qr0(2:4))];
  if(tmp(1)<0), tmp=-tmp; end; % ensure scalar part is non-negative
  cams(i, 1:4)=tmp;
end
pts3D=reshape(p(ncams*cnp+1:ncams*cnp+npts*pnp), pnp, npts)';

% figure; plot3(pts3D(:,1),pts3D(:,2),pts3D(:,3),'b.'); hold on; axis equal;
% plot3(cams(:,5),cams(:,6),cams(:,7),'r.','MarkerSize',20);
nP1 = [quaternion(cams(1,1:4)), cams(1,5:7)'];
nP2 = [quaternion(cams(2,1:4)), cams(2,5:7)'];
P1return = K1*nP1;
P2return = K1*nP2;



end

