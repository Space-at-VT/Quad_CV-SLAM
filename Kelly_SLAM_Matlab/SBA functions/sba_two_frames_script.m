% SBA from two frames

base_path  = 'C:\Users\Daniel\Desktop\Scott\CV_work\octave sfm\vision_algos_0.1\vision_algos_0.1';
addpath(genpath(base_path));
addpath(genpath('../'));

imgs = read_image_data_from_file('dino.tks'); 
im = imread('viff.000.png'); height = size(im,1); width = size(im,2);
K = [width 0 width/2; 0 width height/2; 0 0 1];
ncams = 2; % the number of images

[f1inliers, f2inliers] = findPointMatchesImgCell(1, 2, imgs);
% [F, inliers] = estimateFundamentalMatrix(f1inliers,f2inliers);
[E, inliers] = cv.findEssentialMat(f1inliers,f2inliers); inliers=logical(inliers);
f1inliers=f1inliers(inliers,:);
f2inliers=f2inliers(inliers,:);

tmp = [f1inliers, f2inliers];
pts2D = reshape(tmp,1,size(tmp,1)*size(tmp,2));
pts3D = [f1inliers, ones(size(f1inliers,1),1)];


r0 = [1 0 0 0 1 0 0 0]; % 4 element quaternion for first and second camera pose, in order. initial pose is unknown.
cams=zeros(ncams, 6); % Each row is the initial camera pose estimation - in this case, initializing at the same location with zero rotation and translation

npts = size(f1inliers,1); % the number of 3D points
ncon = 0; % the number of 3D points whose parameters should not be modified
mcon = 0; % the number of images whose parameters should not be modified
vmask = ones(npts,2); %nxm matrix specifying the points visible in each camera. A nonzero element at position (i, j) indicates that point i is visible in image j.
pnp=size(pts3D, 2); % number of parameters for each 3D point
cnp=size(cams, 2); % the number of parameters for each camera
p0=[reshape(cams', 1, ncams*cnp) reshape(pts3D', 1, npts*pnp)];
mnp = 2; % the number of parameters for each image projections
opts=[1E-03, 1E-12, 1E-12, 1E-12, 0.0]; % options vector
cal = [K(1,:), K(2,2), K(2,3)]; % calibration parameters


% motion & structure BA
[ret, p, info]=sba(npts, ncon, ncams, mcon, vmask, p0, cnp, pnp, pts2D, mnp, 'projRTS', 'jacprojRTS', 1000, 1, opts, 'motstr', r0, cal);
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

figure; plot3(pts3D(:,1),pts3D(:,2),pts3D(:,3),'b.'); hold on; axis equal;
plot3(cams(:,5),cams(:,6),cams(:,7),'r.','MarkerSize',20);
break;
im1=im;
im2=imread('viff.001.png');
figure;
showMatchedFeatures(im1,im2,f1inliers,f2inliers,'Montage');
figure;
showMatchedFeatures(im1,im2,f1inliers(inliers,:),f2inliers(inliers,:),'Montage');


