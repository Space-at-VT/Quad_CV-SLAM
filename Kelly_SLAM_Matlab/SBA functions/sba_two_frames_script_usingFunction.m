% SBA from two frames
% clear all, close all, clc;
% getting the dinosaur data
% base_path  = 'C:\Users\Daniel\Desktop\Scott\CV_work\octave sfm\vision_algos_0.1\vision_algos_0.1';
% addpath(genpath(base_path));
% addpath(genpath('../'));
% imgs = read_image_data_from_file('dino.tks'); 
% im = imread('viff.000.png'); height = size(im,1); width = size(im,2);
% K = [width 0 width/2; 0 width height/2; 0 0 1];
% done getting the dino data

% shaping the data, calling sba
[f1inliers, f2inliers] = findPointMatchesImgCell(1, 2, imgs);
[F, inliers] = estimateFundamentalMatrix(f1inliers,f2inliers);
f1inliers=f1inliers(inliers,:);
f2inliers=f2inliers(inliers,:);


% The 3d data points
pts3D = [f1inliers, ones(size(f1inliers,1),1)]; % unknown locations, initializing to unit depths for SBA

% initial camera guesses - initialization
ncon = 0; % don't maintain any 3d point locations
R1 = eye(3); % don't know the initial camera rotations
R2 = eye(3);
c1 = [0 0 0]; % don't know the camera center locations
c2 = [0 0 0];
init = 1; % since it is initialization, modify the positon of both cameras since they're both just at the origin

nItr = 1000; % number of SBA iterations

[ pts3D, nR1, nR2, nc1, nc2 ] = sba_two_frames( f1inliers, f2inliers, pts3D, ncon, K, R1, R2, c1, c2, nItr, init )











figure; plot3(pts3D(:,1),pts3D(:,2),pts3D(:,3),'b.'); hold on; axis equal;
plot3(nc1(:,1),nc1(:,2),nc1(:,3),'r.','MarkerSize',20);
plot3(nc2(:,1),nc2(:,2),nc2(:,3),'r.','MarkerSize',20);


break;
im1=imread('viff.000.png');
im2=imread('viff.001.png');
figure;
showMatchedFeatures(im1,im2,f1inliers,f2inliers,'Montage');
figure;
showMatchedFeatures(im1,im2,f1inliers(inliers,:),f2inliers(inliers,:),'Montage');


