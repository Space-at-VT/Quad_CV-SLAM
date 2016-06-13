function [ convexHull, boundingBox, roiConvexHull, mask, hullPixelList  ] = motionHull( im1, im2, dilation, plotAll  )
%UNTITLED2 Summary of this function goes here
%   Assumes image homographies are already rectified (same background, star
%   image aligned, no motion, etc)

%% Subtract images to find region of interest
avFrame = (double(im1)+double(im2))/2;
sub21 = abs(double(im2)-double(im1))./avFrame ;
mult = abs(double(im2)-double(im1));
mult = (mult - max(max(mult))*.05);
sub21 = uint8(mult.*sub21).^2;
% sub21 = (sub21 - max(max(sub21))*.05).^2;
if plotAll == 2;
    figure;
    imshow(sub21);
end

gaussianFilter = fspecial('gaussian',[5 5],3);

filterSub21 = imfilter(sub21,gaussianFilter);
% filterSub21 = (filterSub21 - max(max(filterSub21))*.05).^2;
if plotAll == 2;
    figure;
    imshow(filterSub21);
end
% filterSub21(1:400,:) = 0; 
% filterSub21(1300:end,:) = 0; 
% filterSub21(1:175,:) = 0; % crop out the dish manually
% filterSub21(1:390,:) = 0; % crop out the satellite manually, only leave the stand
% filterSub21(390:720,:) = 0; % crop out the stand manually, only leave the satellite
% filterSub21(:,1:545) = 0;
%% Show values over a certain threshold
mult = 2;
% graythresh(filterSub21)*255
level = mult*graythresh(filterSub21);
if level > 1, level = 1; end;
bin21 = im2bw(filterSub21,level);

while sum(sum(bin21)) == 0
    mult = mult*.95;
    level = mult*graythresh(filterSub21);
    if level > 1, level = 1; end;
    bin21 = im2bw(filterSub21,level);
end
    

if plotAll == 2;
    figure;
    imshow(bin21);
end


%% Define convex hull and create mask

regions = regionprops(bin21,'PixelList');
allPixels = [];
for i=1:length(regions)
    allPixels = [allPixels; regions(i).PixelList];
end
if size(allPixels,1) == 0
      error(['No motion or need to adjust threshold. Do some investigating!']);
end
convexHull = convhull(allPixels(:,1),allPixels(:,2));
if plotAll == 2;
    hold on, plot(allPixels(convexHull,1), allPixels(convexHull,2), '-r'), hold off;
end


roiMask21 = roipoly(im1,allPixels(convexHull,1), allPixels(convexHull,2));
if plotAll == 2;
    figure;
    imshow(roiMask21);
end

%% Dilate the region of interest

se = strel('disk',dilation);
droi21 = imdilate(roiMask21,se);
if plotAll == 2;
    figure;
    imshow(droi21);
end

%% Get the pixel list for all the pixels in the ROI droi21
tmp = regionprops(droi21,'PixelList');
hullPixelList = tmp.PixelList;

%% See what is left of original image
droi21Mask = droi21;
% droi21Mask(380:720,:) = 0; % crop out the stand manually


filtIm1 = im1.*uint8(droi21Mask);
filtIm2 = im2.*uint8(droi21Mask);
if plotAll == 2;
    figure;
    imshow(filtIm1);
end
mask = droi21;
%% Calculate bounding box and create smaller single image

bbox21 = regionprops(droi21Mask,'BoundingBox','PixelList');
bbox21.BoundingBox = ceil(bbox21.BoundingBox);
boundingBox = bbox21.BoundingBox;

%% Re-calculate the bigger convex hull

roiPixels = bbox21.PixelList;
roiConvexHull = convhull(roiPixels(:,1),roiPixels(:,2));


%% Plot image 1 with convex hull drawn
if plotAll == 1 || plotAll == 2
    figure;
    imshow(im1);
    hold on;
    plot(allPixels(convexHull,1), allPixels(convexHull,2), '-r');
    rectangle('Position',boundingBox, 'EdgeColor', [0 0 1]);
    plot(roiPixels(roiConvexHull,1), roiPixels(roiConvexHull,2), '-y');
    hold off;
    
end

convexHull = [allPixels(convexHull,1), allPixels(convexHull,2)];
roiConvexHull = [roiPixels(roiConvexHull,1), roiPixels(roiConvexHull,2)];
end

