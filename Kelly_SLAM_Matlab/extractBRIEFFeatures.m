function [ features, points ] = extractBRIEFFeatures( img, points )
%extractBRIEFFeatures Extracts the binary BRIEF feature from the input
%image at the location of the array of [x,y] coordinates defined by points
%   Detailed explanation goes here

extractor = cv.DescriptorExtractor('BRIEF');

for m = 1:size(points,1)
    keypoints(m).pt = points(m,:);
    keypoints(m).size = 1;
    keypoints(m).angle = -1;
    keypoints(m).response = 1;
    keypoints(m).octave = 0;
    keypoints(m).class_id = -1;
end


[features, inliers] = extractor.compute(img,keypoints);
features = double(features);

if size(features,1) ~= size(points,1) % if one of the 
end

points = [inliers.pt];
points = reshape(points,2,size(points,2)/2)';
points = single(points);
end

