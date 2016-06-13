function [ rectanglesX, rectanglesY ] = divideRectangle( boundingBox, numDiv )
%divideRectangle divides a rectangular region defined by boundingBox into a
%certain number of sub-rectangular regions, defined by numDiv
%   Detailed explanation goes here
totalX = boundingBox(3)-1;
totalY = boundingBox(4)-1;

xStart = boundingBox(1);
yStart = boundingBox(2);

xSize = floor(totalX/numDiv);
ySize = floor(totalY/numDiv);

xPoints=xStart:xSize:totalX+xStart;
if totalX+xStart > xPoints(end)
    xPoints(end) = totalX+xStart;
end

yPoints = yStart:ySize:totalY+yStart;
if totalY+yStart > yPoints(end)
    yPoints(end) = totalY+yStart;
end
rectanglesX = [];
rectanglesY = [];
ctr=1;

% imshow(plotFrame);
% hold on;
for i = 1:numDiv
    for j = 1:numDiv
        rectanglesX(:,ctr) = [xPoints(i);
                              xPoints(i);
                              xPoints(i+1);
                              xPoints(i+1);
                              xPoints(i)];       
        
        rectanglesY(:,ctr) = [yPoints(j);
                              yPoints(j+1);
                              yPoints(j+1);
                              yPoints(j);
                              yPoints(j)];
       
     
        ctr = ctr+1;
    end
end

end

