function [ featInliers ] = findReducedInliers( origPts, newPts )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% origPts = visiblePoints(:,2:3);
featInliers = false(size(origPts,1),1);
newPtCtr = 1;
for k = 1:size(origPts,1)
    if k > size(newPts,1)
        % Do nothing
    elseif origPts(k,1) == newPts(newPtCtr,1) && origPts(k,2) == newPts(newPtCtr,2)
        featInliers(k) = true;
        newPtCtr = newPtCtr + 1;
    end
end

end

