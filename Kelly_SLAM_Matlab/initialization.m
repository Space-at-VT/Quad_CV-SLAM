function [ P, pointCloud ] = initialization( f1pts, f2pts, K1, K2 )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
globalIndex = f1pts(:,1);
P{1} = K1*[eye(3), [0;0;0]];
[F12, inliers] = estimateFundamentalMatrix(f1pts(:,2:3),f2pts(:,2:3),'Method','Norm8Point');
E12 = K2'*F12*K1;
[P2_4Possible] = getCameraMatrix(E12);
f12_testPoint = [f1pts(1,2:3) 1; f2pts(1,2:3) 1]';
[P{2}] = getCorrectCameraMatrix(P2_4Possible, K1,K2, f12_testPoint);
P{2}=K2*P{2};
Xw12p = Triangulation(makehomogeneous(f1pts(inliers,2:3)'),P{1},makehomogeneous(f2pts(inliers,2:3)'),P{2});

pointCloud(1,:) = globalIndex(inliers);
pointCloud(2:4,:) = Xw12p(1:3,:);

end

