function plotCamera( P, invK, color )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
C = -inv(P(1:3,1:3))*P(:,4);
R = inv(invK*P(1:3,1:3));
H = [R C; zeros(1,3), 1];
scale = 3.5;
f_3Dframe(H,color,0.2*scale,'_{d}');
f_3Dcamera(H,color,0.05*scale); 
% f_3Dframe(H,color,0.6,'_{d}');
% f_3Dcamera(H,color,0.15); 

end

