clear; clc;
% Script for testing differences in Matlab Matrix operations & NumPy array
% operations

A = 2*randn(4)

% first test on a random 4x4 array;
% A = [-2.1377 0.6504 -0.2045 -1.7298;
%     -1.6190 -1.5099 -0.4829 -0.0601;
%     -5.8886 2.7406 0.6384 -0.3298;
%     2.8768 -3.4230 0.6257 1.2554]

% second randomized 4x4 array
% A = [1.0753    0.6375    7.1568    1.4508;
%     3.6678    -2.6154    5.5389   -0.1261;
%     -4.5177   -0.8672   -2.6998    1.4295;
%     1.7243     0.6852    6.0698   -0.4099]

A1n = sqrt(sum(A(1,:).*A(1,:)))
A2n = sqrt(sum(A(2,:).*A(2,:)))
A3n = sqrt(sum(A(3,:).*A(3,:)))
A4n = sqrt(sum(A(4,:).*A(4,:)))
Anorm = [A(1,:)/A1n
    A(2,:)/A2n
    A(3,:)/A3n
    A(4,:)/A4n]
