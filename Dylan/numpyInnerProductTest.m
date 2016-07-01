clear; clc;
A = 2*randn(4)



A1n = sqrt(sum(A(1,:).*A(1,:)))
A2n = sqrt(sum(A(2,:).*A(2,:)))
A3n = sqrt(sum(A(3,:).*A(3,:)))
A4n = sqrt(sum(A(4,:).*A(4,:)))
Anorm = [A(1,:)/A1n
    A(2,:)/A2n
    A(3,:)/A3n
    A(4,:)/A4n]
