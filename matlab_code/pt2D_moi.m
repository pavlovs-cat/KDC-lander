function [ inertiaMtx ] = pt2D_moi( omega, alpha )
% Inputs:   omega, angular velocity as a 3xn vector
%           alpha, angular acceleration as a 3xn vector
% Output:   inertialMtx, moment of inertia matrix

[~, n] = size(omega);
A = zeros(3*n, 6);

% form matrix Ap = 0 to find moment of inertial matrix entries
for i = 1:n
    index = (i-1)*3;
    mtx1 = ...
        [alpha(1,i) 0 0 0 alpha(3,i) alpha(2,i);
        0 alpha(2,i) 0 alpha(3,i) 0 alpha(1,i);
        0 0 alpha(3,i) alpha(2,i) alpha(1,i) 0];
    mtx2 = [0 -omega(3,i) omega(2,i); omega(3,i) 0 -omega(1,i); ...
        -omega(2,i) omega(1,i) 0]* ...
        [omega(1,i) 0 0 0 omega(3,i) omega(2,i); ...
        0 omega(2,i) 0 omega(3,i) 0 omega(1,i);
        0 0 omega(3,i) omega(2,i) omega(1,i) 0];
    A(index+1:index+3,:) = mtx1 + mtx2;
end

[~,~,v] = svd(A);
p = v(:,end);
inertiaMtx = [p(1) p(6) p(5);
              p(6) p(2) p(4);
              p(5) p(4) p(3)];
inertiaMtx(abs(inertiaMtx)<0.01) = 0;
inertiaMtx = abs(inertiaMtx);
end