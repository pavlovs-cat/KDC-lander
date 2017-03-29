function [ inertiaMtx ] = pt2D_moi( omega, alpha )
% Inputs:   omega, angular velocity as a 3xn vector
%           alpha, angular acceleration as a 3xn vector
% Output:   inertialMtx, moment of inertia matrix

[~, n] = size(omega);
optimA = [];

I = sym('I', [3 3]);
% alpha = sym('alpha', [3 1]);
% omega = sym('omega', [3 1]);

for i = 1:n
    tau = I*alpha + crosAs(omega,I*omega);
    [A, ~] = equationsToMatrix([tau(1), tau(2), tau(3)], ...
        [I(1,1), I(1,2), I(1,3), I(2,1), I(2,2), ...
        I(2,3), I(3,1), I(3,2), I(3,3)]);
    optimA = vertcat(optimA,A);
end

inertiaMtx = null(optimA);
inertiaMtx = reshape(inertiaMtx,3,3)';
% what coordinate frame is this??
end