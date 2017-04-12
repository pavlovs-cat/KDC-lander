function [ q,w,a ] = pt2E_traj(q0, w0, a0, I, tstep, N)
%pt2E_traj  Predicts the future trajectory for N timesteps of size tstep 
%           using Euler integration.

q = [q0 zeros(4, N)];
w = [w0 zeros(3, N)];
a = [a0 zeros(3, N)];

R1 = quat2rotm(q0);
for k = 1:N
    R0 = R1;
    wk = w(:, k);
    S = [  0     -wk(3)  wk(2);
          wk(3)    0    -wk(1);
         -wk(2)   wk(1)    0  ];
    R1 = R0 + S*R0'*tstep;
    q(:, k+1) = rotm2quat(R1);
    w(:, k+1) = wk + a(:, k)*tstep;
    a(:, k+1) = -pinv(I)*cross(wk, I*wk);
end

q = q(:, 2:end);
w = w(:, 2:end);
a = a(:, 2:end);