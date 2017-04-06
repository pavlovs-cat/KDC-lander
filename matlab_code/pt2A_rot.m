
%clear

% File name to read data from
data_file = 'd00063-default';

% 100Hz for 10 seconds
samplingRate = 100;
tau = 1/samplingRate;
timespan = 100;
times = tau : tau : timespan - tau;

n = {};
for m = 0:7
    n(end+1:end+3) = {sprintf('%dx',m), sprintf('%dy',m),sprintf('%dz',m)};
end
N = length(times)-1;
pos_vec = extract_data(data_file, 'ml', n, 1:(N+1));

% Generate COMs
[com, vel] = optim_com();

% store quaternions for each timestamp
artifact_rot = zeros(N,4);
Rots = zeros(3, 3, N+1);
Rots(:, :, 1) = eye(3);
th = zeros(4, N);
th2 = zeros(4, N+1);
th2(:, 1) = [1 0 0 0]';
Rw2l = quat2rotm([0 0 0 1]);
Rots(:, :, 1) = eye(3); 
com0 = com;

for j = 2:N+1
    com1 = com0 + vel*tau;
    p = reshape(pos_vec(j-1, :),3,8);
    q = reshape(pos_vec(j, :),3,8);
    [artifact_rot(j-1,:), R] = pt2A_helper(p,q, com0, com1);
    Rots(:,:,j) = R*Rots(:,:,j-1);
    th2(:, j) = quatmultiply(artifact_rot(j-1, :), th2(:, j-1)')'; 
    th(:, j-1) = rotm2quat(Rots(:,:,j));
    com0 = com1;
end

th2 = th2(:, 2:end);
th2 = [th2(1, :); -th2(3, :); th2(2, :); th2(4, :)];
Rots = Rots(:, :, 2:end);
th_sim =  extract_data(data_file, 'a_q', {'0', '1', '2', '3'}, 1:N)';

% Plot velocities for comparison
% plot_results(th_sim(1, :), th2(1, :), times(1:N), 'q_0');
% plot_results(th_sim(2, :), th2(2, :), times(1:N), 'q_1');
% plot_results(th_sim(3, :), th2(3, :), times(1:N), 'q_2');
% plot_results(th_sim(4, :), th2(4, :), times(1:N), 'q_3');

% Extract simulated ang accels and velocities
w_sim = extract_data(data_file, 'a_w', {'x', 'y', 'z'}, 1:N)';

% Convert angv to assignment frame
for idx = 1 : N
    Rots(:, :, idx) = quat2rotm(th2(:, idx)');
end

% Compute angular velocities
w_vec = pt2B_angv(Rots, tau);
w_vec2 = pt2B_angv2(Rots, tau);

% Plot velocities for comparison
% plot_results(w_sim(1, :), w_vec2(1, :), times(1:N), 'w_x');
% plot_results(w_sim(2, :), w_vec2(2, :), times(1:N), 'w_y');
% plot_results(w_sim(3, :), w_vec2(3, :), times(1:N), 'w_z');

% Compute angular accelerations
w_dot_vec = pt2C_angacc(w_vec2, tau);

% Plot accelerations for comparison
%plot_results(w_dot_sim(1, :), w_dot_vec(1, :), times, 'w_x');
%plot_results(w_dot_sim(2, :), w_dot_vec(2, :), times, 'w_y');
%plot_results(w_dot_sim(3, :), w_dot_vec(3, :), times, 'w_z');

frames = [1 1000 2000 3000 4000 5000 6000 7000 8000 9000];
% Compute Moment of Inertia matrix
I = pt2D_moi(w_vec2(:,frames), w_dot_vec(:,frames));

% [ q,w,a ] = pt2E_traj(th2(:, end), w_vec2(:, end), w_dot_vec(:, end),...
%                       I, tstep, 1000);
