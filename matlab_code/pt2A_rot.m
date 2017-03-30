
clear

% File name to read data from
data_file = 'd00063-default';


% 100Hz for 10 seconds
samplingRate = 100;
tau = 1/samplingRate;
timespan = 10;
times = tau : tau : timespan - tau;

n = {};
for m = 0:7
    n(end+1:end+3) = {sprintf('%dx',m), sprintf('%dy',m),sprintf('%dz',m)};
end
N = length(times)-1;
pos_vec = extract_data(data_file, 'ml', n, 1:(N+1));

% store quaternions for each timestamp
artifact_rot = zeros(N,4);
Rots = zeros(3, 3, N+1);
Rots(:, :, 1) = eye(3);

for j = 2:N+1
    p = reshape(pos_vec(j-1, :),3,8);
    q = reshape(pos_vec(j, :),3,8);
    [artifact_rot(j-1,:), R] = pt2A_helper(p,q);
    Rots(:,:,j) = R*Rots(:,:,j-1);
end

Rots = Rots(:, :, 2:end);

% Generate COMs
[com, vel] = optim_com();

% Extract simulated ang accels and velocities
w_sim = extract_data(data_file, 'a_w', {'x', 'y', 'z'}, 1:N)';

% Convert angv to assignment frame
for idx = 1 : N
    w_sim(:, idx) = Rots(:, :, idx)*w_sim(:, idx);
end

% Compute angular velocities
w_vec = pt2B_angv(Rots, tau);

% Plot velocities for comparison
plot_results(w_sim(1, :), w_vec(1, :), times(1:N), 'w_x');
plot_results(w_sim(2, :), w_vec(2, :), times(1:N), 'w_y');
plot_results(w_sim(3, :), w_vec(3, :), times(1:N), 'w_z');

% Compute angular accelerations
w_dot_vec = pt2B_angv(Rots, tau);

% Plot accelerations for comparison
% plot_results(w_dot_sim(1, :), w_dot_vec(1, :), times, 'w_x');
% plot_results(w_dot_sim(2, :), w_dot_vec(2, :), times, 'w_y');
% plot_results(w_dot_sim(3, :), w_dot_vec(3, :), times, 'w_z');

% Compute Moment of Inertia matrix
I = pt2D_moi(w_vec, w_dot_vec);
