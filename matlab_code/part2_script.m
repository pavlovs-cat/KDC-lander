
clear

% File name to read data from
data_file = 'd00063-default';

% Flag for generating plots
gen_plot = 0;

% 100Hz for 10 seconds
samplingRate = 100;
tau = 1/samplingRate;
init_time = 10;
predict_time = 10;
init_times = tau : tau : init_time;
predict_times = init_time + tau: tau : init_time + predict_time;

Nsim = length(init_times);
N = length(predict_times);

n = {};
for m = 0:7
    n(end+1:end+3) = {sprintf('%dx',m), sprintf('%dy',m),sprintf('%dz',m)};
end

% Exctract marker locations for initial period
pos_vec = extract_data(data_file, 'ml', n, 1:(Nsim+1));

% Generate COMs & location for first 10 seconds
[com, vel] = optim_com();

% Store quaternions for each timestamp
artifact_rot = zeros(Nsim, 4);
Rots = cat(3, eye(3), zeros(3, 3, Nsim));
th2 = horzcat([1 0 0 0]',zeros(4, Nsim));

for j = 1:Nsim
    p = reshape(pos_vec(j, :), 3, 8);
    q = reshape(pos_vec(j+1, :), 3, 8);
    artifact_rot(j,:) = pt2A_helper(p,q);
    th2(:, j+1)  = quatmultiply(artifact_rot(j, :), th2(:, j)')';
end

% Do janky swap
th2 = th2(:, 2:end);
th2 = [th2(1, :); -th2(3, :); th2(2, :); th2(4, :)];
for j = 1:Nsim
   Rots(:, :, j+1) = quat2rotm(th2(:, j)'); 
end
%Rots = Rots(:, :, 2:end);
th_sim =  extract_data(data_file, 'a_q', {'0', '1', '2', '3'}, 1 : Nsim+N)';

% Plot  for orientation
if gen_plot == 1
  plot_results(th_sim(1, 1:Nsim), th2(1, :), init_times, 'q_0');
  plot_results(th_sim(2, 1:Nsim), th2(2, :), init_times, 'q_1');
  plot_results(th_sim(3, 1:Nsim), th2(3, :), init_times, 'q_2');
  plot_results(th_sim(4, 1:Nsim), th2(4, :), init_times, 'q_3');
end

% Extract simulated ang accels and velocities
w_sim = extract_data(data_file, 'a_w', {'x', 'y', 'z'}, 1 : Nsim + N)';

% Compute angular velocities
w_vec2 = pt2B_angv2(Rots, tau);

% Plot velocities for comparison
if gen_plot == 1
  plot_results(w_sim(1, 1:Nsim), w_vec2(1, :), init_times, 'w_x');
  plot_results(w_sim(2, 1:Nsim), w_vec2(2, :), init_times, 'w_y');
  plot_results(w_sim(3, 1:Nsim), w_vec2(3, :), init_times, 'w_z');
end

% Compute angular accelerations
w_dot_vec = pt2C_angacc(w_vec2, tau);

% Compute Moment of Inertia matrixs
I2 = pt2D_moi(w_vec2, w_dot_vec);

I = [0.1739 0 0; 0 0.5931  0; 0 0 0.7861]; 
[ q,w,a ] = pt2E_traj(th2(:, end-1), w_vec2(:, end-1), w_dot_vec(:, end-2),...
                      I, tau, N);
if gen_plot == 1
    plot_results(w_sim(1, Nsim+1:Nsim+N), w(1, :), predict_times, 'w^p_x');
    plot_results(w_sim(2, Nsim+1:Nsim+N), w(2, :), predict_times, 'w^p_x');
    plot_results(w_sim(3, Nsim+1:Nsim+N), w(3, :), predict_times, 'w^p_x');
end

% Uncomment to write txt file again
vel2 = [0.05 0.02 0.01];
all_time = [0; init_times'];
com_traj = horzcat(all_time,zeros(length(all_time),3));
com_traj(1,2:4) = com;
for k = 2:length(all_time)
    com_traj(k,2:4) = com + vel2*all_time(k,1);
end
com_traj = horzcat(com_traj,vertcat([1 0 0 0], th2'));
% dlmwrite('com_traj.txt',com_traj,'delimiter',' ','precision',6)