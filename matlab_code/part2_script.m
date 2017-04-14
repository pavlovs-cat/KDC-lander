
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

% COM velocity in the world frame, checked using mrdplot
% vel_w = [0.05 0.02 0.01];

% Calculates the rotation transform for
% [world frame] --> [lander/assignment frame]
% Use quat & R to check values with simulation data given by mrdplot
[quat, R] = lander_world_offset();

all_time = [0; init_times'];
com_pos = horzcat(all_time,zeros(length(all_time),3));
com_pos(1,2:4) = com;
for k = 2:length(all_time)
    com_pos(k,2:4) = com + vel*all_time(k,1);
end

% Store quaternions for each timestamp
Rots = cat(3, eye(3), zeros(3, 3, Nsim));
th = horzcat([1 0 0 0]',zeros(4, Nsim));

for j = 1:Nsim
    p = reshape(pos_vec(j, :), 3, 8);
    q = reshape(pos_vec(j+1, :), 3, 8);
    rot = pt2A_helper(p,q);
    th(:, j+1) = quatmultiply(rot', th(:, j)')';
end

% Do janky swap
th = th(:, 2:end);
th = [th(1, :); -th(3, :); th(2, :); th(4, :)];
for j = 1:Nsim
   Rots(:, :, j+1) = quat2rotm(th(:, j)'); 
end
% Rots = Rots(:, :, 2:end);
th_sim =  extract_data(data_file, 'a_q', {'0', '1', '2', '3'}, 1 : Nsim+N)';

% Plot  for orientation
if gen_plot == 1
  plot_results(th_sim(1, 1:Nsim), th(1, :), init_times, 'q_0');
  plot_results(th_sim(2, 1:Nsim), th(2, :), init_times, 'q_1');
  plot_results(th_sim(3, 1:Nsim), th(3, :), init_times, 'q_2');
  plot_results(th_sim(4, 1:Nsim), th(4, :), init_times, 'q_3');
end

% Extract simulated ang accels and velocities
w_sim = extract_data(data_file, 'a_w', {'x', 'y', 'z'}, 1 : Nsim + N)';

% Compute angular velocities
w_vec = pt2B_angv2(Rots, tau);

% Plot velocities for comparison
if gen_plot == 1
  plot_results(w_sim(1, 1:Nsim), w_vec(1, :), init_times, 'w_x');
  plot_results(w_sim(2, 1:Nsim), w_vec(2, :), init_times, 'w_y');
  plot_results(w_sim(3, 1:Nsim), w_vec(3, :), init_times, 'w_z');
end

% Compute angular accelerations
w_dot_vec = pt2C_angacc(w_vec, tau);

% Compute Moment of Inertia matrixs
I = pt2D_moi(w_vec, w_dot_vec);

% I = [0.1739 0 0; 0 0.5931  0; 0 0 0.7861]; % In WORLD frame
[ q,w,a ] = pt2E_traj(th(:, end-1), w_vec(:, end-1), w_dot_vec(:, end-2),...
                      I, tau, N);
if gen_plot == 1
    plot_results(w_sim(1, Nsim+1:Nsim+N), w(1, :), predict_times, 'w^p_x');
    plot_results(w_sim(2, Nsim+1:Nsim+N), w(2, :), predict_times, 'w^p_x');
    plot_results(w_sim(3, Nsim+1:Nsim+N), w(3, :), predict_times, 'w^p_x');
end

% Uncomment to write txt file again
% Rotate values to assignment frame
th_assign = th;
w_vec_assign = R*w_vec;
w_dot_vec_assign = R*w_dot_vec;
quat_r = rotm2quat(R);
for idx = 1 : size(th, 2)
   th_assign(:, idx) = quatmultiply( quat_r',  th(:, idx)' )';
end

com_pos_world = horzcat(com_pos(:, 1), com_pos(:, 2:4)*R);
com_traj = horzcat(com_pos_world,vertcat([1 0 0 0], th'));
dlmwrite('problem2_3.dat',com_traj,'delimiter',' ','precision',6)