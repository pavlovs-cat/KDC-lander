function plot_results(simulated, approximated, times, name)
%plot_results Plots the simulated and approximated results for given times
%      simulated Results generated in simulation
%      approximated Results generated from simulated measurements
%      times vector of time points
%      name base name for plot title and y axis

% Plot the simulation results
figure;
hold on;
size(times)
size(simulated)
size(approximated)
plot(times, simulated, 'r-')
plot(times, approximated, 'b-')
legend('Simulated', 'Approximated')
xlabel('time (s)')
ylabel(name);
title({sprintf('%s vs time', name)}, 'interpreter', 'Latex');

end