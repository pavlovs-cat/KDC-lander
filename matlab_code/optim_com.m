function [com, vel] = optim_com()
% Use optimization to find center of mass and velocity

% globals
global markers % marker data array NFx(NM*3)
global NF % number of frames

% load markers
t = 0:1:10;
markers = horzcat(t',zeros(length(t),24));

prefix = 'ml'; % 'm'
alienIndx = zeros(1,2);

% Extracts data into matrix D, modify string to read a particulary binary
[D,names,~,~] = mrdplot_convert('d00063-default');

alienIndx(1) = findMRDPLOTindex(names, strcat(prefix,'0','x'));
alienIndx(2) = findMRDPLOTindex(names, strcat(prefix,'7','z'));

for j = 1:length(t)
    rowNo = find(D(:,1)==t(j));
    markers(j,2:end) = D(rowNo,alienIndx(1):alienIndx(end));
end

NF = 10; % number of frames (including initial frame)

% set options for fminunc()
options = optimset('MaxFunEvals',1000000);

% Set initial guess on the com and velocity
n_p = 6; % com 3, vel 3
p0(n_p) = 0;

% Run optimisation
[answer,~,~] = fminunc(@optim_com_criterion,p0,options);

% Extract return values
com = answer(1:3);
vel = answer(4:6);

end

