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
[D,names,units,freq] = mrdplot_convert('d00063-default');
tIndx = findMRDPLOTindex(names, 'time');

alienIndx(1) = findMRDPLOTindex(names, strcat(prefix,'0','x'));
alienIndx(2) = findMRDPLOTindex(names, strcat(prefix,'7','z'));

for j = 1:length(t)
    rowNo = find(D(:,1)==t(j));
    markers(j,2:end) = D(rowNo,alienIndx(1):alienIndx(end));
end

NF = 10; % number of frames (including initial frame)

% set options for fminunc()
% options = optimset();
options = optimset('MaxFunEvals',1000000);

% p0 is the intitial parameter vector
n_p = 6; % com 3, v 3
p0(n_p) = 0;
for i = 1:n_p
 p0(i) = 0;
end

% do optimization
[answer,fval,exitflag]=fminunc(@criterion,p0,options)

