
clear

% Extracts data into matrix D, modify string to read a particulary binary
[D,names,units,freq] = mrdplot_convert('d00063-default');

% Uses 8 markers on alien/artifact
% alienIndx = zeros(3,8);
alienIndx = zeros(1,2);

% Prefix of alien/artifact - Which reference frame to use?? m, ml, mn
prefix = 'ml'; % 'm'

% 100Hz for 10 seconds
samplingRate = 100;
timespan = 10;
% store quaternions for each timestamp
artifact_rot = zeros((samplingRate*timespan)-1,4);

% for i = 1:8
%     % can simplify since (0x, 0y, 0z) to (7x, 7y, 7z) is sequential
%     alienIndx(1,i) = findMRDPLOTindex(names, strcat(prefix,int2str(i-1),'x'));
%     alienIndx(2,i) = findMRDPLOTindex(names, strcat(prefix,int2str(i-1),'y'));
%     alienIndx(3,i) = findMRDPLOTindex(names, strcat(prefix,int2str(i-1),'z'));
% end

alienIndx(1) = findMRDPLOTindex(names, strcat(prefix,'0','x'));
alienIndx(2) = findMRDPLOTindex(names, strcat(prefix,'7','z'));

for j = 2:(samplingRate*timespan)
    p = reshape(D(j-1,alienIndx(1):alienIndx(end)),3,8);
    q = reshape(D(j,alienIndx(1):alienIndx(end)),3,8);
    artifact_rot(j-1,:) = pt2A_helper(p,q);
end