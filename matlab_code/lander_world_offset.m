function [quat, R] = lander_world_offset()
% Calculates the rotation for world -> lander frame
% Returns the quaternion and rotation matrix that represent this transform

% load markers
t = 0:1:10;
markers = horzcat(t',zeros(length(t),24));

prefix_l = 'ml'; % 'm'
alienIndx_l = zeros(1,2);

% Extracts data into matrix D, modify string to read a particulary binary
[D,names,~,~] = mrdplot_convert('d00063-default');

alienIndx_l(1) = findMRDPLOTindex(names, strcat(prefix_l,'0','x'));
alienIndx_l(2) = findMRDPLOTindex(names, strcat(prefix_l,'7','z'));

prefix_w = 'm';
alienIndx_w = zeros(1,2);
alienIndx_w(1) = findMRDPLOTindex(names, strcat(prefix_w,'0','x'));
alienIndx_w(2) = findMRDPLOTindex(names, strcat(prefix_w,'7','z'));

markers_l = D(1,alienIndx_l(1):alienIndx_l(end));
markers_w = D(1,alienIndx_w(1):alienIndx_w(end));

markers_l = reshape(markers_l,3,8);
markers_w = reshape(markers_w,3,8);

% Swap markers_w and markers_l to flip and get the transform 
% for lander -> world
[quat, R] = pt2A_helper(markers_w, markers_l);

end