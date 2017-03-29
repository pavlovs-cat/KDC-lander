function vec = extract_data(data_file, prefix, n, t)
%EXTRACT_DATA Extracts data from a git binary data file using MRD
%   prefix is the id of the data in the file
%   ids    are the sub ids for the prefix
%   t      times at which to extract data

vec = horzcat(t',zeros(length(t),24));

% Extracts data into matrix D, modify string to read a particulary binary
[D,names,~,~] = mrdplot_convert(data_file);

alienIndx = zeros(1,length(n));
for idx = 1:length(n)
    var_name = strcat(prefix,n(idx));
    alienIndx(idx) = findMRDPLOTindex(names, var_name);
end

for j = 1:length(t)
    rowNo = find(D(:,1)==t(j));
    vec(j,2:end) = D(rowNo,alienIndx(1):alienIndx(end));
end

end