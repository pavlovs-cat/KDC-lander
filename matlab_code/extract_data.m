function vec = extract_data(data_file, prefix, n, t_idx_vec)
%EXTRACT_DATA Extracts data from a git binary data file using MRD
%   prefix is the id of the data in the file
%   ids    are the sub ids for the prefix
%   t_idx  times at which to extract data

N = length(n);

vec = zeros(length(t_idx_vec), N);

% Extracts data into matrix D, modify string to read a particulary binary
[D,names,~,~] = mrdplot_convert(data_file);

data_idx = zeros(1, N);
for idx = 1:length(n)
    var_name = strcat(prefix, char(n(idx)));
    data_idx(idx) = findMRDPLOTindex(names, var_name);
end

for idx = 1:N
    for j = t_idx_vec
        vec(j,idx) = D(j, data_idx(idx));
    end
end
end