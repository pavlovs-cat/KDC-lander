function [ quaternion, varargout] = pt2A_helper( p, q )
% Lauren Lieu

% INSTRUCTIONS: SVD algorithm for rigid transforms.
% This function uses an example set of p and q 3D coordinates with n = 8.
% Modify the 3D coordinates of p and q to test different rigid body point
% cloud transformations.

% p is the initial frame
% q is the final frame 

% p = [0 2 0; 1 1 0; 1 -5 0; -1 -5 0; -1 1 0; -2 -6 -5; 1 2 3; 0 8 -10]';
% q = [0 -2 5; -1 -1 5; -1 5 5; 1 5 5; 1 -1 5; 2 6 0; -1 -2 8; 0 -8 -5]';

[~, n] = size(p);

% Find the centroids of each set of n points on the rigid body
p_avg = mean(p,2);
q_avg = mean(q,2);

% Initialize the covariance matrix H
H = zeros(3);

for i = 1:n
    % Calculate the covariance matrix for the 3D point clouds centered
    % about the origin.
    p_hat = p(:,i) - p_avg;
    q_hat = q(:,i) - q_avg;
    H = H + p_hat*q_hat';
end

[U, ~, V] = svd(H);
R = V*U';
t = q_avg - R*p_avg;    % translation

% Applying the transformations to input p to compare the resulting
% coordinates with the given q.
transformedP = R*p + repmat(t,1,n);
% q
diff = transformedP - q;     % reprojection error

quaternion = rotm2quat(R);
if nargout == 2
    varargout{1} = R;
end
end

