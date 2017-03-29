function w_dot_vec = pt2B_angacc(Rots, tstep)
% pt2B_angacc Compute angular accelerations using finite differences
% Input: Rots  relative rotation matrices
%        tstep time step between frames
% Output w_dot_vec vector of angular accelerations

%Initialise variables
Nframes = length(Rots);
w_dot_vec = zeros(3, Nframes);

if Nframes <= 1
    fprintf('Need more than 1 frame. Got %d frames.', Nframes);
    return
end

% Compute axis angles
th = rot2ang(Rots);

% Compute squared time step
t_sq = tstep^2;

% Compute first value using forward difference
w_dot_vec(:,1) = (th(:,3) - 2*th(:,2) + th(:,1))/t_sq;

% Compute remaining frames using central differences
for idx = 2:Nframes-1
   w_dot_vec(:,idx) = (th(:, idx+1) - 2*th(:, idx) + th(:, idx))/t_sq; 
end

% Compute last frame using backward differences
w_dot_vec(:, end) = (th(:,end) - 2*th(:,end-1) + th(:,end-2))/t_sq;

end

