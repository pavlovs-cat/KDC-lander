function w_dot_vec = pt2C_angacc(w, tstep)
% pt2B_angacc Compute angular accelerations using finite differences
% Input: Rots  relative rotation matrices
%        tstep time step between frames
% Output w_dot_vec vector of angular accelerations

%Initialise variables
Nframes = length(w);
w_dot_vec = zeros(3, Nframes);

if Nframes <= 1
    fprintf('Need more than 1 frame. Got %d frames.', Nframes);
    return
end

% Compute derivatives using forward differences
for idx = 1:Nframes-1
   w_dot_vec(:,idx) = (w(:, idx+1) - w(:, idx))/tstep; 
end

end

