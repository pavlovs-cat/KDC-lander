function w_vec = pt2B_angv(Rots, tstep)
% pt2B_angv Compute angular accelerations using finite differences
% Input: Rots  relative rotation matrices
%        tstep time step between frames
% Output w_vec vector of angular velocities

%Initialise variables
Nframes = length(Rots);
w_vec = zeros(3, Nframes);

if Nframes <= 1
    fprintf('Need more than 1 frame. Got %d frames.', Nframes);
    return
end

% Compute axis angles
th = rot2ang(Rots);

% Compute first value using forward difference
w_vec(:,1) = (th(:,2) - th(:,1))/tstep;

% Compute remaining frames using central differences
for idx = 2:Nframes-1
   w_vec(:,idx) = (th(:,idx+1) - th(:, idx-1))/(2*tstep); 
end

% Compute last frame using backward differences
w_vec(:, end) = (th(:,end) - th(:,end-1))/tstep;

end