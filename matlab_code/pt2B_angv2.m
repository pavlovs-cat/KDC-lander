function w_vec = pt2B_angv2(Rots, tstep)
% pt2B_angv Compute angular accelerations using finite differences
% Input: Rots  relative rotation matrices
%        tstep time step between frames
% Output w_vec vector of angular velocities

%Initialise variables
Nframes = length(Rots) - 1;
w_vec = zeros(3, Nframes);

if Nframes <= 1
    fprintf('Need more than 1 frame. Got %d frames.', Nframes);
    return
end
for idx = 1:Nframes
    S = (( Rots(:, :, idx+1)' - Rots(:, :, idx)' )/tstep) * Rots(:, :, idx);
    w_vec(:, idx) = [S(2,3); S(3,1); S(1, 2)]; 
end
end