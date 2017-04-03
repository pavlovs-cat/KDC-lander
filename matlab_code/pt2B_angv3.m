function w_vec = pt2B_angv3(q, tstep)
% pt2B_angv Compute angular accelerations using finite differences
% Input: Rots  relative rotation matrices
%        tstep time step between frames
% Output w_vec vector of angular velocities

%Initialise variables
Nframes = length(q);
w_vec = zeros(3, Nframes);

if Nframes <= 1
    fprintf('Need more than 1 frame. Got %d frames.', Nframes);
    return
end

for idx = 1:(Nframes-1)
    q0 = q(:, idx);
    q1 = q(:, idx+1);
    qn = norm(q0);
    q0c = [q0(1) -q0(2) -q0(3) -q0(4)]/qn;
    r = quatmultiply(q1', q0c);
    th = 2*acos(r(1));
    if th > pi
        th = th - 2*pi;
    end
    n = norm(r);
    w_vec(:, idx) = th*r(1:3)/(tstep*n); 
end
end