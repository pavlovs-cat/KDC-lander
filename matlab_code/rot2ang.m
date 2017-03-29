function th  = rot2ang( Rots )
%rot2ang Converts a list of rotation matrices to a list of rotations around
%        the three principle axes. 
%        
%        Rots  is the list of rotation matrices
%        th    is the list of rotation angles

% Number of input rotation matrices
Nframes = length(Rots);

% Initialise output vector
th = zeros(3, Nframes);

% Compute angular rotations
for idx = 1:Nframes
    R = Rots(:,:,idx);
    th(1, idx) = atan2(R(3,2), R(3,3));
    th(2, idx) = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
    th(3, idx) = atan2(R(2,1), R(1,1));
end 

end

