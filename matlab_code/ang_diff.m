function diff = ang_diff(a, b)
%ANGDIFF return the minimum angular distance between two angles
diff  = a - b; 
diff(diff < -pi) = diff(diff < -pi) + 2*pi;
diff(diff > pi) = diff(diff > pi) - 2*pi;
end

