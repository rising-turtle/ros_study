%% Euler2rot: given Euler angles (x,y,z) make R
%=========================================================================
function R = euler2rot(vals)

    % X-axis
    a = vals(1);
    Rx = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
    
    % Y-axis
    a = vals(2);
    Ry = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
    
    % Z-axis
    a = vals(3);
    Rz = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
    
    % Combine: what order should we use?
    R = Rz * Ry * Rx;%Changed last term to Rx from Rz
    
end
