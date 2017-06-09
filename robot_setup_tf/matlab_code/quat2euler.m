function eulers = quat2euler(q)
%Covert a Quaternion to Euler Angles. All output range issues handled including gimbal lock situation.

%test = q(2)*q(3) + q(4)*q(1);
%if test > 0.499 %singularity at north pole
%	theta = 2 * atan2(q(2),q(1));
%	psii = pi/2.0;
%	phi = 0;
%elseif test < -0.499 %singularity at south pole
%	theta = -2 * atan2(q(2),q(1));
%	psii = -pi/2.0;
%	phi = 0;
%else
%	theta=atan2(2*q(3)*q(1)-2*q(2)*q(4),1-2*q(3)^2-2*q(4)^2); % heading
%	psii=asin(2*q(2)*q(3)+2*q(4)*q(1)); % attitude
%	phi=atan2(2*q(2)*q(1)-2*q(3)*q(4),1-2*q(2)^2-2*q(4)^2); %bank
%end

%eulers = [phi;theta;psii];


% Takes quaternions and calculates the equivalent Euler angles
% Inputs
% [q0,q1,q2,q3] = quaternions with q0 being the "scalar" value
%
% Outputs
% bank,pitch,azimuth = Euler angles (rad) in 1,2,and 3 axis

m11 = 2.*(q(2).*q(3) + q(1).*q(4));
m12 = q(1).^2 + q(2).^2 - q(3).^2 - q(4).^2;
m21 = -2.*(q(2).*q(4) - q(1).*q(3));
m31 = 2.*(q(3).*q(4) + q(1).*q(2));
m32 = q(1).^2 - q(2).^2 - q(3).^2 + q(4).^2;

bank = atan2(m31,m32);
pitch = asin(m21);
azimuth = atan2(m11,m12);

eulers=[bank;pitch;azimuth];
return
