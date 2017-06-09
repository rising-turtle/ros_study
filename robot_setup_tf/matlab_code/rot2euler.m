% rot2euler.m
%
%      usage: rot2euler(rotationMatrix)
%    purpose: get euler angles in radians from rotation matrix
% see http://nghiaho.com/?page_id=846


function eulers = rot2euler(R)
	x = atan2(R(3,2), R(3,3));
	y = atan2(-R(3,1), sqrt(R(3,2)*R(3,2) + R(3,3)*R(3,3)));
	z = atan2(R(2,1), R(1,1));
        eulers=[x y z];
end
