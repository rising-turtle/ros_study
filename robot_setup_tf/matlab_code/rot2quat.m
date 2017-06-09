function q = rot2quat(R)
%q = rot2quat(R)

q0_sum = R(1,1,:)+R(2,2,:)+R(3,3,:)+1;
q0_sum = max(q0_sum,0); % Can be negative with small numerical errors
q0 = sqrt(q0_sum)/2;
%q0 = sqrt(R(1,1,:)+R(2,2,:)+R(3,3,:)+1)/2;

q1 = (R(3,2,:)-R(2,3,:))/4./q0;
q2 = (R(1,3,:)-R(3,1,:))/4./q0;
q3 = (R(2,1,:)-R(1,2,:))/4./q0;

i = find(q0 <= 0);
if ~isempty(i)
   q1(i) = sqrt(abs(.5 *(R(2,2,i)+R(3,3,i))));
   q2(i) = sqrt(abs(.5 *(R(1,1,i)+R(3,3,i))));
   q3(i) = sqrt(abs(.5 *(R(1,1,i)+R(2,2,i))));
   
   j = i(find(q1(i) ~= 0));
   if ~isempty(j)
      q2(j) = q2(j) .* sign(R(1,2,j));
      q3(j) = q3(j) .* sign(R(1,3,j));
   end
   
   j = i(find(q1(i) == 0));
   if ~isempty(j)
      q3(j) = q3(j) .* sign(R(2,3,j));
   end
end
q = [q0(:)'; q1(:)'; q2(:)'; q3(:)'];