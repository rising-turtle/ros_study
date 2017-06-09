function [rot, trans, state] = find_transform_matrix_dr_ye(pset1, pset2)

% This function find rotation matrix (and Euler angles) and translation matrix of two point sets.
% state=0: no solution
% state=1: desired solution found.
% state=2: desired solution found, points are co-planar.

H=[0 0 0; 0 0 0; 0 0 0];    
tmp=size(pset2);    
pnum=tmp(2);
ct1 = sum(pset1,2)/pnum; %average x,y,z values for all interesting features img1 
ct2 = sum(pset2,2)/pnum; %average x,y,z values for all interesting features img2
for i=1:pnum
	q1=pset1(:,i)-ct1; %spread of xyz values in img1 
	q2=pset2(:,i)-ct2; %spread of xyz values in img2    
	Q21=q2*q1';
	H = H+Q21;	%H will have higher values if spreads are higher
end

[U, S, V] = svd(H);         
sv = [abs(S(1,1)), abs(S(2,2)), abs(S(3,3))]; 
Xq = V * U';
mdet = det(Xq);
threshold = 0.00000000001;

if round(mdet) == 1
    rot = Xq;
    trans = ct1 - rot*ct2;
    state = 1;
elseif round(mdet) == -1
    zn = find(sv<threshold);
    zs = size(zn);
    if zs(2) == 1
        V(:, zn)=-V(:, zn);
        rot = V * U';
        trans = ct1 - rot*ct2;
        state = 2;
    else
        state = -1;
        rot=H;
        trans=[0 0 0]';
    end
else
    state = 0;
    rot=H;
    trans=[0 0 0]';
end

