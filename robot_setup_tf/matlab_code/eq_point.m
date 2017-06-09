%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% this function is from icp function 

function [R,T] = eq_point(q,p,imu_rot,weights)


if nargin <= 3
    weights = ones(1,size(q,2));
end

m = size(p,2);
n = size(q,2);

% normalize weights
weights = weights ./ sum(weights);

% find data centroid and deviations from centroid
q_bar = q * transpose(weights);
%%%q_mark = q - repmat(q_bar, 1, n);
% Apply weights
%%%q_mark = q_mark .* repmat(weights, 3, 1);

% find data centroid and deviations from centroid
p_bar = p * transpose(weights);
%%%p_mark = p - repmat(p_bar, 1, m); %mod
% Apply weights
%p_mark = p_mark .* repmat(weights, 3, 1);

%%%N = p_mark*transpose(q_mark); % taking points of q in matched order %mod

%%%[U,~,V] = svd(N); % singular value decomposition %mod

%%%R = V*diag([1 1 det(U*V')])*transpose(U); %mod

R=imu_rot;

T = q_bar - R*p_bar;

end
