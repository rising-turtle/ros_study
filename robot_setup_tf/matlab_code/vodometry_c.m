function [rot, trans, psn, op_pset1, op_pset2, GoodFrames1, GoodFrames2, GoodDescriptor1, GoodDescriptor2, SolutionState] = vodometry_c(file1,file2)
global myCONFIG

nMatches=0;
nIterationRansac=0;
InlierRatio=0;
nSupport=0;
ErrorMean=0;
ErrorVariance=0;
psn=0;

[x1,y1,z1,confidence_map1,img1]=read_sr4000_data_dr_ye(file1);  %%% in SR4000 coordinate

[frm1, des1] = sift(img1);
nFeatures1 = size(frm1,2);

frm1(1,:) = frm1(1,:)+1; %no need to convert to (1,1) reference frame in C++ implementation
frm1(2,:) = frm1(2,:)+1;

if 1 %this is set to 1 in config file
    [frm1, des1] = confidence_filtering(frm1, des1,confidence_map1);
end

RawFrames1 = frm1;
RawDescriptor1 = des1;

[x2,y2,z2,confidence_map2,img2]=read_sr4000_data_dr_ye(file2);

[frm2, des2] = sift(img2);
nFeatures2 = size(frm2,2);

frm2(1,:) = frm2(1,:)+1;
frm2(2,:) = frm2(2,:)+1;

if 1
    [frm2, des2] = confidence_filtering(frm2, des2,confidence_map2);
end

RawFrames2 = frm2;
RawDescriptor2 = des2;

match = siftmatch(des1, des2);
nMatches = size(match,2);

%find the matched two point sets.
%match = [4 6 21 18; 3 7 19 21];

epsilon =0.01;
maxCNUM = 0;
nSetHypothesisGenerator = 4;
pnum = size(match, 2); % equal to RANSAC_STAT.nMatches

if pnum<4
    fprintf('too few sift points for ransac.\n');
    trans=0.0;
    SolutionState = 4;
    return;
else
    rst = min(700, nchoosek(pnum, 4));
    tmp_nmatch=zeros(2, pnum, rst);
    nIterations = rst;
    
    for i=1:min(rst,nIterations)
        tic;
        [n_match, rs_match, cnum] = ransac_dr_ye(frm1, frm2, match, x1, y1, z1, x2, y2, z2); %kp1(2xn),kp2(2xn),match(2xpnum),x1(144x176),y1(144x176),z1(144x176),x2(144x176),y2(144x176),z2(144x176)
        
        for k=1:cnum
            tmp_nmatch(:,k,i) = n_match(:,k); %copy all matches from ransac to a list
        end
        tmp_rsmatch(:, :, i) = rs_match;      
        tmp_cnum(i) = cnum;

        if cnum > maxCNUM
            maxCNUM = cnum;
            nIterations = 5*ceil(log(epsilon) / log( (1-(maxCNUM/pnum)^nSetHypothesisGenerator) ) );
        end
    end
  
    [rs_max, rs_ind] = max(tmp_cnum);
    
    op_num = tmp_cnum(rs_ind);
    if(op_num<3)
        fprintf('no consensus found, ransac fails.\n');
        SolutionState = 4;
        return;
    end
    for k=1:op_num
        op_match(:, k) = tmp_nmatch(:, k, rs_ind); %optimized match via ransac
    end
end

for i=1:op_num
    frm1_index=op_match(1, i);      frm2_index=op_match(2, i);
    matched_pix1=frm1(:, frm1_index);     COL1=round(matched_pix1(1));     ROW1=round(matched_pix1(2));
    matched_pix2=frm2(:, frm2_index);     COL2=round(matched_pix2(1));     ROW2=round(matched_pix2(2));
    op_pset1(1,i)=-x1(ROW1, COL1);   op_pset1(2,i)=-y1(ROW1, COL1);   op_pset1(3,i)=z1(ROW1, COL1);
    op_pset2(1,i)=-x2(ROW2, COL2);   op_pset2(2,i)=-y2(ROW2, COL2);   op_pset2(3,i)=z2(ROW2, COL2);
end

psn=op_num;

[rot, trans, sta] = find_transform_matrix_dr_ye(op_pset1, op_pset2);
ErrorRANSAC = rot*op_pset2+repmat(trans,1,size(op_pset2,2))-op_pset1;
ErrorRANSAC_Norm = sqrt(ErrorRANSAC(1,:).^2+ErrorRANSAC(2,:).^2+ErrorRANSAC(3,:).^2);
ErrorMean = mean(ErrorRANSAC_Norm);
ErrorStd = std(ErrorRANSAC_Norm);
nIterationRansac = min(rst,nIterations);
nSupport = size(op_pset1,2);
ErrorMean = ErrorMean;
ErrorStd = ErrorStd;
SolutionState = sta;
GoodFrames1 = frm1(:, op_match(1, :));
GoodDescriptor1 = des1(:, op_match(1, :)); %128xopnum
GoodFrames2 = frm2(:, op_match(2, :));
GoodDescriptor2 = des2(:, op_match(2, :)); %128xopnum

if nMatches~=0
   InlierRatio = (nSupport/ nMatches)*100;
end

if SolutionState < 1
    trans=[0.0;0.0;0.0];
    rot=eye(3);
else
    %return op_pset1;
    %return op_pset2;
    %return RANSAC_STAT;
end

if SolutionState == -1
    disp('RANSAC FAILED: STOPPING EXECUTION')
end