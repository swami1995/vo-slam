function writetofile(ftr_3d,ftr_2d,num_traces,ftr,num_frames,T_w,K)
%% Description
% This function creates 3 files as per the requirements of SBA.
% 1) SBA_pts - containing the 3d points and the corresponding 2d points 
% 2) SBA_motion_ - containing the 3d quaternions of the rotations and the corresponding translations of the camera
% 3) SBA_calib - containing the precalibrated intrinsic matrix of the camera
% visit https://github.com/balintfodor/sba/tree/master/demo for further details regarding the files.
% 
%   ftr_3d - the list of 3d points in the world coordinte frame 
%   ftr_2d - each row corresponds to a single 3d point and contains the 
%            list of 2d projections the 3d point makes on the respective 
%            frames that it is visible in
%   ftr - each row corresponds to a single 3d point and lists the frames
%         that view the corresponding 3d point
%   num_traces - The number traces, i.e, the total number of distinct 3d
%                points
%   num_frames - The frame at which we want the trajectory to end
%   T_w - camera trajectories in world coordinate frame


%%
fileID = fopen('SBA_pts.txt','w');
for l=1:num_traces
    fprintf(fileID,'%f %f %f %d',ftr_3d(1,l),ftr_3d(2,l),ftr_3d(3,l),sum(ftr(l,:)>0));
    for j=find(ftr(l,:))
         fprintf(fileID,' %d %f %f',j-1,ftr_2d(1,l,j),ftr_2d(2,l,j));
    end
    fprintf(fileID,'\n');
end
fclose(fileID);
fileID = fopen('SBA_motion_.txt','w');
for l=1:num_frames
    q_vec = qGetQ(T_w(1:3,1:3,l));
    fprintf(fileID,'%f %f %f %f %f %f %f\n',q_vec(1),q_vec(2),q_vec(3),q_vec(4),T_w(1,4,l),T_w(2,4,l),T_w(3,4,l));
end
fclose(fileID);
K1=K/0.4;
fileID = fopen('SBA_calib.txt','w');
for l=1:3
        fprintf(fileID,'%f\t%f\t%f\n',K1(l,1),K1(l,2),K1(l,3));
end
fclose(fileID); 
end
