%% Description
%   This file reads the set of optimised camera trajectories returned by 
%   the SBA package and constucts a ply file using these points.
%   effNumFrames - Total number frames sampled
load('effNumFrames.mat')
num_frame1=effNumFrames-2;
num_frame2=1;
pose1 = [0;0;0];
hold on
fileID = fopen('SBA_motion.txt');
a = fscanf(fileID,'%f');
q_vec = zeros(1,4);
T_r = zeros(4,4,num_frame1);
trans = zeros(num_frame1,3);
for l=num_frame2:num_frame1
    q_vec(1,1)=a((l-1)*7 + 1);
    q_vec(1,2)=a((l-1)*7 + 2);
    q_vec(1,3)=a((l-1)*7 + 3);
    q_vec(1,4)=a((l-1)*7 + 4);
    trans(l,1)=a((l-1)*7 + 5);
    trans(l,2)=a((l-1)*7 + 6);
    trans(l,3)=a((l-1)*7 + 7);
    R_r = qGetR(q_vec);
%     if l==1
%         R_1=R_r
%     end
%     norm(trans)
%     R_r=R_r/R_1;
%     trans(l,:)=(trans(l,:)'+R_r*trans(1,:)')';
    T_r(:,:,l)=[R_r trans(l,:)';0 0 0 1];
    pose_r=R_r*pose1+trans(l,:)';
    scatter3(pose_r(1),pose_r(2),pose_r(3),'b','.');
    pause(0.05);
end
ptCloud = pointCloud( trans,'Color',repmat(uint8([0 255 0]),size(trans,1),1));
pcwrite(ptCloud,'ptCloud_motion.ply');
fclose(fileID);    