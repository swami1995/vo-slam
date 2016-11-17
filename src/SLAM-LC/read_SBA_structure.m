%% Description
%   This file reads the set of optimised 3d points returned by the SBA
%   package and constucts a ply file using these points.
fileID = fopen('SBA_struct.txt');
a = fscanf(fileID,'%f');
pts_3d_r = zeros(size(a)/3,3);
for l=1:size(a)/3
        pts_3d_r(l,1)=a((l-1)*3 + 1);
        pts_3d_r(l,2)=a((l-1)*3 + 2);
        pts_3d_r(l,3)=a((l-1)*3 + 3);
   
end
% scatter3(pts_3d_r(:,1),pts_3d_r(:,2),pts_3d_r(:,3));
% axis equal
ptCloud = pointCloud(pts_3d_r,'Color',repmat(uint8([0 0 255]),size(pts_3d_r,1),1));
pcwrite(ptCloud,'ptCloud_struct.ply');
fclose(fileID);    