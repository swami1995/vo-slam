fileID = fopen('camera_pos.txt','r');
formatspec = '%f';
A = fscanf(fileID,formatspec);
B= reshape(A,3,size(A,1)/3)';
plot3(B(:,1),B(:,2),B(:,3));