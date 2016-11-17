fileID = fopen('map_points.txt','r');
formatspec = '%f';
A = fscanf(fileID,formatspec);
B= reshape(A,4,size(A,1)/4)';
plot3(B(:,1),B(:,2),B(:,3),'.');