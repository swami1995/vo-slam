close all;
clear;

load seq01.mat;

files = dir('data/*.jpeg');

worl = [eye(3) zeros(3,1)];
forw = [eye(3) zeros(3,1); 0 0 0 1];
back = [eye(3) zeros(3,1); 0 0 0 1];
h = figure;
subplot(1,2,2);
hold on;
plot3([worl(1,4) worl(1,1)],[worl(2,4) worl(2,1)],[worl(3,4) worl(3,1)],'r-');
plot3([worl(1,4) worl(1,2)],[worl(2,4) worl(2,2)],[worl(3,4) worl(3,2)],'g-');
plot3([worl(1,4) worl(1,3)],[worl(2,4) worl(2,3)],[worl(3,4) worl(3,3)],'b-');
grid on;
axis equal;
drawnow;
for i=2:length(ran)
    imgL = imread(sprintf('data/%s',files(ran(i-1)).name));
    imgLc = imresize(imgL,scale);   
    
    kpL = keyp{i-1,1};
    kpR = keyp{i,1};
    inL = keyp{i-1,3}(1,:);
    inR = keyp{i-1,3}(2,:);
    
    pts3d = keyp{i-1,4};
    
    R = pose{i,2};
    t = pose{i,3};
    singvals = pose{i,4};
    
    figure(h);
    subplot(1,2,1);
    hold off;
    imshow(imgLc);
    hold on;
    quiver(kpL(1,inL),kpL(2,inL),kpR(1,inR)-kpL(1,inL),kpR(2,inR)-kpL(2,inL));
    title(sprintf('%d-%d',ran(i-1),ran(i)));

    if singvals(1)>10
        forw = [R t; 0 0 0 1]*forw;
        back = inv([R t; 0 0 0 1])*back;

        cam = makeinhomogeneous(back*makehomogeneous(worl));
        pts3d = makeinhomogeneous(forw*makehomogeneous(pts3d));        
    
        subplot(1,2,2);
        plot3([cam(1,4) cam(1,1)],[cam(2,4) cam(2,1)],[cam(3,4) cam(3,1)],'r-');
        plot3([cam(1,4) cam(1,2)],[cam(2,4) cam(2,2)],[cam(3,4) cam(3,2)],'g-');
        plot3([cam(1,4) cam(1,3)],[cam(2,4) cam(2,3)],[cam(3,4) cam(3,3)],'b-');
        title('');
    else
        subplot(1,2,2);
        title('NO MOTION');
    end
    
    drawnow;    
end