function [R,t,pts3d,singvals] = getpose(F,K,ptsL,ptsR)
% The (R,t) obtained in this way transforms the second coordinate system to
% the first coordinate system (I,0).

% The (R,t) can also be applied on the 3D points expresed in the first
% coordinate system such that they can be referred to in the local
% coordinate system of the second frame.

% To transform the first coordinate system to the second coordinate system
% such that the motion of the camera can be visualised, use inv(R,t).

E = K'*F*K;
% ptsL(1,:) = ptsL(1,:)-1003.35296*0.4;
% ptsL(2,:) = 0.4*500.79178 - ptsL(2,:);
% ptsR(1,:) = ptsR(1,:)-1003.35296*0.4;
% ptsR(2,:) = 0.4*500.79178 - ptsR(2,:);
[U,S,V] = svd(E);
singvals = diag(S)';
S(2,2) = S(1,1);
S(3,3) = 0;

E = U*S*V';

[U,~,V] = svd(E);

W = [0 -1 0;1 0 0;0 0 1];
Wi = [0 1 0;-1 0 0;0 0 1];

t1 = U(:,3);
t2 = -t1;    
R1 = U*inv(W)*V';
R2 = U*inv(Wi)*V';
if det(R1)<0
    R1 = -R1;
    R2 = -R2;
end

nz = zeros(1,4);

% h = figure;
for i=1:4
    
    PL = K*[eye(3) zeros(3,1)];
    switch i
        case 1
            R = R1;
            t = t1;
        case 2
            R = R1;
            t = t2;
        case 3
            R = R2;
            t = t1;
        case 4
            R = R2;
            t = t2;
    end
    PR = K*[R t];    
    pts3d = triangulate(PL,PR,ptsL,ptsR);
    
    % % Camera 1.
    % axis1 = [eye(3) zeros(3,1)];
    % 
    % % Camera 2.
    % axis2 = inv([R t; 0 0 0 1])*[axis1;1 1 1 1];
    % axis2 = makeinhomogeneous(axis2);
    % 
    % clf(h);
    % hold on;
    % plot3([axis1(1,4) axis1(1,1)],[axis1(2,4) axis1(2,1)],[axis1(3,4) axis1(3,1)],'r-');
    % plot3([axis1(1,4) axis1(1,2)],[axis1(2,4) axis1(2,2)],[axis1(3,4) axis1(3,2)],'g-');
    % plot3([axis1(1,4) axis1(1,3)],[axis1(2,4) axis1(2,3)],[axis1(3,4) axis1(3,3)],'b-');
    % plot3([axis2(1,4) axis2(1,1)],[axis2(2,4) axis2(2,1)],[axis2(3,4) axis2(3,1)],'r:');
    % plot3([axis2(1,4) axis2(1,2)],[axis2(2,4) axis2(2,2)],[axis2(3,4) axis2(3,2)],'g:');
    % plot3([axis2(1,4) axis2(1,3)],[axis2(2,4) axis2(2,3)],[axis2(3,4) axis2(3,3)],'b:');
    % plot3(pts3d(1,:),pts3d(2,:),pts3d(3,:),'bo');
    % grid on;
    % axis equal;

    % Cheirality check.
    nz1 = pts3d(3,:)>=0;
    pts3dp = makeinhomogeneous([R t;0 0 0 1]*makehomogeneous(pts3d));
    nz2 = pts3dp(3,:)>=0;    
    nz(i) = sum(and(nz1,nz2));
end
[~,bestC] = max(nz);

switch bestC
    case 1
        R = R1;
        t = t1;
    case 2
        R = R1;
        t = t2;
    case 3
        R = R2;
        t = t1;
    case 4
        R = R2;
        t = t2;        
end

PL = K*[eye(3) zeros(3,1)];
PR = K*[R t];    
pts3d = triangulate(PL,PR,ptsL,ptsR);

% % Show triangulation of best relative pose.
% h = figure;
% 
% % Camera 1.
% axis1 = [eye(3) zeros(3,1)];
% 
% % Camera 2.
% axis2 = inv([R t; 0 0 0 1])*[axis1;1 1 1 1];
% axis2 = makeinhomogeneous(axis2);
% 
% clf(h);
% hold on;
% plot3([axis1(1,4) axis1(1,1)],[axis1(2,4) axis1(2,1)],[axis1(3,4) axis1(3,1)],'r-');
% plot3([axis1(1,4) axis1(1,2)],[axis1(2,4) axis1(2,2)],[axis1(3,4) axis1(3,2)],'g-');
% plot3([axis1(1,4) axis1(1,3)],[axis1(2,4) axis1(2,3)],[axis1(3,4) axis1(3,3)],'b-');
% plot3([axis2(1,4) axis2(1,1)],[axis2(2,4) axis2(2,1)],[axis2(3,4) axis2(3,1)],'r:');
% plot3([axis2(1,4) axis2(1,2)],[axis2(2,4) axis2(2,2)],[axis2(3,4) axis2(3,2)],'g:');
% plot3([axis2(1,4) axis2(1,3)],[axis2(2,4) axis2(2,3)],[axis2(3,4) axis2(3,3)],'b:');
% plot3(pts3d(1,:),pts3d(2,:),pts3d(3,:),'bo');
% grid on;
% axis equal;

end