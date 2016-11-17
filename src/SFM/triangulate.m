function [pts3d] = triangulate(PL,PR,ptsL,ptsR)
% PL (3x4) : left camera matrix.
% PR (3x4) : right camera matrix.
% ptsL (2xN) : left image measurements.
% ptsR (2xN) : right image measurements.

N = size(ptsL,2);

pts3d = zeros(3,N);

for i=1:N
    
    xL = ptsL(1,i);
    yL = ptsL(2,i);
    xR = ptsR(1,i);
    yR = ptsR(2,i);
    
    A = [ xL*PL(3,1)-PL(1,1) xL*PL(3,2)-PL(1,2) xL*PL(3,3)-PL(1,3) xL*PL(3,4)-PL(1,4) ; ...
          yL*PL(3,1)-PL(2,1) yL*PL(3,2)-PL(2,2) yL*PL(3,3)-PL(2,3) yL*PL(3,4)-PL(2,4) ; ...
          xR*PR(3,1)-PR(1,1) xR*PR(3,2)-PR(1,2) xR*PR(3,3)-PR(1,3) xR*PR(3,4)-PR(1,4) ; ...
          yR*PR(3,1)-PR(2,1) yR*PR(3,2)-PR(2,2) yR*PR(3,3)-PR(2,3) yR*PR(3,4)-PR(2,4) ];
      
    [~,~,V] = svd(A);
    
    pts3d(:,i) = makeinhomogeneous(V(:,end));
end

end
