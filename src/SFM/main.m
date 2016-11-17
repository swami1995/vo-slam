close all;
clear;

cd vlfeat-0.9.20/toolbox/
vl_setup
cd ../../

files = dir('data/*.jpeg');

%%

ran = 400:5:2000; % Results stored as seq01.mat.
% ran = 6400:5:8000; % Results stored as seq02.mat.

% Camera intrinsics.
scale = 0.4;
K = scale*[1682.68657 0 1003.35296 ; 0 1675.72660 500.79178; 0 0 1];

% Storage.
pose = cell(length(ran),3); % F, R and t.
keyp = cell(length(ran),4); % KP coords, KP descrip, match index, 3D coords.

% Initialise.
imgL = imread(sprintf('data/%s',files(ran(1)).name));
imgL = rgb2gray(imgL);
imgL = imresize(imgL,scale);
[keyp{1,1},keyp{1,2}] = vl_sift(single(imgL));
pose{1,1} = [];
pose{1,2} = eye(3);
pose{1,3} = zeros(3,1);
pose{1,4} = ones(1,3)';

% Go.
for i=2:length(ran)
    imgL = imread(sprintf('data/%s',files(ran(i-1)).name));
    imgR = imread(sprintf('data/%s',files(ran(i)).name));
    
    imgLc = imresize(imgL,scale); 
    
    imgL = rgb2gray(imgL);
    imgL = imresize(imgL,scale);
    imgR = rgb2gray(imgR);
    imgR = imresize(imgR,scale);
    
    % SIFT.
    kpL = keyp{i-1,1};
    dcL = keyp{i-1,2};
    [kpR,dcR] = vl_sift(single(imgR));

    % Match and RANSAC.
    [matchinx,scores] = vl_ubcmatch(dcL,dcR);
    matches = [kpL(1:2,matchinx(1,:));kpR(1:2,matchinx(2,:))];
    [F,inx] = ransacfitfundmatrix(matches(1:2,:),matches(3:4,:),0.0001);
    matchinx = matchinx(:,inx);
    matches = matches(:,inx);

    % Triangulate and disambiguate.
    [R,t,pts3d,singvals] = getpose(F,K,matches(1:2,:),matches(3:4,:));
    
    % Save. 
    keyp{i-1,3} = matchinx;
    keyp{i-1,4} = pts3d;
    keyp{i,1} = kpR;
    keyp{i,2} = dcR;
    pose{i,1} = F;
    pose{i,2} = R;
    pose{i,3} = t;
    pose{i,4} = singvals;
    
    fprintf('Done iter %d (frames %d <--> %d).\n',i,ran(i-1),ran(i));    
end
