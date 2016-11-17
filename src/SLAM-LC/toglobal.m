function [G_3d, T_w, match, keypt, isoutlier] = toglobal(Track, effNumFrames,points3d,keyp,num_frames)
%% Description
% This function computes the global trajectories and 3d points coordinates after
% elimination from RANSAC circle fitting. 
% It also simultaneously computes loop matches at intervals of 5 frames for loop closure
%
%   Track -  is a 4x4xnum_frames dimensional matrix containing the relative frame to frame transformations
%   effNumFrames - Total number frames sampled
%   points3d - 3d points corresponding to the matches between the
%   successive sampled frames as seen by the first frame
%   num_frames - total number of frames
%   G_3d - The 3d points in the world coordinate frame
%   T_w - camera trajectories in world coordinate frame
%   match - cell with three columns
%           1st column contains match indices
%           2nd column contains feature coordinates for the matches
%           3rd column contains the index of the matching frame
%   isoutlier - indicates the list of outliers corresponding to each frame
%   keypt - similar to keyp but only for the sampled frames

%%  Initialisations
isoutlier=zeros(effNumFrames,1500);
T_w = zeros(4,4,effNumFrames);
T_w(:,:,1)= eye(4);
G_3d = cell(1,effNumFrames);
keypt = cell(effNumFrames,4);
match= cell(effNumFrames,3);
matches = cell(effNumFrames,321);
num_matches=zeros(effNumFrames,321);
max_matches=0;
thresh = 120;
q = 0;p=1;
circle = zeros(effNumFrames,3);
vec = zeros(1,num_frames-1);
%% Finding outliers and Loop Matches
for i=2:num_frames
    if sum(sum(Track(:,:,i)))
    Tn_1 = Track(:,:,i);
    q=q+1;
    proj=zeros(3,size(points3d{i-p,1},2));
    %% Finding outliers using RANSAC circle fitting
    m=vrrotvec(-Tn_1(1:3,4),[0 0 1]);
    R=vrrotvec2mat(m);

    % Projections
    
    for j=1:size(points3d{i-p,1},2)
        proj(:,j) = R*points3d{i-p,1}(:,j);
        proj(:,j) = proj(:,j) - dot(proj(:,j),[0;0;1])*[0;0;1];
    end
    
    % RANSAC
    [xc,yc,r,in]=ransacfitcircle(proj(1:2,:),0.005); %%   proj_loop(1:2,:)
    for j=1:size(points3d{i-p,1},2)
        proj(:,j) = R*points3d{i-p,1}(:,j)/r;
        if abs(dot(proj(:,j),[0;0;1]))>25
            isoutlier(q,j)=1;
        end
        proj(:,j) = proj(:,j) - dot(proj(:,j),[0;0;1])*[0;0;1];
    end
    [xc,yc,r1,in]=ransacfitcircle(proj(1:2,:),0.005);
    isoutlier(q,1:size(in,1)) = ~in' + isoutlier(q,1:size(in,1));
    
    %% Projecting 3d points into the global frame
    circle(q,:)=[xc yc r];
    Tn_1(1:3,4)=Tn_1(1:3,4)/r;
    T_w(:,:,q+1) = Tn_1*T_w(:,:,q);
    G_3d{1,q} = T_w(:,:,q)\[points3d{i-p,1}/r;ones(1,size(points3d{i-p,1},2))];
    
    keypt{q,1}=keyp{i-p,1};
    keypt{q,2}=keyp{i-p,2};
    keypt{q,3}=points3d{i-p,2};
    keypt{q,4}=points3d{i-p,1};
    
    %% finding loop matches
    % finding loop matches after every 5 frames 
    if mod(q,5)==0
        for j=5:5:i-150
            if vec(1,j)
                k=j;
            elseif vec(1,j-1)
                k=j-1;
            elseif vec(1,j-2)
                k=j-2;
            elseif vec(1,j-3)
                k=j-3;
            elseif vec(1,j-4)
                k=j-4;
            end
            if vec(1,j) || vec(1,j-4) || vec(1,j-3) || vec(1,j-2) || vec(1,j-1)
                [matches{q,j/5},scores] = vl_ubcmatch(keyp{i-p,2},keyp{k,2});
                num_matches(q,j/5)=size(matches{q,j/5},2);
                if ((num_matches(q,j/5)>2*max_matches && sum(match{q,3})+8<vec(1,k)) || (num_matches(q,j/5)>max_matches && sum(match{q,3})+7>vec(1,k))) && num_matches(q,j/5)>thresh
                    X1 = keyp{i-p,1}(1:2,matches{q,j/5}(1,:)) ; X1(3,:) = 1 ;
                    X2 = keyp{k,1}(1:2,matches{q,j/5}(2,:)) ; X2(3,:) = 1 ;
                    [F,inx] = ransacfitfundmatrix(X1,X2,0.0001);
                    max_matches=num_matches(q,j/5);
                    match{q,1}=matches{q,j/5}(:,inx);
                    match{q,2}=[X1(1:2,inx);X2(1:2,inx)];
                    match{q,3}=vec(1,k);
                end
            end
        end
        max_matches=0;
    end

        vec(1,i)=q+1;
        p=1;
        
    else
        p=p+1;
        vec(1,i)=0;
    end
    
end                