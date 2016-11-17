function [ftr_3d,ftr_2d,num_traces,ftr,num_frames] = make_traces(keypt,G_3d,match,effNumFrames,isoutlier)
%% Description
% This function computes the traces( including loop traces) for all 3d points that have trace
% length greater than 1. Trace for a 3d point includes all the frames that
% view that particular 3d point.
%
%   effNumFrames - Total number frames sampled
%   G_3d - The 3d points in the world coordinate frame for each frame
%   match - cell with three columns
%           1st column contains match indices
%           2nd column contains feature coordinates for the matches
%           3rd column contains the index of the matching frame
%   isoutlier - indicates the list of outliers corresponding to each frame
%   keypt - similar to keyp but only for the sampled frames
%   ftr_3d - the list of 3d points in the world coordinte frame 
%   ftr_2d - each row corresponds to a single 3d point and contains the 
%            list of 2d projections the 3d point makes on the respective 
%            frames that it is visible in
%   ftr - each row corresponds to a single 3d point and lists the frames
%         that view the corresponding 3d point
%   num_traces - The number traces, i.e, the total number of distinct 3d
%                points
%   num_frames - The frame at which we want the trajectory to end
%   loop_closure - boolian value, true for implementing loop closure and
%                  false for simple bundle adjustment without loop closure





%%  Initializing

cx=0;
cy=0;
num_frames = effNumFrames-1;
l=1;
num_traces = 0;
k = 1;
num_fts = 0;
ftr_idx = zeros(num_frames,10000);
ftr_3d = zeros(3,200000);
ftr = zeros(200000,num_frames);
ftr_2d = zeros(2,200000,num_frames);
m=0;
n=0;
a=0;
%%  First Frame Trace Initialisation

for j=1:size(keypt{l,4},2)
    if ~isoutlier(l,j)
        ftr_idx(l,j)=k;
        ftr_3d(:,k)=G_3d{1,l}(1:3,j);
        ftr(k,[l l+1])=1;
        ftr_2d(:,k,l)=vertcat(keypt{l,1}(1,keypt{l,3}(1,j))-cx, keypt{l,1}(2,keypt{l,3}(1,j))-cy);
        ftr_2d(:,k,l+1)=vertcat(keypt{l+1,1}(1,keypt{l,3}(2,j))-cx, keypt{l+1,1}(2,keypt{l,3}(2,j))-cy);
        num_fts=num_fts+2;
        num_traces=num_traces+1;
        k=k+1;
    end
end

%%  Trace Formation            
for i=l+1:num_frames-1
    %% Subsequent frame traces
    kp_3d = keypt{i,4};
    kp_match1 = keypt{i-1,3};
    kp_match2 = keypt{i,3};
    kp_point1 = keypt{i,1};
    kp_point2 = keypt{i+1,1};
    i;
    for j=1:size(kp_3d,2)
        if ~isoutlier(i,j)
            if sum(kp_match2(1,j)==kp_match1(2,:)) && ftr_idx(i-1,find(kp_match2(1,j)==kp_match1(2,:),1))
                match_idx = find(kp_match2(1,j)==kp_match1(2,:),1);
                idx=ftr_idx(i-1,match_idx);
                if ftr(idx,i+1) && ftr(idx,i)
                    find(ftr(idx,:));
                    n=n+1;
                end
                m=m+1;
                ftr_idx(i,j)=idx;
                ftr(idx,i+1)=1;
                ftr_2d(:,idx,i+1)=vertcat(kp_point2(1,kp_match2(2,j))-cx, kp_point2(2,kp_match2(2,j))-cy);
                num_fts=num_fts+1;
                
            else 
                ftr_idx(i,j)=k;
                ftr_3d(:,k)=G_3d{1,i}(1:3,j);
                ftr(k,[i i+1])=1;
                ftr_2d(:,k,i)=vertcat(kp_point1(1,kp_match2(1,j))-cx, kp_point1(2,kp_match2(1,j))-cy);
                ftr_2d(:,k,i+1)=vertcat(kp_point2(1,kp_match2(2,j))-cx, kp_point2(2,kp_match2(2,j))-cy);
                num_fts=num_fts+2;
                num_traces=num_traces+1;
                k=k+1;
                
            end
        end
    end

    %%  Adding Loop traces
    
    match_frame=match{i,3}; 
    if sum(match{i,3})>l && loop_closure
    a=a+1;
    if a~=1
    for j=1:size(match{i,1},2)
        if sum(match{i,1}(1,j)==kp_match2(1,:))
            if ~isoutlier(i,match{i,1}(1,j)==kp_match2(1,:)) && ftr_idx(i, find(match{i,1}(1,j)==kp_match2(1,:), 1));
            idx = ftr_idx(i, find(match{i,1}(1,j)==kp_match2(1,:), 1));
            m_idx = ftr_idx(match_frame, find(match{i, 1}(2, j)==keypt{match_frame, 3}(1, :), 1));
            if sum(m_idx)
                ftr(idx, :) = ftr(idx, :) + ftr(m_idx, :);
                ftr(m_idx,:) = ftr(idx, :);
                
                for m=1:num_frames
                    if ~norm(ftr_2d(:,idx,m))
                        ftr_2d(:, idx, m) = ftr_2d(:,m_idx,m);
                    elseif ~norm(ftr_2d(:,m_idx,m))
                        ftr_2d(:, m_idx, m) = ftr_2d(:,idx,m);
                    end
                end
            elseif match_frame>1 && sum(ftr_idx(match_frame-1, find(match{i, 1}(2, j)==keypt{match_frame-1, 3}(2, :), 1)))
                m_idx = ftr_idx(match_frame-1, find(match{i, 1}(2, j)==keypt{match_frame-1, 3}(2, :), 1));
                ftr(idx, :) = ftr(idx, :) + ftr(m_idx, :);
                ftr(m_idx,:) = ftr(idx, :);
                for m=1:num_frames
                    if ~norm(ftr_2d(:,idx,m))
                        ftr_2d(:, idx, m) = ftr_2d(:,m_idx,m);
                    elseif ~norm(ftr_2d(:,m_idx,m))
                        ftr_2d(:, m_idx, m) = ftr_2d(:,idx,m);
                    end
                end
            end    
            ftr(idx, match_frame)=1;
            ftr_idx(i, match_frame)=idx;
            ftr_2d(:,idx, match_frame)= vertcat(keypt{match_frame,1}(1, match{i,1}(2,j))-cx, keypt{match_frame,1}(2,match{i,1}(2,j))-cy);
            end

        elseif sum(match{i,1}(1,j)==kp_match1(2,:))
            if ~isoutlier(i-1,find(match{i,1}(1,j)==kp_match1(2,:),1)) && ftr_idx(i-1,find(match{i,1}(1,j)==kp_match1(2,:),1));
            idx=ftr_idx(i-1,find(match{i,1}(1,j)==kp_match1(2,:),1));
            m_idx = ftr_idx(match_frame, find(match{i, 1}(2, j)==keypt{match_frame, 3}(1, :), 1));
            if sum(m_idx)
                ftr(idx, :) = ftr(idx, :) + ftr(m_idx, :);
                ftr(m_idx,:) = ftr(idx, :);
                for m=1:num_frames
                    if ~norm(ftr_2d(:,idx,m))
                        ftr_2d(:, idx, m) = ftr_2d(:,m_idx,m);
                    elseif ~norm(ftr_2d(:,m_idx,m))
                        ftr_2d(:, m_idx, m) = ftr_2d(:,idx,m);
                    end
                end
            elseif match_frame>1 && sum(ftr_idx(match_frame-1, find(match{i, 1}(2, j)==keypt{match_frame-1, 3}(2, :), 1)))
                m_idx = ftr_idx(match_frame-1, find(match{i, 1}(2, j)==keypt{match_frame-1, 3}(2, :), 1));
                ftr(idx, :) = ftr(idx, :) + ftr(m_idx, :);
                ftr(m_idx,:) = ftr(idx, :);
                for m=1:num_frames
                    if ~norm(ftr_2d(:,idx,m))
                        ftr_2d(:, idx, m) = ftr_2d(:,m_idx,m);
                    elseif ~norm(ftr_2d(:,m_idx,m))
                        ftr_2d(:, m_idx, m) = ftr_2d(:,idx,m);
                    end
                end
            end    
            ftr(idx,match_frame)=1;
            ftr_idx(i,match_frame)=idx;
            ftr_2d(:,idx,match_frame)= vertcat(keypt{match_frame,1}(1,match{i,1}(2,j))-cx, keypt{match_frame,1}(2,match{i,1}(2,j))-cy);
            end

        else

            
            m_idx = ftr_idx(match_frame, find(match{i, 1}(2, j)==keypt{match_frame, 3}(1, :), 1));
            if sum(m_idx)
                ftr(m_idx,i) = 1;
                ftr_2d(:,m_idx,i)= vertcat(keypt{i,1}(1,match{i,1}(1,j))-cx, keypt{i,1}(2,match{i,1}(1,j))-cy);
            elseif match_frame>1 && sum(ftr_idx(match_frame-1, find(match{i, 1}(2, j)==keypt{match_frame-1, 3}(2, :), 1)))
                m_idx = ftr_idx(match_frame-1, find(match{i, 1}(2, j)==keypt{match_frame-1, 3}(2, :), 1));
                ftr(m_idx,i) = 1;
                ftr_2d(:,m_idx,i)= vertcat(keypt{i,1}(1,match{i,1}(1,j))-cx, keypt{i,1}(2,match{i,1}(1,j))-cy);
            end    

        end
        
    end
    end
    end    
end    