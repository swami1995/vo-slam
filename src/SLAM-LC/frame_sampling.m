function [Track, effNumFrames, points3d] = frame_sampling(num_frames,keyp,pose,K)
%   This function samples a list of frames so that the singular values for a
%   succussive pair of frames is more than a threshold, tr.
%   Track -  is a 4x4xnum_frames dimensional matrix containing the relative 
%            frame to frame transformations
%   effNumFrames - Total number frames sampled
%   points3d - 3d points corresponding to the matches between the
%              successive sampled frames as seen by the first frame
%   num_frames - total number of frames
Track = zeros(4,4,num_frames);
points3d = cell(num_frames,3);
chk=0;
effNumFrames=1;
Track(:,:,1) = eye(4);
T_w= Track;
tr=12;
for i= 2:num_frames
    if pose{i,4}(1,1)<tr && chk==0
        chk=chk+1;
        fprintf('%d %d\n',i,1);
        T_w(:,:,i)=T_w(:,:,i-1);
    elseif chk==0
        Track(:,:,i) = [pose{i,2} pose{i,3} ; 0 0 0 1];
        points3d{i-1,1} = keyp{i-1,4};
        points3d{i-1,2} = keyp{i-1,3};
        points3d{i,3} = pose{i,4}(1,1);
        effNumFrames = effNumFrames + 1;
        T_w(:,:,i)=Track(:,:,i)*T_w(:,:,i-1);
                
    elseif pose{i,4}(1,1)<tr
        f2=i;
        f1=i-chk-1;
        keypoint=cell(1,4);
        keypoint{1,1}=keyp{f1,1};
        keypoint{1,2}=keyp{f1,2};
        [keypoint{1,3}, scores] = vl_ubcmatch(keyp{f1,2},keyp{f2,2});
        X1 = keyp{f1,1}(1:2,keypoint{1,3}(1,:)) ; X1(3,:) = 1 ;
        X2 = keyp{f2,1}(1:2,keypoint{1,3}(2,:)) ; X2(3,:) = 1 ;
        [F,inx] = ransacfitfundmatrix(X1(1:2,:),X2(1:2,:),0.0001);
        [R,t,pts3d,singvals] = getpose(F,K,X1(1:2,inx),X2(1:2,inx));
        fprintf('%d %d %d\n',size(inx,2),i,2);
        
        
        if singvals(1,1)>tr
            Track(:,:,i) = [R t ; 0 0 0 1];
            chk=0;
            points3d{f1,1} = pts3d;
            pause(0.1);
            points3d{f1,2} = keypoint{1,3}(:,inx);
            points3d{i,3} = singvals(1,1);
            effNumFrames = effNumFrames + 1;
            T_w(:,:,i)=Track(:,:,i)*T_w(:,:,i-1);
            fprintf('%d \n',i);

        else
            chk=chk+1;
            T_w(:,:,i)=T_w(:,:,i-1);
        end
    else
        f2=i;
        f1=i-chk-1;
        keypoint=cell(1,4);
        keypoint{1,1}=keyp{f1,1};
        keypoint{1,2}=keyp{f1,2};
        [keypoint{1,3}, scores] = vl_ubcmatch(keyp{f1,2},keyp{f2,2});
        X1 = keyp{f1,1}(1:2,keypoint{1,3}(1,:)) ; X1(3,:) = 1 ;
        X2 = keyp{f2,1}(1:2,keypoint{1,3}(2,:)) ; X2(3,:) = 1 ;
        [F,inx] = ransacfitfundmatrix(X1(1:2,:),X2(1:2,:),0.0001);
        [R,t,pts3d,singvals] = getpose(F,K,X1(1:2,inx),X2(1:2,inx));
        Track(:,:,i) = [R t ; 0 0 0 1];
        chk=0;
        points3d{f1,1} = pts3d;
        points3d{f1,2} = keypoint{1,3}(:,inx);
        points3d{i,3} = singvals(1,1);
        effNumFrames = effNumFrames + 1;
        T_w(:,:,i)=Track(:,:,i)*T_w(:,:,i-1);
        fprintf('%d %d %d\n',size(inx,2),i,3);
    end
end
end
