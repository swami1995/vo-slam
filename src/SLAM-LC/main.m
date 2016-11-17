clear
load('seq04.mat')
loop_closure = true;
num_frames=751;
[Track, effNumFrames, points3d] = frame_sampling(num_frames,keyp,pose,K);
[G_3d, T_w, match, keypt, isoutlier] =  toglobal(Track, effNumFrames,points3d,keyp,num_frames);
[ftr_3d,ftr_2d,num_traces,ftr,num_frames] = make_traces(keypt, G_3d, match, effNumFrames, isoutlier,loop_closure);
writetofile(ftr_3d,ftr_2d,num_traces,ftr,num_frames,T_w,K);
save('effNumFrames.mat',effNumFrames);

