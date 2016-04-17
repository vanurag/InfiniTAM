clear all; close all; clc;

% bagFile = 'drz-rig-result_2016-03-19-15-35-46.bag';   % icp
% bagFile = 'drz-rig-result_2016-03-23-15-30-27.bag'; % vio
% bagFile = 'drz-rig-result_2016-04-07-20-15-00_clipped3.bag'; % vio and icp

bagFile = 'drz-rig-result_2016-04-17-21-29-54.bag'; % vio
type = 'vio';
bag = rosbag(bagFile);
%%
%topics
icpPoseTopic = '/itm/pose';
mocapTopic = '/drz_rig/estimated_transform';
vioTopic = '/rovio/odometry';
camTopic = '/cam0/image_raw';

% mocapData = MocapInfo(bag, mocapTopic);
% vioData = VioInfo(bag, vioTopic);
msgData = ReadData(bag, {mocapTopic, vioTopic, icpPoseTopic, camTopic});
%%
T_mocapG_icpG_init = [-0.6704    0.0585   -0.7396    0.2317; ...
    0.7213    0.2845   -0.6314   -0.0931; ...
    0.1734   -0.9568   -0.2328    1.3231; ...
         0         0         0    1.0000];
T_mocapG_vioG_init = [0.7900    0.5660    0.2354    0.6563; ...
   -0.5260    0.8232   -0.2137   -0.0288; ...
   -0.3147    0.0450    0.9480    1.2026; ...
         0         0         0    1.0000];
     
optimize_params_init = [rotm2eul(T_mocapG_icpG_init(1:3,1:3)), T_mocapG_icpG_init(1:3,4)'];
% optimize_params_init = [rotm2eul(T_mocapG_vioG_init(1:3,1:3)), T_mocapG_vioG_init(1:3,4)'];

disp('optimizing...')
optimize_params_optimal = fminsearch(@(optimize_params) ...
    optimize_inertial_alignment_fun(optimize_params, msgData, mocapTopic, vioTopic, icpPoseTopic, type), ...
        optimize_params_init, optimset('Display', 'iter', 'PlotFcns', @optimplotfval));
    
T_inertial_optimal(1:3, 1:3) = eul2rotm(optimize_params_optimal(1,1:3));
T_inertial_optimal(1:3, 4) = optimize_params_optimal(1,4:6)';
disp('done')