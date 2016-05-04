clear all; close all; clc;

% bagFile = 'drz-rig-result_2016-03-19-15-35-46.bag';   % icp
% bagFile = 'drz-rig-result_2016-03-23-15-30-27.bag'; % vio
% bagFile = 'drz-rig-result_2016-04-07-20-15-00_clipped3.bag'; % closed-loop1 (rovio estimate to icp)

% bagFile = 'drz-rig-result_2016-04-17-21-29-54.bag'; % vio
% bagFile = 'drz-rig-result_2016-04-17-21-29-54_3.bag'; % closed-loop1 (rovio estimate to icp)
% bagFile = 'drz-rig-result_2016-04-17-21-29-54_4.bag'; % closed-loop2 (rovio estimate to icp and icp estimate as pose update to rovio)
bagFile = 'drz-rig-result_2016-04-17-21-29-54_5.bag'; % closed-loop2 but ICP kciks in after few minutes
type = 'icp';
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
T_mocapG_icpG_init = [-0.8002    0.4812   -0.3579    3.8604; ...
    0.5657    0.4076   -0.7168   -0.5738; ...
   -0.1990   -0.7761   -0.5984   -1.0592; ...
         0         0         0    1.0000];
T_mocapG_vioG_init = [0.9999   -0.0165    0.0003    0.6483; ...
    0.0164    0.9904   -0.1373   -0.2053; ...
    0.0020    0.1373    0.9905    1.8617; ...
    0         0         0         1     ];
     
if (strcmp(type, 'icp') == 1)
    optimize_params_init = [rotm2eul(T_mocapG_icpG_init(1:3,1:3)), T_mocapG_icpG_init(1:3,4)'];
elseif (strcmp(type, 'vio') == 1)
    optimize_params_init = [rotm2eul(T_mocapG_vioG_init(1:3,1:3)), T_mocapG_vioG_init(1:3,4)'];
end

disp('optimizing...')
optimize_params_optimal = fminsearch(@(optimize_params) ...
    optimize_inertial_alignment_fun(optimize_params, msgData, mocapTopic, vioTopic, icpPoseTopic, type), ...
        optimize_params_init, optimset('Display', 'iter', 'PlotFcns', @optimplotfval));
    
T_inertial_optimal(1:3, 1:3) = eul2rotm(optimize_params_optimal(1,1:3));
T_inertial_optimal(1:3, 4) = optimize_params_optimal(1,4:6)';
disp('done')