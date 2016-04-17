function [ align_error ] = optimize_inertial_alignment_fun( optimize_params , msgData, mocapTopic, vioTopic, icpPoseTopic, type)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

T_mocapG_vioG = eye(4);
T_mocapG_icpG = eye(4);
if (strcmp(type, 'vio') == 1)
    T_mocapG_vioG(1:3, 1:3) = eul2rotm(optimize_params(1,1:3));
    T_mocapG_vioG(1:3, 4) = optimize_params(1,4:6)';
elseif (strcmp(type, 'icp') == 1)
    T_mocapG_icpG(1:3, 1:3) = eul2rotm(optimize_params(1,1:3));
    T_mocapG_icpG(1:3, 4) = optimize_params(1,4:6)';
else
    error('Type should be either vio or icp');
end

%%
num_msgs = size(msgData.times, 1);
T_mocap_vio = [-0.98165076 0.00766302 0.18984042 0.00108668; ...
                0.17795358 -0.31287988  0.93287942 0.11919152; ...
                0.06656298  0.94966982  0.30594812 -0.19977433];
T_vio_mocap = my_inv(T_mocap_vio);
T_vioG_mocapG = my_inv(T_mocapG_vioG);
T_icpG_mocapG = my_inv(T_mocapG_icpG);
latest_mocap_time = inf;
latest_icp_time = inf;
latest_vio_time = inf;
% eval error (cost function)
error_icp = 0;
error_vio = 0;
num_matches_icp = 0;
num_matches_vio = 0;
for i = 1:num_msgs
    T = reshape(msgData.T_G_F(i,:), [4, 4]);

    % update pose, position
    if (strcmp(msgData.source{i}, mocapTopic) == 1)
        latest_mocap_time = msgData.times(i);
        T_mocapG_mocap = T;
        T_mocap_mocapG = my_inv(T_mocapG_mocap);
        T_transformed = T_vio_mocap*T_mocap_mocapG;
        T_other = my_inv(T_transformed);
        position_mocap = [T_other(1, 4), T_other(2, 4), T_other(3, 4)];
    elseif (strcmp(msgData.source{i}, icpPoseTopic) == 1)
        latest_icp_time = msgData.times(i);
        T_icpG_icp = T;
        T_icp_icpG = my_inv(T_icpG_icp);
        T_transformed = T_icp_icpG*T_icpG_mocapG;
        T_other = my_inv(T_transformed);
        position_icp = [T_other(1, 4), T_other(2, 4), T_other(3, 4)];
    elseif (strcmp(msgData.source{i}, vioTopic) == 1)
        latest_vio_time = msgData.times(i);
        T_vioG_vio = T;
        T_vio_vioG = my_inv(T_vioG_vio);
        T_transformed = T_vio_vioG*T_vioG_mocapG;
        T_other = my_inv(T_transformed);
        position_vio = [T_other(1, 4), T_other(2, 4), T_other(3, 4)];
    end


    % error metrics
    if (latest_mocap_time ~= inf && latest_icp_time ~= inf)
        if (abs(latest_mocap_time-latest_icp_time) < 0.01)
            error_icp = error_icp + norm(position_mocap-position_icp);
            num_matches_icp = num_matches_icp + 1;
        end
    end
    if (latest_mocap_time ~= inf && latest_vio_time ~= inf)
        if (abs(latest_mocap_time-latest_vio_time) < 0.01)
            error_vio = error_vio + norm(position_mocap-position_vio);
            num_matches_vio = num_matches_vio + 1;
        end
    end
end

if (strcmp(type, 'vio') == 1)
    align_error = 100.0*error_vio/num_matches_vio;
elseif (strcmp(type, 'icp') == 1)
    align_error = 100.0*error_icp/num_matches_icp;
end

end

