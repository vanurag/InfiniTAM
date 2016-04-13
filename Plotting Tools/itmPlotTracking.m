clear all; close all; clc;

% make sure to update T_mocapG_vioG for each dataset
bagFile = 'drz-rig-result_2016-03-19-15-35-46.bag';
% bagFile = 'drz-rig_2016-03-23-15-30-27.bag';
% T_mocapG_vioG = [ 0.8490  0.3491  0.3966  0.326238415304; ...
%                  -0.1043 -0.6251  0.7735 -0.23859243723; ...
%                   0.5179 -0.6981 -0.4943  1.55077250793; ...
%                   0.0000  0.0000  0.0000  1.0000];
% T_mocapG_vioG = [ 0.8490 -0.1043  0.5179  0.326238415304; ...
%                   0.3491 -0.6251 -0.6981 -0.23859243723; ...
%                   0.3966  0.7735 -0.4943  1.55077250793; ...
%                   0.0000  0.0000  0.0000  1.0000];
% T_mocapG_vioG = [-0.565005104881  -0.761974023331  -0.291781115852  0.431871644954; ...
%                  0.796465167594 -0.591902045105  0.00288397532099 -0.103036423436; ...
%                   -0.177154928262 -0.231863469037 0.956261179417  0.995096238726; ...
%                   0.0000  0.0000  0.0000  1.0000];
              
% T_mocapG_vioG = [-0.5867   -0.7620   -0.2740    0.5429; ...
%     0.7921   -0.6104    0.0013   -0.0809; ...
%    -0.1682   -0.2163    0.9618    1.0277; ...
%          0         0         0    1.0000];

% T_mocapG_vioG = [-0.5743   -0.7685    0.2821    0.3557; ...
%     0.7773   -0.6200   -0.1066    0.2791; ...
%     0.2569    0.1581    0.9535    1.2108; ...
%          0         0         0    1.0000];
     
T_mocapG_vioG = [-0.7030   -0.0354    0.7103    0.2150; ...
    0.6612   -0.4003    0.6345   -0.2965; ...
    0.2619    0.9157    0.3048    1.2562; ...
         0         0         0    1.0000];

% T_mocapG_icpG = [0.9376    0.2176    0.2706    0.2687; ...
%    -0.2200   -0.2308    0.9478   -0.5553; ...
%     0.2687   -0.9483   -0.1684    1.1036; ...
%          0         0         0    1.0000];
              
T_mocapG_icpG = [0.9299    0.2534    0.2660    0.2478; ...
   -0.2067   -0.2380    0.9490   -0.5539; ...
    0.3037   -0.9376   -0.1688    1.0757; ...
         0         0         0    1.0000];
         
% T_mocapG_vioG = [-0.5382 -0.2691  0.7987 -0.2915; ...
%                  -0.7302  0.6220 -0.2825 -0.2370; ...
%                  -0.4208 -0.7353 -0.5313  1.6283; ...
%                   0.0000  0.0000  0.0000  1.0000];
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
plotMocap = 1;
plotVio = 0;
drawMocapVioEdges = 0;
plotIcp = 1;
drawMocapIcpEdges = 1;
showImage = 0;
%setup for plotting    
if(plotMocap || plotVio)
    R = [1 0 0; 0 1 0; 0 0 1];
    figure(1);
    hold on
    marker = plotCamera('Location', [0 0 0], 'Orientation', R, 'Opacity', 0, 'Size', 0.07, 'Color', [1 0 0], 'Label', 'marker', 'AxesVisible', 1);
    odom = plotCamera('Location', [0 0 0], 'Orientation', R, 'Opacity', 0, 'Size', 0.07, 'Color', [0 0 1], 'Label', 'odom', 'AxesVisible', 1);
    icp = plotCamera('Location', [0 0 0], 'Orientation', R, 'Opacity', 0, 'Size', 0.07, 'Color', [0 1 0], 'Label', 'icp', 'AxesVisible', 1);
%     hold on
    grid on
    axis equal
    axis manual
    xlim([-2, 2]);
    ylim([-2, 2]);
    zlim([0, 3]);
    
    num_msgs = size(msgData.times, 1);
    plot3([0, 0.2], [0, 0], [0, 0], 'r');
    plot3([0, 0], [0, 0.2], [0, 0], 'g');
    plot3([0, 0], [0, 0], [0, 0.2], 'b');
    plot3([0, 0], [0, 0], [0, 0], 'k');

    T_mocap_vio = [-0.98165076 0.00766302 0.18984042 0.00108668; ...
                    0.17795358 -0.31287988  0.93287942 0.11919152; ...
                    0.06656298  0.94966982  0.30594812 -0.19977433];

%     T_mocap_vio = eye(4);
    T_vio_mocap = my_inv(T_mocap_vio);
    T_vioG_mocapG = my_inv(T_mocapG_vioG);
    T_icpG_mocapG = my_inv(T_mocapG_icpG);
    T_vioG_vio = eye(4);
    T_icpG_icp = eye(4);
    T_mocap_mocapG = eye(4);
    latest_mocap_time = inf;
    latest_icp_time = inf;
    latest_vio_time = inf;
    % eval error (cost function)
    error_icp = 0;
    error_vio = 0;
    for i = 1:num_msgs
        disp(msgData.source{i})
        T = reshape(msgData.T_G_F(i,:), [4, 4]);
        
        % draw cameras (pose, position)
        if (strcmp(msgData.source{i}, mocapTopic) == 1)
            latest_mocap_time = msgData.times(i);
            color = [1, ((num_msgs-i)/num_msgs)*0.6, ((num_msgs-i)/num_msgs)*0.6];
            T_mocapG_mocap = T;
            T_mocap_mocapG = my_inv(T_mocapG_mocap);
            T_transformed = T_vio_mocap*T_mocap_mocapG;
            T_other = my_inv(T_transformed);
            position_mocap = [T_other(1, 4), T_other(2, 4), T_other(3, 4)];
            if (plotMocap)
                marker.Orientation = T_transformed(1:3, 1:3);
                marker.Location = position_mocap;
                plot3(position_mocap(1), position_mocap(2), position_mocap(3), '.', 'Color', color, 'MarkerSize', 1);
            end
        elseif (strcmp(msgData.source{i}, icpPoseTopic) == 1)
            latest_icp_time = msgData.times(i);
            color = [((num_msgs-i)/num_msgs)*0.6, 1.0, ((num_msgs-i)/num_msgs)*0.6];
            T_icpG_icp = T;
            T_icp_icpG = my_inv(T_icpG_icp);
            T_transformed = T_icp_icpG*T_icpG_mocapG;
            T_other = my_inv(T_transformed);
            position_icp = [T_other(1, 4), T_other(2, 4), T_other(3, 4)];
            if (plotIcp)
                icp.Orientation = T_transformed(1:3, 1:3);
                icp.Location = position_icp;
                plot3(position_icp(1), position_icp(2), position_icp(3), '.', 'Color', color, 'MarkerSize', 1);
            end
        elseif (strcmp(msgData.source{i}, vioTopic) == 1)
            latest_vio_time = msgData.times(i);
            color = [((num_msgs-i)/num_msgs)*0.6, ((num_msgs-i)/num_msgs)*0.6, 1.0];
%             T_vio_vioG = eye(4);
%             T_vio_vioG(1:3, 1:3) = T(1:3, 1:3);
%             T_vio_vioG(1:3, 4) = -T(1:3, 1:3)*T(1:3, 4);

            T_vioG_vio = T;
%             disp('vio')
%             disp(T_vioG_vio)
            T_vio_vioG = my_inv(T_vioG_vio);
            
            T_transformed = T_vio_vioG*T_vioG_mocapG;
            T_other = my_inv(T_transformed);
            position_vio = [T_other(1, 4), T_other(2, 4), T_other(3, 4)];
            if (plotVio)
                odom.Orientation = T_transformed(1:3, 1:3);
                odom.Location = position_vio;
                plot3(position_vio(1), position_vio(2), position_vio(3), '.', 'Color', color, 'MarkerSize', 1);
            end
        elseif (strcmp(msgData.source{i}, camTopic) == 1 && showImage)
            figure(2)
            imshow(msgData.image{i});
            figure(1)
        end
        
        
        % draw edges
        if (latest_mocap_time ~= inf && latest_icp_time ~= inf)
            if (abs(latest_mocap_time-latest_icp_time) < 0.01)
                error_icp = error_icp + norm(position_mocap-position_icp);
                if (drawMocapIcpEdges)
                    plot3([position_mocap(1) position_icp(1)], ...
                          [position_mocap(2) position_icp(2)], ...
                          [position_mocap(3) position_icp(3)], '-', 'Color', [0 0 0]);
                end
            end
        end
        if (latest_mocap_time ~= inf && latest_vio_time ~= inf)
            if (abs(latest_mocap_time-latest_vio_time) < 0.01)
                error_vio = error_vio + norm(position_mocap-position_vio);
                if (drawMocapVioEdges)
                    plot3([position_mocap(1) position_vio(1)], ...
                          [position_mocap(2) position_vio(2)], ...
                          [position_mocap(3) position_vio(3)], '-', 'Color', [0 1 1]);
                end
            end
        end
        drawnow;
        
        
        % inertial alignment
        % T_mocapG_vioG
        disp('align')
        if (latest_mocap_time ~= inf && latest_vio_time ~= inf)
            if (abs(latest_mocap_time-latest_vio_time) < 0.01)
                T_mocap_vio_align(:,:,i) = my_inv(T_vioG_vio*T_vio_mocap*T_mocap_mocapG);
                disp('mocap-vio')
                disp(T_mocap_vio_align(:,:,i))
            end
        end
        % T_mocapG_icpG (NOTE: vio = icp = imu reference frame)
        if (latest_mocap_time ~= inf && latest_icp_time ~= inf)
            if (abs(latest_mocap_time-latest_icp_time) < 0.01)
                T_mocap_icp_align(:,:,i) = my_inv(T_icpG_icp*T_vio_mocap*T_mocap_mocapG);
                disp('mocap-icp')
                disp(T_mocap_icp_align(:,:,i));
            end
        end
%         pause;
    end
%     plot3(msgData.T_G_F(:,13), msgData.T_G_F(:,14), msgData.T_G_F(:,15));
end