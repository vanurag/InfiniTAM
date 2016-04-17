clear all; close all; clc;

% make sure to update T_mocapG_vioG for each dataset

% bagFile = 'drz-rig-result_2016-03-19-15-35-46.bag';   % icp
% bagFile = 'drz-rig-result_2016-03-23-15-30-27.bag';   % vio

% bagFile = 'drz-rig-result_2016-04-07-20-15-00_clipped.bag'; % vio
% bagFile = 'drz-rig-result_2016-04-07-20-15-00_clipped2.bag'; % icp
% bagFile = 'drz-rig-result_2016-04-07-20-15-00_clipped3.bag'; % closed-loop1 (rovio estimate to icp)
% bagFile = 'drz-rig-result_2016-04-07-20-15-00_clipped4.bag'; % closed-loop2 (rovio estimate to icp and icp estimate as pose update to rovio)

bagFile = 'drz-rig-result_2016-04-17-21-29-54.bag'; % vio

% drz-rig-result_2016-03-23-15-30-27.bag
% T_mocapG_vioG = [-0.9514   -0.3068   -0.0256   -0.0140; ...
%     0.3068   -0.9518    0.0036    0.1722; ...
%    -0.0254   -0.0044    0.9997    0.8937; ...
%          0         0         0    1.0000];

% drz-rig-result_2016-04-07-20-15-00_clipped.bag
% T_mocapG_vioG = [0.9918    0.1203   -0.0442    0.3336; ...
%    -0.1208    0.9926   -0.0094   -0.1867; ...
%     0.0428    0.0147    0.9990    1.0122; ...
%     0         0         0         1     ];

% drz-rig-result_2016-04-17-21-29-54.bag
T_mocapG_vioG = [0.7900    0.5660    0.2354    0.6563; ...
   -0.5260    0.8232   -0.2137   -0.0288; ...
   -0.3147    0.0450    0.9480    1.2026; ...
         0         0         0    1.0000];

% drz-rig-result_2016-03-19-15-35-46.bag
% T_mocapG_icpG = [0.8908    0.1880   -0.4137    0.2268; ...
%     0.4463   -0.1905    0.8744   -0.5271; ...
%     0.0855   -0.9635   -0.2536    1.0217; ...
%          0         0         0    1.0000];

% drz-rig-result_2016-04-07-20-15-00_clipped.bag
T_mocapG_icpG = [-0.9798    0.0998   -0.1731    0.3157; ...
    0.1923    0.2356   -0.9526   -0.1082; ...
   -0.0543   -0.9667   -0.2501    1.3096; ...
   0          0            0       1];
         
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
plotVio = 1;
drawMocapVioEdges = 0;
plotVioUncertainity = 0;
plotIcp = 0;
drawMocapIcpEdges = 0;
showImage = 0;
%setup for plotting    
if(plotMocap || plotVio)
    R = [1 0 0; 0 1 0; 0 0 1];
    figure(1);
    hold on
    title('real-time pose estimates from Mocap, VIsual-Inertial Odometry and ICP')
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
    position_mocap = inf;
    distance_travelled = 0.0;
    % eval error (cost function)
    error_icp = 0;
    error_vio = 0;
    num_matches_icp = 0;
    num_matches_vio = 0;
    for i = 1:num_msgs
        disp(sprintf('msg #%d', i));
        disp(sprintf('distance travelled: %d m', distance_travelled));
        T = reshape(msgData.T_G_F(i,:), [4, 4]);
        
        % draw cameras (pose, position)
        if (strcmp(msgData.source{i}, mocapTopic) == 1)
            latest_mocap_time = msgData.times(i);
            color = [1, ((num_msgs-i)/num_msgs)*0.6, ((num_msgs-i)/num_msgs)*0.6];
            T_mocapG_mocap = T;
            T_mocap_mocapG = my_inv(T_mocapG_mocap);
            T_transformed = T_vio_mocap*T_mocap_mocapG;
            T_other = my_inv(T_transformed);
            if (position_mocap ~= inf)
                distance_travelled = distance_travelled + norm(position_mocap-T_other(1:3,4)');
            end
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
            T_vioG_vio = T;
            T_vio_vioG = my_inv(T_vioG_vio);
            T_transformed = T_vio_vioG*T_vioG_mocapG;
            T_other = my_inv(T_transformed);
            position_vio = [T_other(1, 4), T_other(2, 4), T_other(3, 4)];
            if (plotVio)
                odom.Orientation = T_transformed(1:3, 1:3);
                odom.Location = position_vio;
                plot3(position_vio(1), position_vio(2), position_vio(3), '.', 'Color', color, 'MarkerSize', 1);
                % covariance
                pose_cov = reshape(msgData.covariance{i}, 6, 6)';
                % TODO: transform covariance matrix to mocap inertial frame
                if (rem(distance_travelled, 0.5) < 0.05 && plotVioUncertainity)
                    covarianceEllipse3D(position_vio, pose_cov(1:3,1:3), color, 1.0);
                end
            end
        elseif (strcmp(msgData.source{i}, camTopic) == 1 && showImage)
            figure(2)
            imshow(msgData.image{i});
            figure(1)
        end
        
        
        % evaluate and plot alignment errors
        if (latest_mocap_time ~= inf && latest_icp_time ~= inf)
            if (abs(latest_mocap_time-latest_icp_time) < 0.01)
                % average alignment error
                error_icp = (num_matches_icp*(error_icp) + norm(position_mocap-position_icp))/...
                            (num_matches_icp+1);
                num_matches_icp = num_matches_icp + 1;
                figure(3)
                title('Mocap - ICP alignment error')
                xlabel('msg #')
                ylabel('avg. error in cm')
                hold on
                plot(i, 100.0*error_icp, '*', 'Color', [0 0 1]);
                figure(1)
                disp(sprintf('mocap - icp alignment error: %f cm', 100.0*(error_icp)));
                % plot
                if (drawMocapIcpEdges)
                    plot3([position_mocap(1) position_icp(1)], ...
                          [position_mocap(2) position_icp(2)], ...
                          [position_mocap(3) position_icp(3)], '-', 'Color', [0 0 0]);
                end
                % inertial align
                T_mocap_icp_align(:,:,i) = my_inv(T_icpG_icp*T_vio_mocap*T_mocap_mocapG);
                disp('mocap-icp')
                disp(T_mocap_icp_align(:,:,i))
            end
        end
        if (latest_mocap_time ~= inf && latest_vio_time ~= inf)
            if (abs(latest_mocap_time-latest_vio_time) < 0.01)
                % average alignment error
                error_vio = (num_matches_vio*(error_vio) + norm(position_mocap-position_vio))/...
                            (num_matches_vio+1);
                num_matches_vio = num_matches_vio + 1;
                figure(4)
                xlabel('msg #')
                ylabel('avg. error in cm')
                title('Mocap - Visual-Inertial Odometry alignment error')
                hold on
                plot(i, 100.0*error_vio, '*', 'Color', [0 0 1]);
                if (rem(distance_travelled, 0.5) < 0.05)
                    covarianceEllipse1D([i; 100.0*error_vio], pose_cov(1:3,1:3), [0 1 0], 100.0);
                end
                figure(1)
                disp(sprintf('mocap - vio alignment error: %f cm', 100.0*(error_vio)));
                % plot
                if (drawMocapVioEdges)
                    plot3([position_mocap(1) position_vio(1)], ...
                          [position_mocap(2) position_vio(2)], ...
                          [position_mocap(3) position_vio(3)], '-', 'Color', [0 1 1]);
                end
                % inertial align
                T_mocap_vio_align(:,:,i) = my_inv(T_vioG_vio*T_vio_mocap*T_mocap_mocapG);
                disp('mocap-vio')
                disp(T_mocap_vio_align(:,:,i))
            end
        end
        drawnow;
        pause;
    end
%     plot3(msgData.T_G_F(:,13), msgData.T_G_F(:,14), msgData.T_G_F(:,15));
end