function [ msgData ] = ReadData(bag, topic)

%check inputs
validateattributes(topic,{'char', 'cell'},{'vector'});

%topics
msgData.topic = topic;

%filter bag
bag = select(bag,'Topic',topic);

%timestamaps
msgData.times = bag.MessageList.Time;

%preallocate memory -> Current Frame to Inertial Frame
msgData.T_G_F = zeros(length(msgData.times), 16);
msgData.image = cell(length(msgData.times), 1);
msgData.source = cell(length(msgData.times), 1);
msgData.covariance = cell(length(msgData.times), 1);

for i = 1:length(msgData.times)
    if(mod(i,1000) == 0)
        UpdateMessage('Reading Transform for transform %i of %i', i, length(msgData.times));
    end
    
    in = readMessages(bag, i);
    assignin('base', 'debug', in);
    
    tformMat = eye(4);
    
    if (strcmp(in{1}.MessageType, 'geometry_msgs/TransformStamped') == 1)
        tformMat(1:3,1:3)  = quat2rotm([in{1}.Transform.Rotation.W, in{1}.Transform.Rotation.X, in{1}.Transform.Rotation.Y, in{1}.Transform.Rotation.Z]);
        tformMat(1:3,4) = [in{1}.Transform.Translation.X, in{1}.Transform.Translation.Y, in{1}.Transform.Translation.Z];
        msgData.source{i} = char(bag.MessageList(i, 2).Topic);
    elseif (strcmp(in{1}.MessageType, 'nav_msgs/Odometry') == 1)
        or = in{1}.Pose.Pose.Orientation;
        pos = in{1}.Pose.Pose.Position;
        tformMat(1:3,1:3)  = quat2rotm([or.W, or.X, or.Y, or.Z]);
        tformMat(1:3,4) = [pos.X, pos.Y, pos.Z];
        msgData.source{i} = char(bag.MessageList(i, 2).Topic);
        msgData.covariance{i} = in{1}.Pose.Covariance();
    elseif (strcmp(in{1}.MessageType, 'sensor_msgs/Image') == 1)
        image = reshape(in{1}.Data,in{1}.Width,in{1}.Height)';
%         image = Undistort(image, camData.D, camData.K, camData.DistModel);
        msgData.image{i} = image;
        msgData.source{i} = char(bag.MessageList(i, 2).Topic);
    end
%     tformMat(1:3,1:3)  = quat2rot([in{1}.Pose.Orientation.W, in{1}.Pose.Orientation.X, in{1}.Pose.Orientation.Y, in{1}.Pose.Orientation.Z]');
%     tformMat(1:3,4) = [in{1}.Pose.Position.X, in{1}.Pose.Position.Y, in{1}.Pose.Position.Z];
        
    %write to navData
    tVec = reshape(tformMat, [1, 16]);
    msgData.T_G_F(i,:) = tVec(1, 1:16);
end

