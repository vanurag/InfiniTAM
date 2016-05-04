function [ mocapData ] = MocapInfo(bag, topic)

%check inputs
validateattributes(topic,{'char'},{'vector'});

%topics
mocapData.topic = topic;

%filter bag
bag = select(bag,'Topic',topic);

%timestamaps
mocapData.times = bag.MessageList.Time;

%preallocate memory -> Marker Frame to Inertial Frame
mocapData.T_G_M = zeros(length(mocapData.times), 16);

for i = 1:length(mocapData.times)
    if(mod(i,1000) == 0)
        UpdateMessage('Reading Transform for vicon transform %i of %i', i, length(mocapData.times));
    end
    
    in = readMessages(bag, i);
    assignin('base', 'msgs', in);
%     assignin('base', 'debug', in);
    
    tformMat = eye(4);
    tformMat(1:3,1:3)  = quat2rotm([in{1}.Transform.Rotation.W, in{1}.Transform.Rotation.X, in{1}.Transform.Rotation.Y, in{1}.Transform.Rotation.Z]);
    tformMat(1:3,4) = [in{1}.Transform.Translation.X, in{1}.Transform.Translation.Y, in{1}.Transform.Translation.Z];
%     tformMat(1:3,1:3)  = quat2rot([in{1}.Pose.Orientation.W, in{1}.Pose.Orientation.X, in{1}.Pose.Orientation.Y, in{1}.Pose.Orientation.Z]');
%     tformMat(1:3,4) = [in{1}.Pose.Position.X, in{1}.Pose.Position.Y, in{1}.Pose.Position.Z];
        
    %write to navData
    tVec = reshape(tformMat, [1, 16]);
    mocapData.T_G_M(i,:) = tVec(1, 1:16);
end

