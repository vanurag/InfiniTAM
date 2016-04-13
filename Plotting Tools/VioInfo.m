function [ viData ] = VioInfo(bag, topic)

%check inputs
validateattributes(topic,{'char'},{'vector'});

%topics
viData.topic = topic;

%filter bag
bag = select(bag,'Topic',topic);

%timestamaps
viData.times = bag.MessageList.Time;

%preallocate memory -> Odometry frame to interial frame
viData.T_G_O = zeros(length(viData.times), 16);

for i = 1:length(viData.times)
    if(mod(i,1000) == 0)
        UpdateMessage('Reading Transform for VI transform %i of %i', i, length(viData.times));
    end
    
    in = readMessages(bag, i);
    
    tformMat = eye(4);
    or = in{1}.Pose.Pose.Orientation;
    pos = in{1}.Pose.Pose.Position;
    tformMat(1:3,1:3)  = quat2rotm([or.W, or.X, or.Y, or.Z]);
    tformMat(1:3,4) = [pos.X, pos.Y, pos.Z];

    %write to navData
    tVec = reshape(tformMat, [1, 16]);
    viData.T_G_O(i,:) = tVec(1, 1:16);
end

