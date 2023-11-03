function z = helperLidarModel(pos, vel, dim, yaw, measurementParameters)
% This is a helper function and may be removed in a future release.

% Copyright 2022 The MathWorks, Inc.


% state is defined as [x;vx;y;vy;w;z;vz;theta;L;W;H];
% lidar measurement is defined as [x;y;z;theta;L;W;H];
sensorPose = measurementParameters.SensorPose;
egoPose = measurementParameters.EgoPose;
[pos, vel, dim, yaw] = transformForward(pos, vel, dim, yaw, egoPose);
[pos, ~, dim, yaw] = transformForward(pos, vel, dim, yaw, sensorPose);
z = zeros(7,size(pos,2),'like',pos);
z(1:3,:) = pos;
z(4,:) = yaw;
z(5:7,:) = dim;
end
