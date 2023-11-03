function [pos, vel, dim, yaw, posCov, velCov, dimCov, yawCov] = helperInverseLidarModel(z, zCov, measurementParameters)
% This is a helper function and may be removed in a future release.

% Copyright 2022 The MathWorks, Inc.

sensorPose = measurementParameters.SensorPose;
egoPose = measurementParameters.EgoPose;
[pos, vel, dim, yaw, posCov, velCov, dimCov, yawCov] = computeInverseLidarMeasurement(z, zCov);
[pos, vel, dim, yaw, posCov, velCov, dimCov, yawCov] = transformBackward(pos, vel, dim, yaw, sensorPose, posCov, velCov, dimCov, yawCov);
[pos, vel, dim, yaw, posCov, velCov, dimCov, yawCov] = transformBackward(pos, vel, dim, yaw, egoPose, posCov, velCov, dimCov, yawCov);
end

function [pos, vel, dim, yaw, posCov, velCov, dimCov, yawCov] = computeInverseLidarMeasurement(z, zCov)
pos = z(1:3);
vel = zeros(3,1);
yaw = z(4);
dim = z(5:7);
posCov = zCov(1:3,1:3);
velCov = blkdiag(900,900,10);
dimCov = zCov(5:7,5:7);
yawCov = zCov(4,4);
end