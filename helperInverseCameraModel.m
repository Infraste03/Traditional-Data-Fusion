function [pos, vel, dim, yaw, posCov, velCov, dimCov, yawCov] = helperInverseCameraModel(z, zCov, measurementParameters)
% This is a helper function and may be removed in a future release.

% Copyright 2022 The MathWorks, Inc.

sensorPose = measurementParameters.SensorPose;
egoPose = measurementParameters.EgoPose;
camIndex = measurementParameters.CameraIndex;
camera = getMonoCamera(camIndex,sensorPose);
[pos, vel, dim, yaw, posCov, velCov, dimCov, yawCov] = computeInverseCameraMeasurement(camera, z, zCov);
[pos, vel, dim, yaw, posCov, velCov, dimCov, yawCov] = transformBackward(pos, vel, dim, yaw, egoPose, posCov, velCov, dimCov, yawCov);
end

function [pos, vel, dim, yaw, posCov, velCov, dimCov, yawCov] = computeInverseCameraMeasurement(camera, z, zCov)
L = 4.7;
W = 1.8;
H = 1.4;
u = z(1) + z(3)/2;
v = z(2) + z(4);
pos2D = imageToVehicle(camera,[u v]);
pos = [pos2D(:);H/2];
vel = zeros(3,1);
dim = [L;W;H];
yaw = 0;
posCov = 5*eye(3);
velCov = blkdiag(900,900,10);
dimCov = 0.1*eye(3);
yawCov = 180^2;
end
