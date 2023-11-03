function filter = helperInitLidarCameraFusionFilter(detection)
% helperInitLidarCameraFusionFilter The function returns an Extended Kalman
% Filter to fuse camera and lidar detections.

% This is a helper function and may be removed in a future release.

% Copyright 2022 The MathWorks, Inc.

z = detection.Measurement;
zCov = detection.MeasurementNoise;
params = detection.MeasurementParameters;

if params.SensorType == 2
    z(1:4) = z(1:4)*pixelScale;
    zCov(1:4,1:4) = zCov(1:4,1:4)*pixelScale^2;
    [pos, vel, dim, yaw, posCov, velCov, dimCov, yawCov] = helperInverseCameraModel(z, zCov, params);
else
    [pos, vel, dim, yaw, posCov, velCov, dimCov, yawCov] = helperInverseLidarModel(z, zCov, params);
end

state = zeros(11,1);
stateCov = 100*eye(11);
state([1 3 6]) = pos;
state([2 4 7]) = vel;
state(8) = yaw;
state(9:11) = dim;
stateCov([1 3 6],[1 3 6]) = posCov;
stateCov([2 4 7],[2 4 7]) = velCov;
stateCov(8,8) = yawCov;
stateCov([9 10 11],[9 10 11]) = dimCov;
Q = blkdiag(10,10,3,1);

filter = trackingEKF(@stateTransitionFcn,@measurementFcn, state,...
    'StateCovariance',stateCov,...
    'ProcessNoise',Q,...
    'HasAdditiveProcessNoise',false,...
    'MeasurementNoise',zCov);

end

function state = stateTransitionFcn(state,w,dT)
if isscalar(w)
    wN = repmat(w,[4 size(state,2)]);
else
    wN = w;
end
state(1:7,:) = constturn(state(1:7,:),wN,dT);
state(8,:) = state(8,:) + state(5,:)*dT + wN(3,:)*dT^2/2;
end

function z = measurementFcn(state, params)
pos = state([1 3 6],:);
vel = state([2 4 7],:);
yaw = state(8,:);
dim = state(9:11,:);
if params.SensorType == 2
    z = helperCameraModel(pos,vel,dim,yaw,params);
    z(1:4,:) = z(1:4,:)/pixelScale();
else
    z = helperLidarModel(pos,vel,dim,yaw,params);
end

end