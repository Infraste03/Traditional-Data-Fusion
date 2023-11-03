function zExp = helperCameraModel(pos, vel, dim, yaw, measurementParameters)
% This is a helper function and may be removed in a future release.

% Copyright 2022 The MathWorks, Inc.

sensorPose = measurementParameters.SensorPose;
egoPose = measurementParameters.EgoPose;
[pos, ~, dim, yaw] = transformForward(pos, vel, dim, yaw, egoPose);
camIdx = measurementParameters.CameraIndex;
camera = getMonoCamera(camIdx, sensorPose);
zExp = zeros(7,size(pos,2));
for i = 1:size(pos,2)
    zExp(:,i) = computeCameraMeasurement(camera,pos(:,i),vel(:,i),dim(:,i),yaw(i));
end
end

function zExpi = computeCameraMeasurement(camera, pos, vel, dim, yaw)
v = [0.5000   -0.5000    0.5000
    0.5000    0.5000    0.5000
    -0.5000    0.5000    0.5000
    -0.5000   -0.5000    0.5000
    0.5000   -0.5000   -0.5000
    0.5000    0.5000   -0.5000
    -0.5000    0.5000   -0.5000
    -0.5000   -0.5000   -0.5000];
v = v.*dim(:)';
orient = [cosd(yaw) -sind(yaw) 0;sind(yaw) cosd(yaw) 0;0 0 1];
v = v*orient';
v = v + pos(:)';
R = eul2rotm(deg2rad([camera.Yaw camera.Pitch camera.Roll]));
p = [camera.SensorLocation camera.Height];
vCamera = v*R + p;
[az,el] = cart2sph(vCamera(:,1),vCamera(:,2),vCamera(:,3));
[azFov, elFov] = computeFieldOfView(camera.Intrinsics.FocalLength,camera.Intrinsics.ImageSize);
inside = abs(az) < 1.5*azFov/2 & abs(el) < 1.5*elFov/2;
if any(inside)
    cameraPoints = vehicleToImage(camera,v + [0 0 0.3158]);
    cameraPoints(:,1) = max(0,min(camera.Intrinsics.ImageSize(2),cameraPoints(:,1)));
    cameraPoints(:,2) = max(0,min(camera.Intrinsics.ImageSize(1),cameraPoints(:,2)));
    u = min(cameraPoints(:,1));
    v = min(cameraPoints(:,2));
    w = max(cameraPoints(:,1)) - min(cameraPoints(:,1));
    h = max(cameraPoints(:,2)) - min(cameraPoints(:,2));
else
    u = 0;
    v = 0;
    h = 0;
    w = 0;
end

% We assumed that the bottom center lies on the ground. Enforce it using a
% pseudo measurement.
dGround = pos(3) - dim(3)/2 + 0.3158;
zExpi = [u;v;w;h;dGround;0;0];
end

function [azFov, elFov] = computeFieldOfView(focalLength, imageSize)
    azFov = 2*atan(imageSize(2)/(2*focalLength(1)));
    elFov = 2*atan(imageSize(1)/(2*focalLength(2)));
end