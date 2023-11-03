function camera = getMonoCamera(intrinsics, sensorPose)
% getMonoCamera The function returns monoCamera objects for pandaset
% cameras.

% This is a helper function and may be removed in a future release.

% Copyright 2022 The MathWorks, Inc.

camera = getMonoCameraObject(intrinsics);
camera.Height = sensorPose.Position(3) + 0.3158; % H from ground
camera.SensorLocation = sensorPose.Position(1:2);
camera.Yaw = sensorPose.Orientation(1);
camera.Pitch = sensorPose.Orientation(2);
camera.Roll = sensorPose.Orientation(3);
end

function camera = getMonoCameraObject(sensorIdx)
% This is done to avoid creating monoCamera object everytime.
persistent allMonoCameras

if isempty(allMonoCameras)
    intrinsicData = coder.const(getIntrinsicsData());
    allMonoCameras = cell(numel(intrinsicData),1);
    for i = 1:numel(intrinsicData)
        camIntrinsics = cameraIntrinsics(intrinsicData(i).FocalLength,intrinsicData(i).PrincipalPoint,intrinsicData(i).ImageSize);
        allMonoCameras{i} = monoCamera(camIntrinsics,0.5);
    end
end

camera = allMonoCameras{sensorIdx};

end