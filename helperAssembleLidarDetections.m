function detections = helperAssembleLidarDetections(lidarBox, lidarPose, time, sensorIdx,egoPose)
% helperAssembleLidarDetections transform lidar bounding box detections into
% objectDetection format.

% This is a helper function and may be removed in a future release.

% Copyright 2022 The MathWorks, Inc.

if nargin<5
    egoPose = struct;
end

measurementParameters.SensorType = 1;
measurementParameters.SensorPose = lidarPose;
measurementParameters.EgoPose = egoPose;
measurementParameters.CameraIndex = 0; % Dummy;

n = size(lidarBox,1);

sampleDetection = objectDetection(time, zeros(7,1),...
    'SensorIndex',sensorIdx,...
    'MeasurementNoise',1/(2*pi),...
    'MeasurementParameters',measurementParameters,...
    'ObjectAttributes',struct);

detections = repmat({sampleDetection},n,1);

for i = 1:size(lidarBox,1)
    detections{i}.Measurement(:) = lidarBox(i,[1:3,9,4:6]);
    detections{i}.MeasurementNoise = blkdiag(0.25,0.25,0.25,4,0.25,0.25,0.25);
end

end