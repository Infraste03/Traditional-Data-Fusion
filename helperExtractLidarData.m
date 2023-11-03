function [ptCloud, lidarBoxes, pose, time] = helperExtractLidarData(dataLog)
% This is a helper function and may be removed in a future release.

% Copyright 2022 The MathWorks, Inc.

ptCloud = dataLog.LidarData.PointCloud;
pose = dataLog.LidarData.Pose;
time = dataLog.LidarData.Timestamp;
lidarBoxes = dataLog.LidarData.Detections;

end