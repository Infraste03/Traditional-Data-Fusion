function egoTrajectory = helperGenerateEgoTrajectory(gpsData)
% This is a helper function and may be removed in a future release.

% Copyright 2022 The MathWorks, Inc.

geoReference = [gpsData(1).latitude gpsData(1).longitude gpsData(1).altitude];
[xEast, yNorth, zUp] = latlon2local([gpsData.latitude]',[gpsData.longitude]',[gpsData.altitude]',geoReference);
waypoints = [xEast,yNorth,zUp];

% Raw GPS data often contains noise. Smooth the GPS waypoints by using the smoothdata function.
window = round(size(waypoints,1)*0.2);
waypoints = smoothdata(waypoints,"rloess",window);

%Create the ego trajectory using the waypoints and their corresponding time of arrivals by using the waypointTrajectory  System object™.
waypoints = double([waypoints(:,1:2) zeros(size(zUp))]);
gpsTime = [gpsData.timeStamp]' - gpsData(1).timeStamp;
egoTrajectory = waypointTrajectory(waypoints,gpsTime,ReferenceFrame="ENU");
end

