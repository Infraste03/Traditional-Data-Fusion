% main file data fusion 
% This code is used to define the path to a zip file that contains some data
dataFolder = tempdir;
dataFileName = "PandasetLidarCameraData.zip";
url = "https://ssd.mathworks.com/supportfiles/driving/data/" + dataFileName;
filePath = fullfile(dataFolder,dataFileName);

% Downloads a file from a URL if it does not already exist in the specified file path.
% 
% Inputs:
% - filePath: a string representing the file path where the file should be saved.
% - url: a string representing the URL from where the file should be downloaded.
%
if ~isfile(filePath)
    websave(filePath,url);
end


% This code unzips a file located at 'filePath' to a folder named 'dataFolder'.
% Then, it loads a .mat file containing lidar and camera data from the Pandaset dataset.
% The file is located at 'dataPath' and its name is determined by the variable 'fileName'.
% The loaded data is stored in the variable 'dataLog'.
unzip(filePath,dataFolder)
dataPath = fullfile(dataFolder,"PandasetLidarCameraData") ;
fileName = fullfile(dataPath,strcat(num2str(1,"%03d"),".mat"));
load(fileName,"dataLog");


% This code sets the timestamp offset to the timestamp of the Lidar data in the data log.
tOffset = dataLog.LidarData.Timestamp;

% This code extracts lidar data from a data log and displays it using pcshow.
% 
% INPUTS:
%   - dataLog: a data log containing lidar data
%
% OUTPUTS:
%   - ptCld: point cloud data extracted from the lidar data
%   - lidarBboxes: bounding boxes of the detected objects in the lidar data
[ptCld,lidarBboxes] = helperExtractLidarData(dataLog);
ax = pcshow(ptCld);


% This code block shows a cuboid shape with pink color, opacity of 0.15 and line width of 1 around the lidar bounding boxes on the given axes. The axes are then zoomed in by a factor of 8.

showShape("Cuboid",lidarBboxes,Color="magenta",Parent=ax,Opacity=0.15,LineWidth=1);
zoom(ax,8)

% This code extracts camera data from a data log and folder for a specific camera index.
% 
% INPUTS:
% - dataLog: data log file path
% - dataFolder: data folder path
% - camIdx: camera index
%
% OUTPUTS:
% - img: camera image
% - cameraBBoxes: camera bounding boxes

camIdx = 1; 
[img,cameraBBoxes] = helperExtractCameraData(dataLog,dataFolder,camIdx);

% Insert bounding boxes around detected vehicles in an image and display it.
% 
% img: input image
% cameraBBoxes: bounding boxes around detected vehicles
% 
% Returns:
% img: output image with bounding boxes around detected vehicles
img = insertObjectAnnotation(img,"Rectangle",cameraBBoxes,"Vehicle");
imshow(img)


% Setup the tracker 
% The tracker is used to fuse lidar and camera detections to track vehicles.
tracker = trackerJPDA( ...
    TrackLogic="Integrated",...
    FilterInitializationFcn=@helperInitLidarCameraFusionFilter,...
    AssignmentThreshold=[20 200],...
    MaxNumTracks=500,...
    DetectionProbability=0.95,...
    MaxNumEvents=50,...
    ClutterDensity=1e-7,...
    NewTargetDensity=1e-7,...
    ConfirmationThreshold=0.99,...
    DeletionThreshold=0.2,...
    DeathRate=0.5);



% Setup the visualization
% This code initializes a helperLidarCameraFusionDisplay object and assigns it to the variable 'display'.
% The object is used to visualize the results of fusing lidar and camera detections to track vehicles.
display = helperLidarCameraFusionDisplay;

% Load GPS data
% This code loads GPS data from a .mat file located in the specified data path.

fileName = fullfile(dataPath,"gpsData.mat");
load(fileName,"gpsData");

% Create ego trajectory from GPS data using waypointTrajectory
% This code generates an ego trajectory based on GPS data.
% 
% INPUTS:
% - gpsData: a matrix containing GPS data
%
% OUTPUTS:
% - egoTrajectory: a matrix containing the generated ego trajectory

egoTrajectory = helperGenerateEgoTrajectory(gpsData);


% Set the number of frames to process from the dataset
numFrames = 79;

% This script loads lidar and camera data, extracts object detections from them,
% and tracks the detected objects using a trackingKF object. The script then
% visualizes the results. The lidar and camera data are loaded from MAT files
% in a specified folder. The ego pose is obtained from GPS data and is used to
% transform the detections into the global coordinate frame. The script uses
% helper functions to extract and assemble the data, and to visualize the results.
% 
for frame = 1:numFrames   
    % Load the data
    fileName = fullfile(dataPath,strcat(num2str(frame,"%03d"),".mat"));
    load(fileName,'dataLog');

    % Find current time
    time = dataLog.LidarData.Timestamp - tOffset;

    % Update ego pose using GPS data to track in global coordinate frame
    [pos, orient, vel] = egoTrajectory.lookupPose(time);
    egoPose.Position = pos;
    egoPose.Orientation = eulerd(orient,"ZYX","frame");
    egoPose.Velocity = vel;

    % Assemble lidar detections into objectDetection format
    [~, lidarBoxes, lidarPose] = helperExtractLidarData(dataLog);
    lidarDetections = helperAssembleLidarDetections(lidarBoxes,lidarPose,time,1,egoPose);

    % Assemble camera detections into objectDetection format
    cameraDetections = cell(0,1);
    for k = 1:1:numel(dataLog.CameraData)
        [img, camBBox,cameraPose] = helperExtractCameraData(dataLog, dataFolder,k);
        cameraBoxes{k} = camBBox; %#ok<SAGROW>
        thisCameraDetections = helperAssembleCameraDetections(cameraBoxes{k},cameraPose,time,k+1,egoPose);
        cameraDetections = [cameraDetections;thisCameraDetections]; %#ok<AGROW> 
    end

    % Concatenate detections
    detections = [lidarDetections;cameraDetections];
   
    % Run the tracker
    [confirmed,tentative,tracks,info] = tracker(detections, time);

    % Visualize the results
    display(dataFolder,dataLog, egoPose, lidarDetections, cameraDetections, tracks);

   
    for i = 1:1:length(info.Clusters)
        disp('---- Cluster number: ')
        disp(i)
        disp('--------- Info Cluster: ')
        disp(info.Clusters{i})
        disp('--------- Cluster, Validation matrix of the cluster: ')
        disp(info.Clusters{i}.ValidationMatrix)
        disp('--------- Cluster, Individual association likelihoods based on kinematic information: ')
        disp(info.Clusters{i}.Likelihood)
        disp('--------- Cluster, Matrix of marginal posterior joint association probabilities: ')
        disp(info.Clusters{i}.MarginalProbabilities)
        
    end

    disp('-----------------------------')
    
end