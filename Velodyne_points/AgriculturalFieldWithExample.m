%% Implementation of an Obstacle Detection Algorithm based on an Example
% This script uses apply a method found in an example of Matlab to our data
% sample. The original script can be found: 
% https://it.mathworks.com/help/driving/ug/ground-plane-and-obstacle-detection-using-lidar.html

%% Environment Setup
clear;  %Clean the WorkSpace
clc;    %Clean the Command Window

%% Ground Plane and Obstacle Detection Using Lidar
% This example shows how to process 3-D lidar data from a sensor mounted on
% a vehicle by segmenting the ground plane and finding nearby obstacles.
% This can facilitate drivable path planning for vehicle navigation. The
% example also shows how to visualize streaming lidar data.
%
% Copyright 2016-2018 The MathWorks, Inc.

%% Loading of the LiDAR Scans (.mat file) & Definition of Useful Parameters

load('velodyne_points.mat');
X_LIMITS_VIEW = [-15 15]; 
Y_LIMITS_VIEW = [-15 15];
Z_LIMITS_VIEW = [-5 10];
X_SENSOR_LOCATION = 0; %0
Y_SENSOR_LOCATION = 0; %0
Z_SENSOR_LOCATION = 0; %0
ELEVATION_DELTA = 10; %10
RADIUS_SENSOR_OBSTACLE = 8; %40
SEARCH_AREA_LIMITS = [-10 10 -10 10 -2 0];
MIN_DISTANCE_CLUSTERS = 1; %Min distance between points from two clusters
MIN_POINTS_CLUSTERS = 12; %Set the minimum number of points per cluster (12)
ROVER_RADIUS = 2.5;

%% Definition of a PcPlayer to show the Obstacle Detection results

scansViewer = pcplayer(X_LIMITS_VIEW, Y_LIMITS_VIEW, Z_LIMITS_VIEW);
xlabel(scansViewer.Axes, 'x [m]')
ylabel(scansViewer.Axes, 'y [m]')
zlabel(scansViewer.Axes, 'z [m]')

scansViewerClustered = pcplayer(X_LIMITS_VIEW, Y_LIMITS_VIEW, Z_LIMITS_VIEW);
xlabel(scansViewerClustered.Axes, 'x [m]')
ylabel(scansViewerClustered.Axes, 'y [m]')
zlabel(scansViewerClustered.Axes, 'z [m]')

%% Code to get the LiDAR scan

%Steps to create the PointCloud of the Outdoor Environment
frame = readXYZ(velo_points{1,1});
searchArea = pickSearchArea(frame, SEARCH_AREA_LIMITS, ROVER_RADIUS);
PointCloud = pointCloud(searchArea); 

%Steps to create an organized PointCloud from an unorganized one (knowing
%the datasheet of the Laser Scanner
vResolution = 16;       
hResolution = 512;     
vFOVUp = 15;     
vFOVDown = -15;    
hFOV = 360;
beamConfig = 'Uniform';
if strcmp(beamConfig,'Uniform')
    vbeamAngles = linspace(vFOVUp,vFOVDown,vResolution);
end
hbeamAngles = linspace(0,hFOV,hResolution);
PointCloud = convertUnorgToOrg(PointCloud,vResolution,hResolution,vbeamAngles,hbeamAngles);

%% Definition of the ColorMap

% In this example, we will be segmenting points belonging to the ground
% plane, the ego vehicle and nearby obstacles. Set the colormap for
% labeling these points.

% Define labels to use for segmented points
colorLabels = [...
    0      0.4470 0.7410; ... % Unlabeled points, specified as [R,G,B]
    0.4660 0.6740 0.1880; ... % Ground points
    0.9290 0.6940 0.1250; ... % Ego points
    0.6350 0.0780 0.1840];    % Obstacle points

% Define indices for each label
colors.Unlabeled = 1;
colors.Ground    = 2;
colors.Ego       = 3;
colors.Obstacle  = 4;

% Set the colormap
colormap(scansViewer.Axes, colorLabels)

%% Segment the Ego Vehicle
% The lidar is mounted on top of the vehicle, and the point cloud may
% contain points belonging to the vehicle itself, such as on the roof or
% hood. Knowing the dimensions of the vehicle, we can segment out points
% that are closest to the vehicle.

% Create a |<docid:driving_ref#mw_daf540f5-2511-4100-829d-9d152ed13cf2
% vehicleDimensions>| object for storing dimensions of the vehicle.
vehicleDims = vehicleDimensions(1,1,1.5); 

% Specify the mounting location of the lidar in the vehicle coordinate
% system. The vehicle coordinate system is centered at the center of the
% rear-axle, on the ground, with positive X direction pointing forward,
% positive Y towards the left, and positive Z upwards. In this example, the
% lidar is mounted on the top center of the vehicle, parallel to the
% ground. 

mountLocation = [...
    vehicleDims.Length/2 - vehicleDims.RearOverhang, ... % x
    0, ...                                               % y
    vehicleDims.Height];                                 % z

% Segment the ego vehicle using the helper function
% |helperSegmentEgoFromLidarData|. This function segments all points within
% the cuboid defined by the ego vehicle. Store the segmented points in a
% struct |points|.
points = struct();
points.EgoPoints = helperSegmentEgoFromLidarData(PointCloud, vehicleDims, mountLocation);

% Visualize the point cloud with segmented ego vehicle. Use the
% |helperUpdateView| helper function.
closePlayer = false;
helperUpdateView(scansViewer, PointCloud, points, colors, closePlayer);

%% Segment Ground Plane 
% In order to identify obstacles from the lidar data, first segment the
% ground plane using the
% |<docid:vision_ref#mw_0b012529-aa6a-40ef-993b-23eec3d6a6da
% segmentGroundFromLidarData>| function to accomplish this. This function
% segments points belonging to ground from organized lidar data.
points.GroundPoints = segmentGroundFromLidarData(PointCloud, 'ElevationAngleDelta', ELEVATION_DELTA);

% Visualize the segmented ground plane.
helperUpdateView(scansViewer, PointCloud, points, colors, closePlayer);

%% Segment Nearby Obstacles
% Remove points belonging to the ego vehicle and the ground plane by using
% the |<docid:vision_ref#buph2kw-1 select>| function on the point cloud.
% Specify the |'OutputSize'| as |'full'| to retain the organized nature of
% the point cloud.
nonEgoGroundPoints = ~points.EgoPoints & ~points.GroundPoints;
ptCloudSegmented = select(PointCloud, nonEgoGroundPoints, 'OutputSize', 'full');

% Next, segment nearby obstacles by looking for all points that are not
% part of the ground or ego vehicle within some radius from the ego
% vehicle. This radius can be determined based on the range of the lidar
% and area of interest for further processing.
sensorLocation  = [X_SENSOR_LOCATION, Y_SENSOR_LOCATION, Z_SENSOR_LOCATION]; % Sensor is at the center of the coordinate system
radius          = RADIUS_SENSOR_OBSTACLE; % meters

points.ObstaclePoints = findNeighborsInRadius(ptCloudSegmented, ...
    sensorLocation, radius);

% Visualize the segmented obstacles
helperUpdateView(scansViewer, PointCloud, points, colors, closePlayer);

%% Execute the previous Steps for Each LiDAR Scan

isPlayerOpen = true;
ExecutionTimeArray = zeros(size(velo_points,1),1);
SegmentationExecutionTimeArray = zeros(size(velo_points,1),1);
ClusterizationExecutionTimeArray = zeros(size(velo_points,1),1);
AddBoundariesToGUIExecutionTimeArray = zeros(size(velo_points,1),1);

for scanIndex=1:size(velo_points,1) 

    mainTime = tic;
    %Steps to create the PointCloud of the Outdoor Environment
    frame = readXYZ(velo_points{scanIndex,1});
    searchArea = pickSearchArea(frame, SEARCH_AREA_LIMITS, ROVER_RADIUS);
    PointCloud = pointCloud(searchArea); 
    
    %Steps to create an organized PointCloud from an unorganized one (knowing
    %the datasheet of the Laser Scanner
    PointCloud = convertUnorgToOrg(PointCloud,vResolution,hResolution,vbeamAngles,hbeamAngles);
    
    segmentTime = tic;
    % Segment points belonging to the ego vehicle
    points.EgoPoints = helperSegmentEgoFromLidarData(PointCloud, vehicleDims, mountLocation);
    
    % Segment points belonging to the ground plane
    points.GroundPoints = segmentGroundFromLidarData(PointCloud, 'ElevationAngleDelta', ELEVATION_DELTA);
    
    % Remove points belonging to the ego vehicle and ground plane
    nonEgoGroundPoints = ~points.EgoPoints & ~points.GroundPoints;
    ptCloudSegmented = select(PointCloud, nonEgoGroundPoints, 'OutputSize', 'full');
    
    % Segment obstacles
    points.ObstaclePoints = findNeighborsInRadius(ptCloudSegmented, sensorLocation, radius);
    
    [indices, ~] = findNeighborsInRadius(ptCloudSegmented, sensorLocation, radius);
    maybeObstacles = select(ptCloudSegmented, indices);
    SegmentationExecutionTimeArray(scanIndex,1) = toc(segmentTime); 
    
    clusterizationTime = tic;
    [clustersLabel,numberOfClusters] = pcsegdist(maybeObstacles, ...
                                       MIN_DISTANCE_CLUSTERS,              ...
                                       'NumClusterPoints',                 ...
                                       MIN_POINTS_CLUSTERS);
    ClusterizationExecutionTimeArray(scanIndex,1) = toc(clusterizationTime);  
    idxValidPoints = find(clustersLabel);
    labelColorIndex = clustersLabel(idxValidPoints);
    segmentedPtCloud = select(maybeObstacles,idxValidPoints);
    
    %colormap(hsv(numberOfClusters))
    if segmentedPtCloud.Count ~= 0 
        view(scansViewerClustered, segmentedPtCloud.Location,labelColorIndex );
    end
    
    % Update lidar display
    isPlayerOpen = helperUpdateView(scansViewer, PointCloud, points, colors, closePlayer);
    
    addBoundariesToGUITime = tic;
    boundariesOfCuboids = fitCuboidAroundObstacles(maybeObstacles, clustersLabel, numberOfClusters);
    showShape('cuboid', boundariesOfCuboids,'Parent',scansViewer.Axes, ...
              'Color','green','Opacity',0.1);
    drawnow;
    AddBoundariesToGUIExecutionTimeArray(scanIndex,1) = toc(addBoundariesToGUITime); 
    
    ExecutionTimeArray(scanIndex,1) = toc(mainTime);
    
end

snapnow
standardDeviation=std(ExecutionTimeArray);
meanExecutionTime=mean(ExecutionTimeArray);

propSegmentTime = mean(SegmentationExecutionTimeArray./ExecutionTimeArray)*100;
propClusterizationTime = mean(ClusterizationExecutionTimeArray./ExecutionTimeArray)*100;
propAddBoundariesTime = mean(AddBoundariesToGUIExecutionTimeArray./ExecutionTimeArray)*100;

%% Supporting Functions
% |helperSegmentEgoFromLidarData| segments points belonging to the ego
% vehicle given the dimensions of the vehicle and mounting location.
function egoPoints = helperSegmentEgoFromLidarData(ptCloud, vehicleDims, mountLocation)
%helperSegmentEgoFromLidarData segment ego vehicle points from lidar data
%   egoPoints = helperSegmentEgoFromLidarData(ptCloud,vehicleDims,mountLocation)
%   segments points belonging to the ego vehicle of dimensions vehicleDims
%   from the lidar scan ptCloud. The lidar is mounted at location specified
%   by mountLocation in the vehicle coordinate system. ptCloud is a
%   pointCloud object. vehicleDimensions is a vehicleDimensions object.
%   mountLocation is a 3-element vector specifying XYZ location of the
%   lidar in the vehicle coordinate system.
%
%   This function assumes that the lidar is mounted parallel to the ground
%   plane, with positive X direction pointing ahead of the vehicle,
%   positive Y direction pointing to the left of the vehicle in a
%   right-handed system.

% Buffer around ego vehicle 
bufferZone = [0.1, 0.1, 0.1]; % in meters

% Define ego vehicle limits in vehicle coordinates
egoXMin = -vehicleDims.RearOverhang - bufferZone(1);
egoXMax = egoXMin + vehicleDims.Length + bufferZone(1);
egoYMin = -vehicleDims.Width/2 - bufferZone(2);
egoYMax = egoYMin + vehicleDims.Width + bufferZone(2);
egoZMin = 0 - bufferZone(3);
egoZMax = egoZMin + vehicleDims.Height + bufferZone(3);

egoXLimits = [egoXMin, egoXMax];
egoYLimits = [egoYMin, egoYMax];
egoZLimits = [egoZMin, egoZMax];

% Transform to lidar coordinates
egoXLimits = egoXLimits - mountLocation(1);
egoYLimits = egoYLimits - mountLocation(2);
egoZLimits = egoZLimits - mountLocation(3);

% Use logical indexing to select points inside ego vehicle cube
egoPoints = ptCloud.Location(:,:,1) > egoXLimits(1) ...
    & ptCloud.Location(:,:,1) < egoXLimits(2) ...
    & ptCloud.Location(:,:,2) > egoYLimits(1) ...
    & ptCloud.Location(:,:,2) < egoYLimits(2) ...
    & ptCloud.Location(:,:,3) > egoZLimits(1) ...
    & ptCloud.Location(:,:,3) < egoZLimits(2);
end

% |helperUpdateView| updates the streaming point cloud display with the
% latest point cloud and associated color labels.
function isPlayerOpen = helperUpdateView(lidarViewer, ptCloud, points, colors, closePlayer)
%helperUpdateView update streaming point cloud display
%   isPlayerOpen = helperUpdateView(lidarViewer, ptCloud, points, colors, closePlayers)
%   updates the pcplayer object specified in lidarViewer with a new point
%   cloud ptCloud. Points specified in the struct points are colored
%   according to the colormap of lidarViewer using the labels specified by
%   the struct colors. closePlayer is a flag indicating whether to close
%   the lidarViewer.

if closePlayer
    hide(lidarViewer);
    isPlayerOpen = false;
    return;
end
    
scanSize = size(ptCloud.Location);
scanSize = scanSize(1:2);

% Initialize colormap
colormapValues = ones(scanSize, 'like', ptCloud.Location) * colors.Unlabeled;

if isfield(points, 'GroundPoints')
    colormapValues(points.GroundPoints) = colors.Ground;
end

if isfield(points, 'EgoPoints')
    colormapValues(points.EgoPoints) = colors.Ego;
end

if isfield(points, 'ObstaclePoints')
    colormapValues(points.ObstaclePoints) = colors.Obstacle;
end

% Update view
view(lidarViewer, ptCloud.Location, colormapValues)

% Check if player is open
isPlayerOpen = isOpen(lidarViewer);

end

