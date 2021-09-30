%% Implementation of an Obstacle Detection Algorithm based on SMRF Method
% This script uses a function available on Matlab which is described in 
% https://www.sciencedirect.com/science/article/abs/pii/S0924271613000026?via%3Dihub.

%% Environment Setup
clear;  %Clean the WorkSpace
clc;    %Clean the Command Window

%% Loading of the LiDAR Scans (.mat file) & Definition of Useful Parameters

load('simple_obstacles_1.mat');
OBSTACLE_COLOR_RGB = [255 206 0]; %It is a type of Yellow
GROUND_COLOR_RGB   = [0 85 155];  %It is a type of Blue
ELEVATION_THRESHOLD = 0.1;        
ELEVATION_SCALE = 0.25;
X_LIMITS_VIEW = [-80 80]; 
Y_LIMITS_VIEW = [-80 80];
Z_LIMITS_VIEW = [-20 20];
SEARCH_AREA_LIMITS = [-10 10 -10 10 -2 0];
MIN_DISTANCE_CLUSTERS = 1; %Min distance between points from two clusters
MIN_POINTS_CLUSTERS = 12; %Set the minimum number of points per cluster (10)
ROVER_RADIUS = 2.5;

%% Definition of PcPlayers to show the Obstacle Detection results

scansViewer = pcplayer(X_LIMITS_VIEW, Y_LIMITS_VIEW, Z_LIMITS_VIEW);
xlabel(scansViewer.Axes, 'x [m]')
ylabel(scansViewer.Axes, 'y [m]')
zlabel(scansViewer.Axes, 'z [m]')

clusteredScansViewer = pcplayer(X_LIMITS_VIEW, Y_LIMITS_VIEW, Z_LIMITS_VIEW);
xlabel(clusteredScansViewer.Axes, 'x [m]')
ylabel(clusteredScansViewer.Axes, 'y [m]')
zlabel(clusteredScansViewer.Axes, 'z [m]')

%% For Loop that carries out the Segmentation Task

ExecutionTimeArray = zeros(size(velo_msgs,1),1);

for scanIndex=1:size(velo_msgs,1) 

    tic
    %Steps to create the PointCloud of the Outdoor Environment
    frame = readXYZ(velo_msgs{scanIndex,1});
    PointCloud = pointCloud(frame);  
    
    %Call of the function that segments ground from non-ground points
    [~,nonGroundPtCloud,groundPtCloud] = segmentGroundSMRF(...,
                              PointCloud, ...
                              'ElevationThreshold', ELEVATION_THRESHOLD,...
                              'ElevationScale',     ELEVATION_SCALE);
                          
    %After the likely-obstacle points are selected, clusterization is
    %performed   
    [clustersLabel,numberOfClusters] = pcsegdist(nonGroundPtCloud, ...
                                       MIN_DISTANCE_CLUSTERS,              ...
                                       'NumClusterPoints',                 ...
                                       MIN_POINTS_CLUSTERS);
    idxValidPoints = find(clustersLabel);
    labelColorIndex = clustersLabel(idxValidPoints);
    segmentedPtCloud = select(nonGroundPtCloud,idxValidPoints);

    %Ground PointCloud is coloured in Blue                      
    groundPtCloud.Color= pickColorsGround(groundPtCloud, GROUND_COLOR_RGB);

    %Non-Ground PointCloud is coloured in Yellow
    segmentedPtCloud.Color=pickColorsObstacles(segmentedPtCloud,OBSTACLE_COLOR_RGB);

    %The two PointClouds are merged. This function  returns a merged point 
    %cloud using a box grid filter. gridStep specifies the size of the 3-D 
    %box for the filter.
    PointCloudMerged = pcmerge(segmentedPtCloud, groundPtCloud, 1);
    
    %Show the merged PointClouds in the PcPlayer
    view(scansViewer,PointCloudMerged);
    
    if segmentedPtCloud.Count ~= 0 
        view(clusteredScansViewer, segmentedPtCloud.Location,labelColorIndex);
    end
    
    boundariesOfCuboids = fitCuboidAroundObstacles(nonGroundPtCloud, clustersLabel, numberOfClusters);
    showShape('cuboid', boundariesOfCuboids,'Parent',scansViewer.Axes, ...
              'Color','green','Opacity',0.1);
    drawnow;
    
    ExecutionTimeArray(scanIndex,1) = toc;

end

standardDeviation=std(ExecutionTimeArray);
mean=mean(ExecutionTimeArray);