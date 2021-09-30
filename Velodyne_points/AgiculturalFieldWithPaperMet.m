%% A Practical Obstacle Detection System for Autonomous Orchard Vehicle 
% This script uses a method developed in the Article "A Practical Obstacle 
% Detection System for Autonomous Orchard Vehicle"

%% Environment Setup
clear;  %Clean the WorkSpace
clc;    %Clean the Command Window

%% Loading of the LiDAR Scans (.mat file) & Definition of Useful Parameters

load('velodyne_points.mat');

% EULER_ANGLES_FIRST  = [0 0 0]; %First transformation's Euler angles(OL->OV)
% EULER_ANGLES_SECOND = [0 0 0]; %Second transformation's Euler angles(OV->OI)
% vehicleDims = vehicleDimensions(1,1,1.5); 
% MOUNT_LOCATION = [... 
%     vehicleDims.Length/2 - vehicleDims.RearOverhang, ... 
%     0, ...                                               
%     vehicleDims.Height];      %First transformation's displacement                                
% SECOND_TRANSLATION = [0 0 0]; %Second transformation's displacement

OBSTACLE_COLOR_RGB = [255 206 0]; %It is a type of Yellow
GROUND_COLOR_RGB   = [0 85 155];  %It is a type of Blue
ALPHA_VALUE = 15; %18
H_VALUE = 0.1;    %0.2 %0.5
MIN_DISTANCE_CLUSTERS = 1; %Min distance between points from two clusters
MIN_POINTS_CLUSTERS = 12; %Set the minimum number of points per cluster (10)
SEARCH_AREA_LIMITS = [-10 10 -10 10 -2 0];
X_LIMITS_VIEW = [-15 15]; 
Y_LIMITS_VIEW = [-15 15];
Z_LIMITS_VIEW = [-5 10];
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

%% For Loop that carries out the Segmentation Task

ExecutionTimeArray = zeros(size(velo_points,1),1);
SegmentationExecutionTimeArray = zeros(size(velo_points,1),1);
ClusterizationExecutionTimeArray = zeros(size(velo_points,1),1);
AddBoundariesToGUIExecutionTimeArray = zeros(size(velo_points,1),1);

for scanIndex=1:size(velo_points,1)

    mainTime = tic;
    %Steps to create the PointCloud of the Outdoor Environment
    frame = readXYZ(velo_points{scanIndex,1});
    searchArea = pickSearchArea(frame, SEARCH_AREA_LIMITS,ROVER_RADIUS);
    PointCloud = pointCloud(frame);
    
    %The PointCloud is downsampled
    PointCloud = pcdownsample(PointCloud, 'gridAverage', 0.4);

    %     %Change of reference frame  
    %     firstRotation = eul2rotm(EULER_ANGLES_FIRST,'ZYZ');
    %     firstTransformation = rigid3d(firstRotation,MOUNT_LOCATION);
    %     PointCloud = pctransform(PointCloud,firstTransformation);
    %     secondRotation = eul2rotm(EULER_ANGLES_SECOND,'ZYZ');
    %     secondTransformation = rigid3d(secondRotation,SECOND_TRANSLATION);
    %     PointCloud = pctransform(PointCloud,secondTransformation);

    segmentTime = tic;
    %Consider only the points inside a parallelepiped specified using ROI
    pointIndices = findPointsInROI(PointCloud,SEARCH_AREA_LIMITS);
    allIndices = zeros(1,numel(pointIndices));
    likelyObstacle = zeros(1,numel(pointIndices));
    arrayIndex1 = 0;
    arrayIndex2 = 0;
    for element = 1:numel(pointIndices)
        firstPoint = select(PointCloud,pointIndices(element));
        [closeElementsIndices,] = findNearestNeighbors(PointCloud,firstPoint.Location,2);
        secondPoint = select(PointCloud,closeElementsIndices(2));
        if ( abs(secondPoint.Location(3)-firstPoint.Location(3))/sqrt( ...
                (secondPoint.Location(1)-firstPoint.Location(1))^2+ ...
                (secondPoint.Location(2)-firstPoint.Location(2))^2)>tand(ALPHA_VALUE)) ...
            || (abs(secondPoint.Location(3)-firstPoint.Location(3))>=H_VALUE)
                 arrayIndex1=arrayIndex1+1;  
                 likelyObstacle(arrayIndex1) = pointIndices(element);                       
        else     
                 arrayIndex2=arrayIndex2+1; 
                 allIndices(arrayIndex2) = pointIndices(element);        
         end
    end
    likelyObstacle = likelyObstacle(likelyObstacle~=0);
    allIndices = allIndices(allIndices~=0);
    likelyObstaclePointCloud = select(PointCloud,likelyObstacle);
    groundPointCloud = select(PointCloud,allIndices);
    SegmentationExecutionTimeArray(scanIndex,1) = toc(segmentTime);
    
    clusterizationTime = tic;
    %After the likely-obstacle points are selected, clusterization is
    %performed   
    [clustersLabel,numberOfClusters] = pcsegdist(likelyObstaclePointCloud, ...
                                       MIN_DISTANCE_CLUSTERS,              ...
                                       'NumClusterPoints',                 ...
                                       MIN_POINTS_CLUSTERS);
    ClusterizationExecutionTimeArray(scanIndex,1) = toc(clusterizationTime);  
    idxValidPoints = find(clustersLabel);
    labelColorIndex = clustersLabel(idxValidPoints);
    segmentedPtCloud = select(likelyObstaclePointCloud,idxValidPoints);

    %Ground PointCloud is coloured in Blue                      
    groundPointCloud.Color= pickColorsGround(groundPointCloud, GROUND_COLOR_RGB);

    %Non-Ground PointCloud is coloured in Yellow
    segmentedPtCloud.Color=pickColorsObstacles(segmentedPtCloud,OBSTACLE_COLOR_RGB);

    %The two PointClouds are merged. This function  returns a merged point 
    %cloud using a box grid filter. gridStep specifies the size of the 3-D 
    %box for the filter.
    PointCloudMerged = pcmerge(segmentedPtCloud, groundPointCloud, 1);

    %Show the merged PointClouds in the PcPlayer
    view(scansViewer, PointCloudMerged);

    %colormap(hsv(numberOfClusters))
    if segmentedPtCloud.Count ~= 0 
        view(scansViewerClustered, segmentedPtCloud.Location,labelColorIndex );
    end 
    
    addBoundariesToGUITime = tic;
    boundariesOfCuboids = fitCuboidAroundObstacles(likelyObstaclePointCloud, clustersLabel, numberOfClusters);
    showShape('cuboid', boundariesOfCuboids,'Parent',scansViewer.Axes, ...
              'Color','green','Opacity',0.1);
    drawnow;
    AddBoundariesToGUIExecutionTimeArray(scanIndex,1) = toc(addBoundariesToGUITime);  
    
    ExecutionTimeArray(scanIndex,1) = toc(mainTime);
end

standardDeviation=std(ExecutionTimeArray);
meanExecutionTime=mean(ExecutionTimeArray);

propSegmentTime = mean(SegmentationExecutionTimeArray./ExecutionTimeArray)*100;
propClusterizationTime = mean(ClusterizationExecutionTimeArray./ExecutionTimeArray)*100;
propAddBoundariesTime = mean(AddBoundariesToGUIExecutionTimeArray./ExecutionTimeArray)*100;

 
