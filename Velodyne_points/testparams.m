%% A Practical Obstacle Detection System for Autonomous Orchard Vehicle 
% This script uses a method developed in the Article "A Practical Obstacle 
% Detection System for Autonomous Orchard Vehicle"

%% Environment Setup
clear;  %Clean the WorkSpace
clc;    %Clean the Command Window

%% Loading of the LiDAR Scans (.mat file) & Definition of Useful Parameters

load('velodyne_points.mat');
EULER_ANGLES_FIRST  = [0 0 0]; %First transformation's Euler angles(OL->OV)
EULER_ANGLES_SECOND = [0 0 0]; %Second transformation's Euler angles(OV->OI)
vehicleDims = vehicleDimensions(1,1,1); 
MOUNT_LOCATION = [... 
    vehicleDims.Length/2 - vehicleDims.RearOverhang, ... 
    0, ...                                               
    vehicleDims.Height];      %First transformation's displacement                                
SECOND_TRANSLATION = [0 0 0]; %Second transformation's displacement
OBSTACLE_COLOR_RGB = [255 165 0]; %It is a type of Orange
GROUND_COLOR_RGB =   [255 255 255];  %It is a type of Blue
ROI_DIM = 5;     %Dimension of the interesting area
ALPHA_VALUE = 18; %18
H_VALUE = 0.03;    %0.2 %0.5
MIN_DISTANCE_CLUSTERS = 1; %Min distance between points from two clusters
MIN_POINTS_CLUSTERS = 4; %Set the minimum number of points per cluster (10)
POINT_CLOUD_COLOR = [255 199 64];
ROI = [-10 10 -10 10 -2 1];
X_LIMITS_VIEW = [-60 60]; 
Y_LIMITS_VIEW = [-50 50];
Z_LIMITS_VIEW = [-10 20];

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

% for scanIndex=1:size(velo_points,1)
    
    tic
    %Steps to create the PointCloud of the Outdoor Environment
    rawScan = velo_points{1,1};
    scan = readXYZ(rawScan);
    scan = getXYZofSearchArea(scan,-10,10,-10,10,-2,1);
    PointCloud = pointCloud(scan);
    
    %The PointCloud is downsampled
    PointCloud = pcdownsample(PointCloud, 'gridAverage', 0.25);
    
%     %Change of reference frame  
%     firstRotation = eul2rotm(EULER_ANGLES_FIRST,'ZYZ');
%     firstTransformation = rigid3d(firstRotation,MOUNT_LOCATION);
%     PointCloud = pctransform(PointCloud,firstTransformation);
%     secondRotation = eul2rotm(EULER_ANGLES_SECOND,'ZYZ');
%     secondTransformation = rigid3d(secondRotation,SECOND_TRANSLATION);
%     PointCloud = pctransform(PointCloud,secondTransformation);
    
    %Consider only the points inside a parallelepiped specified using ROI
    pointIndices = findPointsInROI(PointCloud,ROI);
    likelyObstacle = zeros(1,numel(pointIndices));
    arrayIndex = 0;
    for element = 1:numel(pointIndices)
        firstPoint = select(PointCloud,pointIndices(element));
        [closeElementsIndices,] = findNearestNeighbors(PointCloud,firstPoint.Location,2);
        secondPoint = select(PointCloud,closeElementsIndices(2));
        if ( abs(secondPoint.Location(3)-firstPoint.Location(3))/sqrt( ...
                (secondPoint.Location(1)-firstPoint.Location(1))^2+ ...
                (secondPoint.Location(2)-firstPoint.Location(2))^2)>tand(ALPHA_VALUE)) ...
            || (abs(secondPoint.Location(3)-firstPoint.Location(3))>=H_VALUE)
             likelyObstacle(arrayIndex +1) = pointIndices(element);
             likelyObstacle(arrayIndex +2) = closeElementsIndices(2);
             arrayIndex=arrayIndex+2;
         end
    end
    likelyObstacle = likelyObstacle(:,1:arrayIndex);
    likelyObstaclePointCloud = select(PointCloud,likelyObstacle);
%     pcshow(likelyObstaclePointCloud);
%     groundPointCloud = select(PointCloud,allIndices);
    %After the likely-obstacle points are selected, clusterization is
    %performed   
    [clustersLabel,numberOfClusters] = pcsegdist(likelyObstaclePointCloud, ...
                                       MIN_DISTANCE_CLUSTERS,              ...
                                       'NumClusterPoints',                 ...
                                       MIN_POINTS_CLUSTERS);
    idxValidPoints = find(clustersLabel);
    labelColorIndex = clustersLabel(idxValidPoints);
    segmentedPtCloud = select(likelyObstaclePointCloud,idxValidPoints);

    %Base PointCloud is coloured in Blue
    pointscolor=uint8(zeros(PointCloud.Count,3));
    pointscolor(:,1)=GROUND_COLOR_RGB(1);
    pointscolor(:,2)=GROUND_COLOR_RGB(2);
    pointscolor(:,3)=GROUND_COLOR_RGB(3);
    PointCloud.Color=pointscolor;

    %Obstacle PointCloud is coloured in Orange
    pointscolor=uint8(zeros(segmentedPtCloud.Count,3));
    pointscolor(:,1)=OBSTACLE_COLOR_RGB(1);
    pointscolor(:,2)=OBSTACLE_COLOR_RGB(2);
    pointscolor(:,3)=OBSTACLE_COLOR_RGB(3);
    segmentedPtCloud.Color=pointscolor;

    %The two PointClouds are merged. This function  returns a merged point 
    %cloud using a box grid filter. gridStep specifies the size of the 3-D 
    %box for the filter.
    ptCloudOut = pcmerge(segmentedPtCloud, PointCloud, 1);
    
    %Show the merged PointClouds in the PcPlayer
    view(scansViewer, ptCloudOut);
    
    %colormap(hsv(numberOfClusters))
    view(scansViewerClustered, segmentedPtCloud.Location,labelColorIndex );
    toc
    
% end

function searchAreaScan = getXYZofSearchArea(scan, xMin, xMax, yMin, yMax, zMin, zMax)
    searchAreaScan = scan;
    for i=1:size(searchAreaScan,1) 
            if ~(searchAreaScan(i, 1)>=xMin && ...
                 searchAreaScan(i, 1)<=xMax && ...
                 searchAreaScan(i, 2)>=yMin && ...
                 searchAreaScan(i, 2)<=yMax && ...
                 searchAreaScan(i, 3)>=zMin && ...
                 searchAreaScan(i, 3)<=zMax)
             
                searchAreaScan(i,1)=0;
                searchAreaScan(i,2)=0;
                searchAreaScan(i,3)=0;
            end
    end
    searchAreaScan = searchAreaScan(any(searchAreaScan,2),:);
end