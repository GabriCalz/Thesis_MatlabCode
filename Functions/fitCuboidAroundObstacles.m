function listOfCuboids = fitCuboidAroundObstacles(PointCloud, clustersLabel, numberOfClusters)
    listOfCuboids = zeros(numberOfClusters, 9);
    for i = 1:numberOfClusters
        idx = find(clustersLabel == i);
        model = pcfitcuboid(PointCloud,idx);
        listOfCuboids(i,:) = ...
           [model.Center(1) model.Center(2) model.Center(3) ...
            model.Dimensions(1) model.Dimensions(2) model.Dimensions(3) ...
            model.Orientation(1) model.Orientation(2) model.Orientation(3)];
    end
end