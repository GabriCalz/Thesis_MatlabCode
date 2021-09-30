function pointscolor = pickColorsObstacles(PointCloud,OBSTACLE_COLOR_RGB)
    pointscolor = uint8(zeros(PointCloud.Count,3));
    pointscolor(:,1) = OBSTACLE_COLOR_RGB(1);
    pointscolor(:,2) = OBSTACLE_COLOR_RGB(2);
    pointscolor(:,3) = OBSTACLE_COLOR_RGB(3);
end

