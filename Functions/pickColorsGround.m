function pointscolor = pickColorsGround(PointCloud,GROUND_COLOR_RGB)
    pointscolor = uint8(zeros(PointCloud.Count,3));
    pointscolor(:,1) = GROUND_COLOR_RGB(1);
    pointscolor(:,2) = GROUND_COLOR_RGB(2);
    pointscolor(:,3) = GROUND_COLOR_RGB(3);
end

