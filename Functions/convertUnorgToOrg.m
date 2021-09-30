function ptCloudOrganized = convertUnorgToOrg(ptCloud,vResolution,hResolution,vbeamAngles,hbeamAngles)

    locations = ptCloud.Location;
    
    if ~isempty(ptCloud.Intensity)
        intensity = ptCloud.Intensity;
    else
        intensity = zeros(size(ptCloud.Location,1),1);
    end  
    
    % Calculate the radial distance for every point.
    r = sqrt(locations(:,1).^2 + locations(:,2).^2);
    r(r==0) = 1e-6;
    
    % Calculate the pitch and yaw angles for each point in the point cloud.
    pitch = atan2d(locations(:,3),r);
    yaw = atan2d(locations(:,2),locations(:,1));
    
    % Shift the range of the yaw angle from [-pi,pi] to [0,2*pi]. 
    yaw = 180-yaw;    
    
    % Calculate the row indices for all points based on the bin in which the pitch angle for each point falls into.
    [~,~,rowIdx] = histcounts(pitch,flip(vbeamAngles));
    rowIdx(rowIdx == 0) = NaN;
    rowIdx = vResolution - rowIdx;
    
    % Calculate the column indices for all points based on the bin in which the yaw angle for each point falls into.
    [~,~,colIdx] = histcounts(yaw,hbeamAngles);
    colIdx(colIdx == 0) = NaN;
    
    % Create a pseudo image and fill in the values with the corresponding location and intensity values.
    pseduoImage = NaN(vResolution,hResolution,4);
    for i = 1:size(locations,1)
        if ~isnan(rowIdx(i,1)) && ~isnan(colIdx(i,1))
            pseduoImage(rowIdx(i,1),colIdx(i,1),1) = locations(i,1);
            pseduoImage(rowIdx(i,1),colIdx(i,1),2) = locations(i,2);
            pseduoImage(rowIdx(i,1),colIdx(i,1),3) = locations(i,3);
            pseduoImage(rowIdx(i,1),colIdx(i,1),4) = intensity(i,1);
        end
    end
    
    % Create a point cloud object from the locations and intensity.
    if ~isempty(ptCloud.Intensity)
        ptCloudOrganized = pointCloud(pseduoImage(:,:,1:3), 'Intensity', pseduoImage(:,:,4));
    else
        ptCloudOrganized = pointCloud(pseduoImage(:,:,1:3));
    end
end


