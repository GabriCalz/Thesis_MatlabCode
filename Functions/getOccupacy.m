function listOfCircles = getOccupacy(locations, numb) 
    listOfCircles = zeros(numb, 3);
    for indexOfCluster=1:numb
        xCenter = locations(indexOfCluster,1);
        yCenter = locations(indexOfCluster,2);
        radius = 0.01;
        listOfCircles(indexOfCluster,:) = [xCenter yCenter radius];
    end
end