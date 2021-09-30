function boundariesOfObstacles = getBoundariesOfObstacles(searchArea, labels, numberOfClusters) 
    searchArea(:, 4) = labels;
    boundariesOfObstacles = zeros(numberOfClusters, 6);
    for i=1:numberOfClusters 
        focus = searchArea(searchArea(:,4)==i , :);
        boundariesOfObstacles(i,1)=min(focus(:,1));
        boundariesOfObstacles(i,2)=max(focus(:,1));
        boundariesOfObstacles(i,3)=min(focus(:,2));
        boundariesOfObstacles(i,4)=max(focus(:,2));
        boundariesOfObstacles(i,5)=min(focus(:,3));
        boundariesOfObstacles(i,6)=max(focus(:,3));
    end
end