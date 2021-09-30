
function searchArea = pickSearchArea(points, extSize, roverRadius)
    searchArea = points;
    for index=1:size(searchArea,1)
        vehicle = searchArea(index, 1)^2+searchArea(index, 2)^2 <= roverRadius^2;
        outOfBoundaries = ~(searchArea(index, 1)>=extSize(1) && ...
            searchArea(index, 1)<=extSize(2) && ...
            searchArea(index, 2)>=extSize(3) && ...
            searchArea(index, 2)<=extSize(4) && ...
            searchArea(index, 3)>=extSize(5) && ...
            searchArea(index, 3)<=extSize(6));
        if outOfBoundaries || vehicle
                searchArea(index,1)=0;
                searchArea(index,2)=0;
                searchArea(index,3)=0;
        end
    end
    searchArea = searchArea(any(searchArea,2),:);
end