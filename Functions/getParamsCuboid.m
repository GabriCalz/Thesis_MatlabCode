function listOfCuboids = getParamsCuboid(boundariesOfClusters, numb) 
            listOfCuboids = zeros(numb, 9);
            for indexOfCluster=1:numb
                xCenter = (boundariesOfClusters(indexOfCluster, 1) + boundariesOfClusters(indexOfCluster, 2))/2;
                yCenter = (boundariesOfClusters(indexOfCluster, 3) + boundariesOfClusters(indexOfCluster, 4))/2;
                zCenter = (boundariesOfClusters(indexOfCluster, 5) + boundariesOfClusters(indexOfCluster, 6))/2;
                xLen = abs(boundariesOfClusters(indexOfCluster, 1) - boundariesOfClusters(indexOfCluster, 2));
                yLen = abs(boundariesOfClusters(indexOfCluster, 3) - boundariesOfClusters(indexOfCluster, 4));
                zLen = abs(boundariesOfClusters(indexOfCluster, 5) - boundariesOfClusters(indexOfCluster, 6));
                listOfCuboids(indexOfCluster,:) = [xCenter yCenter zCenter xLen yLen zLen 0 0 0];
            end
        end