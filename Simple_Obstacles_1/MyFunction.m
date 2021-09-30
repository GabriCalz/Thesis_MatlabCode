function [groundPtsIdx, nonGroundPtCloud, groundPtCloud] = segmentGroundSMRF(ptCloud, varargin)
%segmentGroundSMRF Segment ground from lidar data using SMRF algorithm
%   Simple morphological filter (SMRF) segments point cloud data into
%   ground and non-ground points. SMRF initially converts the point cloud
%   into a regularly spaced minimum surface grid, and applies a series of
%   morphological opening operations to estimate the ground surface. These
%   estimates are compared with the elevation values of original data to
%   segment the points into ground and non-ground. The algorithm assumes
%   the elevation of points to be along Z-axis.
%
%   groundPtsIdx = segmentGroundSMRF(ptCloud) segments the input point
%   cloud, ptCloud, into ground and non-ground points and returns a logical
%   matrix, groundPtsIdx. A true value in the matrix indicates that the
%   corresponding point in the original point cloud is a ground point and
%   vice-versa.
%
%   groundPtsIdx = segmentGroundSMRF(ptCloud, gridResolution) segments the
%   point cloud using the specified grid constraints, gridResolution.
%   gridResolution is a positive scalar that specifies the resolution of
%   grid element. By default, grid resolution is assumed to be 1 meter per
%   element.
%
%   [..., nonGroundPtCloud, groundPtCloud] = segmentGroundSMRF(...)
%   additionally returns segmented non-ground and ground point clouds.
%
%   [...] = segmentGroundSMRF(..., Name, Value) additionally specifies the
%   name-value pair arguments described below:
%
%   'MaxWindowRadius'       A positive integer that specifies the maximum
%                           radius of the disk-shaped structuring element
%                           in the morphological opening operation. The
%                           radius starts from 1 and increments by one in
%                           each iteration. Opening operation stops when
%                           the radius reaches the specified value.
%                           Increase this value to segment large buildings
%                           as non-ground at the expense of additional
%                           computation.
%
%                           Default: 18
%
%   'ElevationThreshold'    A non-negative scalar that specifies the
%                           elevation threshold to identify non-ground
%                           points. A point is classified as non-ground
%                           when the elevation difference between the point
%                           and the ground surface estimated from opening
%                           operation is greater than this value. Increase
%                           this value to encompass more points from bumpy
%                           ground.
%
%                           Default: 0.5
%
%   'SlopeThreshold'        A non-negative scalar that specifies the slope
%                           threshold to identify non-ground grid elements
%                           in the minimum surface. A grid element is
%                           classified as non-ground when the slope is
%                           greater than this value. Increase this value to
%                           estimate steep slopes as ground.
%
%                           Default: 0.15
%
%   'ElevationScale'        A non-negative scalar that scales the elevation
%                           threshold with respect to the slope of the
%                           estimated ground surface from the opening
%                           operation. Increase this value to identify
%                           ground points on steep slopes.
%
%                           Default: 1.25
%
%   Class Support
%   -------------
%   ptCloud must be a pointCloud object. groundPtsIdx is a logical matrix
%   for organized point cloud and vector for unorganized point cloud.
%   nonGroundPtCloud and groundPtCloud are pointCloud objects.
%
%   Example 1: Segment ground from unorganized point cloud
%   ------------------------------------------------------
%   % Construct lasFileReader object.
%   fileName = fullfile(toolboxdir('lidar'), 'lidardata', 'las', ...
%   'aerialLidarData2.las');
%   lasReader = lasFileReader(fileName);
%
%   % Read all points from the lasReader.
%   ptCloud = readPointCloud(lasReader);
%
%   % Segment the ground data
%   groundPtsIdx = segmentGroundSMRF(ptCloud);
%
%   % Extract ground and non-ground points
%   groundPtCloud = select(ptCloud, groundPtsIdx);
%   nonGroundPtCloud = select(ptCloud, ~groundPtsIdx);
%
%   % Visualize ground and non-ground points
%   figure
%   pcshowpair(groundPtCloud, nonGroundPtCloud)
%
%   Example 2: Segment ground from organized point cloud
%   ----------------------------------------------------
%   % Load an organized lidar point cloud
%   ld = load('drivingLidarPoints.mat');
%
%   % Segment and extract the ground and non-ground points
%   [~, nonGroundPtCloud, groundPtCloud] = segmentGroundSMRF(...,
%       ld.ptCloud, "ElevationThreshold", 0.1, "ElevationScale", 0.25);
%
%   % Visualize ground and non-ground points
%   figure
%   pcshowpair(groundPtCloud, nonGroundPtCloud)
%
%   See also pointCloud, lasFileReader, segmentGroundFromLidarData,
%   pcfitplane, pcsegdist, segmentLidarData, pcshow, pcshowpair.

%   Copyright 2020 The MathWorks, Inc.
%
%   References
%   ----------
%   Pingel, Thomas J., Keith C. Clarke, and William A. McBride. "An
%   improved simple morphological filter for the terrain classification of
%   airborne LIDAR data." ISPRS Journal of Photogrammetry and Remote
%   Sensing 77 (2013): 21-30.

narginchk(1, 10);
% Validate inputs
[gridRes, maxRadius, slopeThres, elevThres, elevScale] = validateAndParseInputs(ptCloud, varargin{:});

if ismatrix(ptCloud.Location)
    groundPtsIdx = false(size(ptCloud.Location, 1), 1);
else
    groundPtsIdx = false(size(ptCloud.Location, 1:2));
end

% Remove invalid points
[validPtCloud, validIndices]  = removeInvalidPoints(ptCloud);

if ~isempty(validIndices)

    % Create Initial Minimum Surface
    xGridLims = [ceil(validPtCloud.XLimits(1)/gridRes) * gridRes floor(validPtCloud.XLimits(2)/gridRes) * gridRes];
    yGridLims = [ceil(validPtCloud.YLimits(1)/gridRes) * gridRes floor(validPtCloud.YLimits(2)/gridRes) * gridRes];
    xGridLims = cast(xGridLims, 'like', validPtCloud.Location);
    yGridLims = cast(yGridLims, 'like', validPtCloud.Location);
    minSurface = lidar.internal.createSurfaceImpl(validPtCloud.Location, xGridLims, yGridLims, gridRes, gridRes);

    % Create mask to fill empty grids
    %emptyGrids = isnan(minSurface);
    %if (any(emptyGrids(:)))
    %    if all(size(minSurface) >= [3 3])
    %        minSurface = regionfill(minSurface, emptyGrids);
     %   else
     %       minSurface = fillmissing(minSurface', 'nearest');
     %       minSurface = fillmissing(minSurface', 'nearest');
        %end
   % end

    % Detect Outliers
    thres = 5 * gridRes;
    openedSurface = imopen(-minSurface, strel('disk', 1));
    outlierGrids = (minSurface + openedSurface) < -thres;

    % Calculate disk radius for each iteration
    minDim = min(size(minSurface));
    windowSizes = 1:min(maxRadius, minDim + 1);

    % Define elevation thresholds using grid size, window size and slope tolerance
    thres = slopeThres * (windowSizes * gridRes);

    % Iteratively apply opening operation to detect ground and non-ground points
    objectGrids = zeros(size(minSurface), 'logical');
    curSurface = minSurface;

    for i = 1:length(windowSizes)
        openedSurface = imopen(curSurface, strel('disk', windowSizes(i)));
        objectGrids = objectGrids | (curSurface - openedSurface > thres(i));
        curSurface = openedSurface;
    end

    % Create mask to fill non-ground grids
    nonGrndGrids = emptyGrids | outlierGrids | objectGrids;
    estGrndSurface = minSurface;
    % Fill non-ground grids in initial minimum surface to create digital elevation model of ground
    if (any(nonGrndGrids(:)))
        if all(size(minSurface) >= [3 3])
            %estGrndSurface = regionfill(estGrndSurface, nonGrndGrids);
        else
            estGrndSurface = fillmissing(estGrndSurface', 'nearest');
            estGrndSurface = fillmissing(estGrndSurface', 'nearest');
        end
    end

    % Calculate slope at each point
    if isvector(estGrndSurface)
        surfaceSlope = gradient(estGrndSurface / gridRes);

        if numel(estGrndSurface) == 1
            estElevation = repmat(estGrndSurface, size(validPtCloud.Location, 1), 1);
            estSlope = repmat(surfaceSlope, size(validPtCloud.Location, 1), 1);

        elseif size(estGrndSurface, 1) == 1
            c = (validPtCloud.Location(:, 1) - xGridLims(1) + gridRes)/gridRes;
            estElevation = interp1(estGrndSurface, c, 'spline');
            estSlope = interp1(surfaceSlope, c, 'spline');

        else
            r = (validPtCloud.Location(:, 2) - yGridLims(1) + gridRes)/gridRes;
            estElevation = interp1(estGrndSurface, r, 'spline');
            estSlope = interp1(surfaceSlope, r, 'spline');
        end
    else
        [gradX, gradY] = gradient(estGrndSurface / gridRes);
        surfaceSlope = sqrt(gradX.^2 + gradY.^2);
        % Interpolate digital elevation model to identify elevation at each point.
        r = (validPtCloud.Location(:, 2) - yGridLims(1) + gridRes)/gridRes;
        c = (validPtCloud.Location(:, 1) - xGridLims(1) + gridRes)/gridRes;

        estElevation = interp2(estGrndSurface, c, r, 'spline');
        estSlope = interp2(surfaceSlope, c, r, 'spline');
    end

    % Calculate threshold
    allowableThres = elevThres + (elevScale * estSlope);
    grndIds = abs(estElevation - validPtCloud.Location(:, 3)) <= allowableThres;
    groundPtsIdx(validIndices) = grndIds;
end

if nargout >= 2
    nonGroundPtCloud = select(ptCloud, ~groundPtsIdx, 'OutputSize', 'full');
end

if nargout == 3
    groundPtCloud = select(ptCloud, groundPtsIdx, 'OutputSize', 'full');
end

end

% -------------------------------------------------------------------------
% Validate and parse inputs
% -------------------------------------------------------------------------
function [gridRes, maxRadius, slopeThres, elevThres, elevScale] = validateAndParseInputs(ptCloud, varargin)

validateattributes(ptCloud, {'pointCloud'}, {'scalar'}, mfilename, 'ptCloud');
% Get default parameters
defaults = getDefaultParameters();

% Set input parser
%parser = inputParser;
%parser.CaseSensitive = false;
%parser.addOptional('gridResolution', defaults.gridResolution);
%%parser.addParameter('MaxWindowRadius', defaults.MaxWindowRadius);
%parser.addParameter('SlopeThreshold', defaults.SlopeThreshold);
%parser.addParameter('ElevationThreshold', defaults.ElevationThreshold);
%parser.addParameter('ElevationScale', defaults.ElevationScale);
%parser.parse(varargin{:});

% Extract inputs
gridRes = parser.Results.gridResolution;
maxRadius  = parser.Results.MaxWindowRadius;
slopeThres = parser.Results.SlopeThreshold;
elevThres  = parser.Results.ElevationThreshold;
elevScale = parser.Results.ElevationScale;

% Validate PV pairs
validateattributes(gridRes, {'numeric'}, ...
    {'nonnan', 'finite', 'nonsparse', 'scalar', 'positive'}, ...
    mfilename, 'gridResolution');
validateattributes(maxRadius, {'numeric'}, ...
    {'nonnan', 'finite', 'nonsparse', 'real', 'scalar', 'positive', 'integer'}, ...
    mfilename, 'MaxWindowRadius');
validateattributes(slopeThres, {'numeric'}, ...
    {'nonnan', 'finite', 'nonsparse', 'real', 'scalar', 'nonnegative'}, ...
    mfilename, 'SlopeThreshold');
validateattributes(elevThres, {'numeric'}, ...
    {'nonnan', 'finite', 'nonsparse', 'real', 'scalar', 'nonnegative'}, ...
    mfilename, 'ElevationThreshold');
validateattributes(elevScale, {'numeric'}, ...
    {'nonnan', 'finite', 'nonsparse', 'real', 'scalar', 'nonnegative'}, ...
    mfilename, 'ElevationScale');

% Cast input to point cloud locations type
slopeThres = cast(slopeThres, 'like', ptCloud.Location);
elevThres  = cast(elevThres, 'like', ptCloud.Location);
elevScale  = cast(elevScale, 'like', ptCloud.Location);
gridRes    = cast(gridRes, 'like', ptCloud.Location);
maxRadius  = double(maxRadius);
end

% -------------------------------------------------------------------------
% Default parameters
% -------------------------------------------------------------------------
function defaults = getDefaultParameters()
defaults = struct(...
    'gridResolution', 1, ...
    'MaxWindowRadius', 18, ...
    'SlopeThreshold', 0.15,...
    'ElevationThreshold', 0.5,...
    'ElevationScale', 1.25);
end
