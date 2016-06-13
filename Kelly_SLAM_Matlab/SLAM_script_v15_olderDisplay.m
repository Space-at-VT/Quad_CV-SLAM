%% Monocular Simultaneous Localization and Mapping Code
% Scott Kelly's Masters Thesis - March 2015
% Title: A MONOCULAR SLAM METHOD TO ESTIMATE RELATIVE POSE DURING SATELLITE PROXIMITY OPERATIONS

close all;
clear all;
clc; 
addpath(genpath('./')); % Folder with this code and other necessay scripts
addpath(genpath('C:\Users\Daniel\Desktop\Scott\Data')); % Folder with the videos in it

%% Open video to read




% CubeSat on air bearing videos
% vid = VideoReader('high_bright_backAndForth_2015-01-23-114623-0000.avi'); cropVid = 0; cal='ptgrey'; manualBoundingBox = [340 268 800-340 502-268]; subRegionCount = 5; countThreshold = 19; startFrame = 75; endFrame = 740;
% vid = VideoReader('high_bright_backAndForth_2015-01-23-114514-0000.avi'); cropVid = 0; cal='ptgrey'; manualBoundingBox = [330 245 839-330 502-245]; subRegionCount = 5; countThreshold = 19; startFrame = 40; endFrame = 305;
% vid = VideoReader('high_bright_2015-01-23-112011-0000.avi'); cropVid = 0; cal='ptgrey'; manualBoundingBox = [345 268 770-345 502-268]; subRegionCount = 5; countThreshold = 19; startFrame = 40;  %initPixDist = .5;
% vid = VideoReader('high_bright_2015-01-23-111938-0000.avi'); cropVid = 0; cal='ptgrey'; manualBoundingBox = [345 268 770-345 502-268]; startFrame = 100; subRegionCount = 5; countThreshold = 19; initPixDist = 1;


% CLAM Synthetic and Real Data
% vid = VideoReader('data050.avi'); cropVid = 0; cal='est'; initPixDist = 2; subRegionCount = 5; countThreshold = 9; %Synthetic scenes numbered 050 - 058
% vid = VideoReader('data067.avi'); cropVid = 0; cal='est'; subRegionCount = 5; countThreshold = 9; %Synthetic scenes numbered 059 - 068
% vid = VideoReader('volleyball.mov'); cropVid = 0; cal='clam'; countThreshold = 9; subRegionCount = 5; %Other filenames: saltines.mov statue.mov volleyvall.mov hat.mov book.mov

% Orbital Express Data
% vid = VideoReader('orbexp2_selfInspection.mp4'); cropVid = 0; cal='est'; startFrame = 814; endFrame = 934; subRegionCount = 3; countThreshold = 19; distMult = 2; hammDistThresh = .9;  % .6, 100, 1000, 2.5, 19, 5, commented, 4
% vid = VideoReader('orbexp2_selfInspection.mp4'); cropVid = 0; cal='est'; startFrame = 1; endFrame = 200; subRegionCount = 3; countThreshold = 19; distMult = 2; hammDistThresh = .9;  % .6, 100, 1000, 2.5, 19, 5, commented, 4
% vid = VideoReader('orbital_express_stk.avi'); cropVid = 0; cal='est'; manualBoundingBox = [1 1 623 391]; subRegionCount = 3; countThreshold = 9;
% vid = VideoReader('orbital_express_clip_1.avi'); cropVid = 0; cal='est'; countThreshold = 50; subRegionCount = 9; manualBoundingBox = [1 1 320 317];



% vid = VideoReader('mug.mov'); cropVid = 0; cal='jamesSLR';

% vid = VideoReader('vid4_noCheckerboard.avi'); cropVid = 0; cal='iphone'; endFrame = 300;
% vid = VideoReader('stk.avi'); cropVid = 0; cal='est';

% vid = VideoReader('day2_test3_2015-01-16-160005-0000.avi'); cal='ptgrey';  manualBoundingBox = [340 275 940-340 565-275]; cropVid = 1; crop = [200 630 300 1040]; initPixDist = .9;
% vid = VideoReader('cubesat_test2.avi'); cropVid = 1; crop = [250 820 270 900]; cal='est'; startFrame = 50; maxBdErr = .1; manualBoundingBox = [340 340 820-340 610-350]; subRegionCount = 5; countThreshold = 19; hammDistThresh = .7;
% vid = VideoReader('cubesat_test1_clip.avi'); cropVid = 1; crop = [250 820 270 900]; cal = 'est';  startFrame = 1; maxBdErr = .1; manualBoundingBox = [340 350 800-340 720-350]; subRegionCount = 5; countThreshold = 19; hammDistThresh = 19;
vid = VideoReader('homer_clip.avi'); cropVid = 1; cal = 'est'; crop = [100 640 460 1050]; % hammDistThresh = .6; maxBdErr = .05; startFrame = 10;


% vid = VideoReader('shuttle_convert.mp4'); cropVid = 0; cal = 'est'; % Start frames 1, 1850, 4200. thresh ~ 50, init thresh ~ 60-200
% vid = VideoReader('prisma_entire_clip.avi'); cropVid = 0; cal = 'est'; hammDistThresh = .75; startFrame = 50; maxBdErr = .05; endFrame = 450; 
% vid = VideoReader('shuttle_from_iss_clip1.avi'); cropVid = 0; cal = 'est'; manualBoundingBox = [22 74 462-22 254-74];

numFrames = get(vid, 'NumberOfFrames');
frames = read(vid);

% reprojectionErrorThresh = 4;
initPixDist = .75;
startFrame=2;
% endFrame = 350;
% endFrame = 408;
% countThreshold = 19;
subRegionCount = 5;
% startFrame = 150;
hammDistThresh = .6;
% clear endFrame
% clear maxBdErr
% startFrame = 80;
% maxBdErr = .05;
% manualBoundingBox = [550 175 993-550 391-175];
% startFrame = 1;
% initPixDist = .5;
% maxBdErr = .05;
% starFrame = 75;
% initPixDist = 4;
% subRegionCount = 5;
% countThreshold = 19;



bestSubRegionKeep = 100;
totalTrackPointLimit = 1000;
if ~exist('hammDistThresh'), hammDistThresh = .5; end
if ~exist('initPixDist'), initPixDist = 1; end
if ~exist('countThreshold'), countThreshold = 9; end
if ~exist('subRegionCount'), subRegionCount = 3; end
if ~exist('distMult'), distMult = 1; end
if ~exist('maxBdErr'), maxBdErr = .01; end
if ~exist('reprojectionErrorThresh'), reprojectionErrorThresh = 4; end

% Only recalculate the motion bbox every motionRate number of iterations
motionRate = 10; motionCtr = 0;


%% Open video writer object
clear featureUpdateRate;
if ~exist('startFrame'), startFrame = 1; end
if ~exist('endFrame'), endFrame =  numFrames; end
plotAll = 0;
stepSize = 1;
plotSubRectangles = 0;
plotMotionHulls = 1;
saveVideo = 0; saveSLAM = 1; doSLAM = 1;
if saveVideo == 1;
    obj=VideoWriter('cubesat_2.avi');
    obj.FrameRate=15;
    open(obj);
end

%% Identify region of interest/bounding box from motionHull function

motionFrame1 = rgb2gray(frames(:,:,:,startFrame));
motionFrame2 = rgb2gray(frames(:,:,:,startFrame+20));
[ convexHull, boundingBox, roiConvexHull  ] = motionHull( motionFrame1, motionFrame2, 25, plotAll  );
if exist('manualBoundingBox')
    boundingBox = manualBoundingBox;
end


%% Assign/estimate camera calibration parameters

height = size(motionFrame1,1); width = size(motionFrame1,2); diagLength = sqrt(height^2 + width^2);
if strcmp(cal,'est')
    x01 =  width/2 ; y01 = height/2 ; 
    K1 = [width 0 x01; 0 width y01; 0 0 1];
elseif strcmp(cal,'clam')
    K1 = [1211.27296 0 632.18432 ; 0 1215.2592 346.752; 0 0 1]; 
elseif strcmp(cal,'ptgrey')
    K1 = [1701.504056 0 663.1777275 ; 0 1691.4521576 316.203396; 0 0 1];
elseif strcmp(cal,'iphone')
    K1 = [1003.0156417 0 473.084279; 0 1019.788973 283.6189157; 0 0 1];
elseif strcmp(cal,'jamesSLR')
    K1 = [2589.63074654 0 891.0970385; 0 2577.0647924 530.28276494; 0 0 1];
end

invK = inv(K1);
% If camera parameters are same
K2 = K1;

%% Identify initial set of points to track - corner detection with the FAST algorithm
im1points = detectFASTFeatures(motionFrame1,'ROI',boundingBox);
im1points = selectStrongest(im1points, totalTrackPointLimit); % Limit the number of points detected to 1000

%% Add in a binary feature extractor and matching routine to increase robustness of point tracks
[feats, featureLocation] = extractBRIEFFeatures(motionFrame1,im1points.Location); 

%% Create a structure to hold all of the points accumulated over all the views

% Add a global point number to tag along with each track point (it makes them easy to find in different views/the point cloud)
totalNumPoints = size(featureLocation,1);
visiblePoints = [[1:totalNumPoints]' featureLocation];
pointMatches{1} = visiblePoints';
frameCtr = 2; % The next point tracks will be saved as frame 2
% Create a pointLife vector to view the number of frames each point exists
pointLife = ones(totalNumPoints,1);
pointLifeSinceUpdate = ones(totalNumPoints,1);

%% Create a point tracker

pointTracker = vision.PointTracker('MaxBidirectionalError', maxBdErr, 'NumPyramidLevels',3); 
% pointTracker = vision.PointTracker('MaxBidirectionalError', .015,'NumPyramidLevels',1,'BlockSize',[13 13]); %%%%%%%%%%%% POSSIBLE ADJUSTMENTS

currFrame = motionFrame1;
prevFrame = currFrame;
initialize(pointTracker, visiblePoints(:,2:3), currFrame);

%% Set up the initial display figure

figure('units','normalized','outerposition',[0 .04 1 .96]); % Maximizes the figure window
plotFrame = insertMarker(currFrame, im1points, '+', 'Color', 'cyan');
if cropVid == 1
    plotFrame = plotFrame(crop(1):crop(2),crop(3):crop(4),:); % Crop it down to make it easier to see
end
tightsubplot(2,1,1);
h1 = imshow(plotFrame);
h2 = tightsubplot(2,1,2);

%% Extra initializations to be used
fpsLog = []; totalFpsLog = []; fps = []; mode = 'preinit';
pnpCtr = 0; badSbaCtr = 0; currPnpRate = 5;

for i = startFrame+stepSize:stepSize:endFrame
    tic
    %% KLT Point tracker (if not using matlab CV toolbox, calcOptFlowPyrLK in OpenCV - or even better, try and find implementation of KLT tracker with affine consistency check)
    currFrame = rgb2gray( frames(:,:,:,i) ); % Get the new frame    
    [kltUpdatePoints, isFound] = step(pointTracker, currFrame); % Track the points (matlab's KLT tracker in the CV toolbox)
    visiblePoints = [visiblePoints(isFound,1) kltUpdatePoints(isFound, :)]; % Track points updated with the new frame from the KLT tracker (update the first column as well with the global point numbers)
        
    %% Binary feature point extraction
    % Extract BRIEF features from the current frame
    [currFrameFeatures, ptsWithFeature] = extractBRIEFFeatures(currFrame,visiblePoints(:,2:3)); 
    if size(ptsWithFeature,1) ~= size(visiblePoints,1) % Sometimes points aren't good for extracting features, need to find the inlier index so they can be removed from the global index. Can tell if some didn't get features calculated if the sizes aren't the same
        featInliers = findReducedInliers(visiblePoints(:,2:3), ptsWithFeature); % Finding the inliers that had features calculated
        visiblePoints = visiblePoints(featInliers,:); % Saving them (or removing the points that didn't have a feature calculated for them
    end
    % Compare to the original occurrence of that point
    originalFeatures = feats(visiblePoints(:,1),:); % Use the first column of visiblePoints as the global point numbers of the original feature
      
    % Calculate the hamming distance between the points and save inliers as points with a distance less than the threshold (hammDistThresh)
    allDists = pdist2(currFrameFeatures,originalFeatures,'Hamming');
    sameDist = diag(allDists);
    inliers = [sameDist < hammDistThresh]; 
    
    visiblePoints = visiblePoints(inliers,:); % Only keep the matched points in visiblePoints (global point numbers are included as well)
    pointLife(visiblePoints(:,1)) = pointLife(visiblePoints(:,1)) + 1; % Increment the number of frames each visible point has existed
    pointLifeSinceUpdate(visiblePoints(:,1)) = pointLifeSinceUpdate(visiblePoints(:,1)) + 1; % Same for the pointLifeSinceUpdate vector
    
    %% Update features for points with a life since update greater than featureUpdateRate # of frames
    
    currPtLife = pointLife(visiblePoints(:,1));
    if exist('featureUpdateRate')
        updatePointsIndex = pointLifeSinceUpdate(visiblePoints(:,1)) > featureUpdateRate;
        if sum(updatePointsIndex) > 0 % If there are any points to be updated
            updateFeatures = extractBRIEFFeatures(currFrame,visiblePoints(updatePointsIndex,2:3)); % Extract new features for those points that have been around for a number of frames divisible by featureUpdateRate
            feats(visiblePoints(updatePointsIndex,1),:) = updateFeatures; % Replace the global feature reference set with the updated features
        end
    end

    %% Track motion from one frame to the next using the motionHull function
    % This is fairly inefficient - only doing it every so often (every motionRate # of frames)

    motionCtr = motionCtr + 1;
    if motionCtr == motionRate
        [ convexHull, boundingBox, roiConvexHull ] = motionHull( prevFrame, currFrame, 10, 0  );
        if exist('manualBoundingBox')
            boundingBox = manualBoundingBox;
        end
        prevFrame = currFrame;
        motionCtr = 0;
    end

    %% Automated initialization and  Updating the plotFrame display image
    plotFrame = currFrame; % Initial frame to display
    % While in preinit mode, draw a line from the initial point location to current location
    if strcmp(mode,'preinit')
        existSinceInit = find(pointLife == frameCtr); % Find the points that are visible that have existed since the initial frame
        firstLoc = pointMatches{1}(:,ismember(pointMatches{1}(1,:),existSinceInit))'; % The initial point locations
        currLoc = visiblePoints(ismember(visiblePoints(:,1),existSinceInit),:); % The points current (visibile) location
        plotFrame = insertShape(plotFrame,'Line',[firstLoc(:,2:3), currLoc(:,2:3)],'LineWidth',3); % Draw a line from the point's initial location to its current location
%         plotFrame = insertMarker(plotFrame, firstLoc(:,2:3), 'o', 'Color', 'red'); 
        plotFrame = insertMarker(plotFrame, currLoc(:,2:3), '+', 'Color', 'white'); 

        sameDist = diag(pdist2(firstLoc(:,2:3),currLoc(:,2:3))); 
        avgDist = mean(sameDist); % The average motion of all the points
        percentDist = avgDist/diagLength*100; % The normalized average distance (as a percentage of the image width)
        dispStr = strcat('Avg pix dist: ', num2str(avgDist), '. Pix dist %: ', num2str(percentDist), '. Num points that exist since first frame: ', num2str(size(existSinceInit,1)), ', ', num2str(size(existSinceInit,1)/size(im1points,1)));
        disp(dispStr);
       
        %% Perform automated initialization here
        if percentDist > initPixDist  % If the average pixel has moved at least 1 percent of the width of the frame or if only 35% of the initial points are left
            disp('Entering intialization mode. Reason: track point motion is greater than the threshold defined');
            mode = 'init'; % Change the mode to initialization
        elseif size(existSinceInit,1)/size(im1points,1) < .2 || size(existSinceInit,1) < 20  % The second reason, points are disappearing. Separated if statements to display what the init reason is
            disp('Entering initialization mode. Reason: disappearing points.');
            mode = 'init';
        end
        
    else
        % Plot 
        has3dIndex = find2D3Dcorrespondence(pointCloud, visiblePoints);
        
        plotFrame = insertShape(plotFrame,'FilledCircle', [visiblePoints(has3dIndex,2:3), 3*ones(size(visiblePoints(has3dIndex,2:3),1),1)], 'Color', 'yellow','Opacity',1);
        plotFrame = insertShape(plotFrame,'Circle', [visiblePoints(has3dIndex,2:3), 3*ones(size(visiblePoints(has3dIndex,2:3),1),1)], 'Color', 'blue','Opacity',1);
        
%         plotFrame = insertMarker(plotFrame, visiblePoints(has3dIndex,2:3), 'o', 'Color', 'cyan','size',5);
        % Only plot a cross on pts that are in ptLifeInliers
%         plotFrame = insertMarker(plotFrame, visiblePoints([currPtLife >= currPnpRate],2:3), '+', 'Color', 'yellow','size',5);         
    end

    % Insert the shape for the rectangular ROI if desired (to do so, set plotMotionHulls variable at top of file to 1)
    convexHullPolygon = reshape(convexHull',1,[]);
    roiConvexHullPolygon = reshape(roiConvexHull',1,[]);
    if plotMotionHulls == 1
        plotFrame = insertShape(plotFrame,'Rectangle',boundingBox,'Color','blue','LineWidth',3); % Draw bounding Box
        plotFrame = insertShape(plotFrame,'Polygon',convexHullPolygon,'Color','red'); % Draw Convex Hull
        plotFrame = insertShape(plotFrame,'Polygon',roiConvexHullPolygon,'Color','cyan'); % Draw Dilated Convex Hull
    end
    
    if strcmp(mode,'pnp_poseOnly')
        % Find visible 2D points that already have a 3d location
        [has3dIndex, pcIndex] = find2D3Dcorrespondence(pointCloud, visiblePoints);
        [R, T] = cv.solvePnPRansac(pointCloud(2:4,pcIndex)',visiblePoints(has3dIndex,2:3),K1,[],'ReprojectionError',8,'IterationsCount',25);
        currentPose = K1*[cv.Rodrigues(R) T];
        % If the motion is sufficient, do pnp for new position, but don't triangulate new points
        % Considerations:
        % 1. Translation between current location and last camera pose used for triangulation
        % 2. Number of visible points that match with a location in the point cloud
        cents(1,:) = -inv(currentPose(1:3,1:3))*currentPose(:,4);
        cents(2,:) = -inv(P{end}(1:3,1:3))*P{end}(:,4);
        lastT = pdist2(cents(1,:),cents(2,:));

        if lastT/initialT > .5 && pnpCtr > 5 || sum(pcIndex) < 12 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            mode = 'pnp_triangulate';
            currPnpRate = P_index(end) - P_index(end-1);
%             if exist('featureUpdateRate')
%                 dispStr = strcat('Feature update rate: ', num2str( featureUpdateRate  ), ', Current pnp rate: ', num2str(currPnpRate), ', ratio: ', num2str(lastT/initialT)); 
%             else
%                 dispStr = strcat('Current pnp rate: ', num2str(currPnpRate), ', ratio: ', num2str(lastT/initialT)); 
%             end
%             disp(dispStr);
        end
    end

    %% SLAM PORTION OF CODE    
    %% EITHER DO INITIALIZATION OR PnP PROBLEM
    %% INITIALIZATION
    if doSLAM == 1
    if strcmp(mode,'init')
        % Perform initialization with the sets of point correspondences found above

        [P, pointCloud] = initialization(firstLoc, currLoc, K1, K2);
        % Run SBA on those two frames - still possible, but changed to running SBA after initialization and the next frame is added (first 3 frames)
%         [pts3Dsba, P2f, info] = sba_two_frames(firstLoc(:,2:3), currLoc(:,2:3), pointCloud(2:4,:)',P{1},P{2},K1,0,1,25);
%         if info(7) == 2
%             pointCloud(2:4,:) = pts3Dsba';
%             P{2} = P2f;
%         end

        P_index = [1 frameCtr]'; % Store what frame each projection matrix comes from

        % Point Cloud Cleanup method
        [pointCloud, distThreshold, center] = initPcCleanup(pointCloud);
        distThreshold = distMult*distThreshold;

        % Test somehow if initialization was good - NOT YET IMPLEMENTED
        % If initialization was good, change mode to 'pnp' to add to the point cloud with successive frames
        mode = 'pnp_poseOnly';
        pnpCtr = 0;
        % Plot the point cloud
        plot3(h2,pointCloud(2,:),pointCloud(3,:),pointCloud(4,:),'.'); axis equal; hold all; view(-2, -56);
        % Plot the first camera and first axes
        plotCamera(P{1},invK,'g');
        % Plot the second camera and second axes
        plotCamera(P{2},invK,'b');
        % Plot a line between them
        cents(1,:) = -inv(P{1}(1:3,1:3))*P{1}(:,4);
        cents(2,:) = -inv(P{2}(1:3,1:3))*P{2}(:,4);
        plot3(cents(:,1),cents(:,2),cents(:,3),'Color',[.1 .1 .1])
        % Save the initial translation amount - will be used to determine future PnP rates
        initialT = pdist2(cents(1,:),cents(2,:));
        
    %% PnP PROBLEM
    elseif strcmp(mode,'pnp_triangulate') % PnP with triangulation

        % Find visible 2D points that already have a 3d location
        [has3dIndex, pcIndex] = find2D3Dcorrespondence(pointCloud, visiblePoints);
        if sum(has3dIndex) <= 3
            error('Error! There might not be enough points to solve PnP! Check here.');
        end
        
        % Solve PnP Problem to add next camera projection matrix P now that 2d - 3d matches are found
        [R, T, pnpIdxInliers] = cv.solvePnPRansac(pointCloud(2:4,pcIndex)',visiblePoints(has3dIndex,2:3),K1,[],'ReprojectionError',reprojectionErrorThresh,'IterationsCount',500);
        if exist('pnpIdxInliers')
            pnpIdxInliers = pnpIdxInliers + 1; % Since the OpenCV function begins indexing at zero
            % If it is an outlier, end that tracked point
            if size(pointCloud(2:4,pcIndex)',1) ~= size(pnpIdxInliers,1)
                [ pointCloud, visiblePoints ] = removePnpOutliers( pointCloud, pcIndex, pnpIdxInliers, visiblePoints, frameCtr );
                % Need to recalculate has3dIndex since removed some visible points during RANSAC above
                [has3dIndex, pcIndex] = find2D3Dcorrespondence(pointCloud, visiblePoints);
            end
        end

        % Add results of PnP RANSAC to P{}
        P{end+1} = K1*[cv.Rodrigues(R) T];
        P_index(end+1) = frameCtr; % Update what frame the projection matrix corresponds to in P_index
        % Plot new camera position
        plotCamera(P{end},invK,'b');
        cents(1,:) = -inv(P{end-1}(1:3,1:3))*P{end-1}(:,4);
        cents(2,:) = -inv(P{end}(1:3,1:3))*P{end}(:,4);
        plot3(cents(:,1),cents(:,2),cents(:,3),'Color',[.1 .1 .1])
        % Update the feature update rate (every other pose estimate, update the BRIEF features)
        featureUpdateRate = (P_index(end) - P_index(end-2));
        currPnpRate = P_index(end) - P_index(end-1);


        % Add new 2d point matches to the 3d point cloud via triangulation        
        no3dIndex = not(has3dIndex); % Find points that are visible but aren't projected to 3D yet
        no3dIndex = no3dIndex & pointLife(visiblePoints(:,1)) > round(currPnpRate); % Reduce that list to points that were visible at least in the previous frame
        
        if sum(no3dIndex) > 0 % Add points to the point cloud that are in no3dIndex
            % Get global index numbers of points being added
            globalIndex = visiblePoints(no3dIndex,1);
            % Get current location of points to add
            f2pts = visiblePoints(no3dIndex,:);
            % Get projection matrix for current occurrence
            P2 = P{end};
            % Get frame number of projection matrix for last occurrence
            f1index = ismember(pointMatches{P_index(end-1)}(1,:)',globalIndex);
            f1pts = pointMatches{P_index(end-1)}(:,f1index)'; % To view the matches: showMatchedFeatures(frames(:,:,:,startFrame + origFrame - 1),frames(:,:,:,startFrame+frameCtr-1),f1pts,f2pts,'Montage')
            % Get projection matrix for initial point occurrence
            P1 = P{end-1};
            % Add points to the point cloud through triangulation            
            if size(f1pts,1) > 0 % If there are any points to be projected to 3D
                Xw = Triangulation(makehomogeneous(f1pts(:,2:3)'),P1,makehomogeneous(f2pts(:,2:3)'),P2);
                [Xw, globalIndex] = newPcPointsCleanup(Xw, distThreshold, center, globalIndex); % Clean up the new 3D points (Xw) based on the center and distThreshold stats
                % If Xw has points left to be added to the point cloud, add them and plot them
                if size(Xw,2) > 0
                    plot3(h2,Xw(1,:),Xw(2,:),Xw(3,:),'.');
                    pointCloud = [pointCloud, [globalIndex'; Xw(1:3,:)]];
                end
            end
        end

        %% Perform Incremental SBA - currently only doing it on the first three pose estimates (motion and structure)
        if length(P) == 3
            pointMatches{frameCtr} = visiblePoints'; % Assign this early to make it easier to reference with the history of points                    
            % Incremental SBA - mod a few frames (numFramesToMod) based on the measurements from a larger set (numRefFrames)
            numRefFrames = 3;
            numFramesToMod = 3;
            numIter = 100;
            sbaMode = 'motstr'; % Motion and structure (motstr) SBA or just motion (mot) or just structure (str)

            [ pointCloud, P, badSbaCtr, replot ] = incrementalSba( pointMatches, P, P_index, pointCloud, pointLife, badSbaCtr, numRefFrames, numFramesToMod, numIter, sbaMode, K1, invK );
%             if replot
                disp('Replotting point cloud and cameras.');
                hold off;
                % Plot the point cloud
                plot3(h2,pointCloud(2,:),pointCloud(3,:),pointCloud(4,:),'.'); axis equal; hold all; view(-2, -56);
                % Plot the first camera and first axes
                plotCamera(P{1},invK,'g');
                for j = 2:length(P)
                    % Plot the rest of the cameras and axes
                    plotCamera(P{j},invK,'b');
                    % Plot a line between them
                    cents(1,:) = -inv(P{j-1}(1:3,1:3))*P{j-1}(:,4);
                    cents(2,:) = -inv(P{j}(1:3,1:3))*P{j}(:,4);
                    plot3(cents(:,1),cents(:,2),cents(:,3),'Color',[.1 .1 .1])
                end
%             end
            cents(1,:) = -inv(P{end}(1:3,1:3))*P{end}(:,4);
            cents(2,:) = -inv(P{end-2}(1:3,1:3))*P{end-2}(:,4);
            % Save the initial translation amount - will be used to determine future PnP rates
            initialT = pdist2(cents(1,:),cents(2,:));
        end
  
        pnpCtr = 0;
        mode = 'pnp_poseOnly';

        % Set some sort of check to see if the results are terrible.
    else
        % Not sure what other modes there might be yet
    end
    end % End doSLAM if statement
    
    %%
    %% Adding more points - divide up the bounding box into sub-rectangles
    [rectanglesX, rectanglesY] = divideRectangle(boundingBox,subRegionCount);
    % Plot the rectangular sub-regions
    if plotSubRectangles == 1
        for j = 1:size(rectanglesX,2)
            plotFrame = insertShape(plotFrame,'Rectangle',[rectanglesX(1,j),rectanglesY(1,j),rectanglesX(3,j)-rectanglesX(1,j)+1,rectanglesY(2,j)-rectanglesY(1,j)+1]);
        end
    end
    
    
    %% Now go through each rectangle and search how many points are present

%     countThreshold = 5;
    trackPoints = visiblePoints(:,2:3);
    for j = 1:size(rectanglesX,2)
        xv = rectanglesX(:,j);
        yv = rectanglesY(:,j);
        x = trackPoints(:,1);
        y = trackPoints(:,2);
        in = inpolygon(x,y,xv,yv);
        count = sum(in);
        if plotSubRectangles == 1
            plotFrame = insertText(plotFrame, [xv(1)+7, yv(1)+8], [num2str(count)],'TextColor','black' );
        end;
        
        % If there are less than a certain number of points in a specific
        % region, then detect more points there
        if count < countThreshold
            regionBbox = [xv(1), yv(1), xv(3)-xv(1)+1, yv(2)-yv(1)+1];
            newCornerPoints = detectFASTFeatures(currFrame,'ROI',regionBbox);
%             newCornerPoints = selectStrongest(newCornerPoints,bestSubRegionKeep); % Only take the best X points from the sub-region that were detected
            
            if size(newCornerPoints,1) > 0 % If there are new corner points detected in the region where the number of points are below the threshold
                [newFeats, newPoints] = extractBRIEFFeatures(currFrame,newCornerPoints.Location);
                newPointGlobalNums = [totalNumPoints+1:totalNumPoints+size(newPoints,1)]';
                totalNumPoints = totalNumPoints + size(newPoints,1);
                feats = [feats; [ newFeats ] ];
                visiblePoints = [visiblePoints; [ newPointGlobalNums newPoints ] ];
                pointLife(newPointGlobalNums) = 1;
                pointLifeSinceUpdate(newPointGlobalNums) = 1;
            end
        end
        
        
    end
    
%     break;
    
    %% Storing the matched point information
    pointMatches{frameCtr} = visiblePoints';
    
    %% Reset the points
    setPoints(pointTracker, visiblePoints(:,2:3));
    
    %% Display the point cloud and cameras


    %% Update the average framerate
    fpsLog(end+1) = 1/toc;
    totalFpsLog(end+1) = fpsLog(end);
    if size(fpsLog,2) == 5
        fps = mean(fpsLog);
        fpsLog = [];
    end
    
    %% Final display of plot with FPS in the corner

    if cropVid == 1
        plotFrame = plotFrame(crop(1):crop(2),crop(3):crop(4),:); % Crop it down to make it easier to see
    end
    if fps
        plotFrame = insertText(plotFrame, [10 10], ['FPS: ' num2str(fps)],'TextColor','black','FontSize',18,'BoxColor','yellow');
    end
    set(h1,'Cdata',plotFrame);
    drawnow;  

    
    %% Saving to a video
    if saveVideo == 1
        if saveSLAM == 1
            tmp = getframe(gca);
            writeVideo(obj,tmp.cdata); 
        else
            writeVideo(obj,plotFrame);
        end
    end
    frameCtr = frameCtr + 1;
    pnpCtr = pnpCtr + 1;

%     waitforbuttonpress;
end


% Close the video writer object
if saveVideo == 1,close(obj); end;























break;
%%%%%%%%%%%%%%%%%%%%%%%%%%
% Observe point match windows
blockSize = 33;

pointNum = 1;
inliers = ismember(pointMatches{1}(1,:),pointNum);
ptLoc = pointMatches{1}(2:3,inliers);




plotFrame = 2;
frameNum = startFrame + P_index(plotFrame) - 1;
imshow(frames(:,:,:,frameNum));
hold on;
plot(pointMatches{P_index(plotFrame)}(2,:),pointMatches{P_index(plotFrame)}(3,:),'b.','MarkerSize',20)

featCtr = 1;

for i = 1:length(P_index)
    inliers = ismember(pointMatches{P_index(i)}(1,:),pointNum);
    
    ptLoc = pointMatches{P_index(i)}(2:3,inliers)';
    frameNum = startFrame + P_index(i) - 1;
    feat = extractFeatures(rgb2gray(frames(:,:,:,frameNum)),ptLoc,'Method','Block','BlockSize',blockSize);
    feat = reshape(feat,blockSize,blockSize);
    imshow(feat);
%     waitforbuttonpress;
    if mod(i-1,2) == 0
        imwrite(feat,strcat('featDrift',num2str(featCtr),'.png'),'PNG');
        featCtr = featCtr + 1;
    end
    
end










