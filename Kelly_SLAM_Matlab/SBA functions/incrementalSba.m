function [ pointCloud, P, badSbaCtr, replot ] = incrementalSba( pointMatches, P, P_index, pointCloud, pointLife, badSbaCtr, numRefFrames, numFramesToMod, numIter, sbaMode, K1, invK )
%incrementalSba Formats the history of motion and structure data for input to SBA and runs SBA
%   Inputs are numRefFrames and numFramesToMod. Incremental SBA means that the reprojection error is minimized 
%   based on the data from the last number of reference frames (numRefFrames). The optimization parameters that
%   are changed to achieve the minimization are only based on the last numFramesToMod number of frames.
%   
replot = false;
% Get the last numRefFrames (5?) frames worth of motion and structure - set up the inputs for SBA
    if length(P) < numRefFrames
        ncams = length(P);
    else ncams = numRefFrames; end % Number of frames (m variable when calling SBA)

    if length(P) < numFramesToMod
        numFramesToMod = length(P);
    end

    % Find all 3D points that were visible in the last 5 frames
    % First, find all the 2D point matches from the last 5 frames
    lastFive2DpointNums = [];
    for j = ncams:-1:1
        lastFive2DpointNums = [lastFive2DpointNums pointMatches{P_index(end+1-j)}(1,:)];
    end
    lastFive2DpointNums = unique(lastFive2DpointNums);
    % Sort based on point life
%                         tmp = sortrows([-pointLife(lastFive2DpointNums) lastFive2DpointNums']);
%                         lastFive2DpointNums = tmp(:,2)';
    % Keep the points from lastFive2DpointNums that have a point life <= numRefFrames*pnpRate
    tmp = pointLife(lastFive2DpointNums) <= P_index(end)-P_index(end-ncams+1)+1;
    lastFive2DpointNums = lastFive2DpointNums(tmp);

    % Of all the 2D point matches from the last 5 frames, find the ones that have a 3D location
    pts3DlastFiveIndex = ismember(pointCloud(1,:)',lastFive2DpointNums);
    % Find their global indices
    globalLastFiveIndex = pointCloud(1,pts3DlastFiveIndex);

    % Find the points that we don't want to modify
    % points that exist in the last numFramesToMod * pnpRate frames
    tmp = false(size(globalLastFiveIndex,2),1);
    for j = 1:numFramesToMod
        tmp(ismember(globalLastFiveIndex, pointMatches{P_index(end+1-j)}(1,:))') = true;
    end
    % Index for points that exist <= numFramesToMod * pnpRate frames from globalLastFiveIndex

%                     tmp = tmp & pointLife(globalLastFiveIndex) <= numFramesToMod * pnpRate;
    tmp = tmp & pointLife(globalLastFiveIndex) <= P_index(end) - P_index(end-numFramesToMod+1) + 1;
    % Re-order the globalLastFiveIndex points so that the points we don't want to modify are last
    globalLastFiveIndex = [globalLastFiveIndex(not(tmp)) globalLastFiveIndex(tmp)];
    npts = size(globalLastFiveIndex,2); 
    numStrucPtsNoMod = size(globalLastFiveIndex(not(tmp)),2);

    % Find the 3D points from the point cloud, in the order of globalLastFiveIndex
    [idx, tmp] = ismember(globalLastFiveIndex,pointCloud(1,:));
    pts3DlastFive = pointCloud(2:4,tmp);

    % Initialize sba mask to all zeros
    dmask=zeros(size(globalLastFiveIndex,2), ncams); %nxm matrix specifying the points visible in each camera. A nonzero element at position (i, j) indicates that point i is visible in image j.

    % Find all the 2D point matches for those points in the last five frames
    % In the order (x_11, .. x_1m, ..., x_n1, .. x_nm), where x_ij is the projection of the i-th point on the j-th image.
    % m is the number of views (ncams)
    % n is the number of 3D pts
    pts2DlastFive = [];

    for j = 1:size(globalLastFiveIndex,2)
        for k = 1:ncams
            idx = find(pointMatches{P_index(end-ncams+k)}(1,:) == globalLastFiveIndex(j));
            if idx
                pts2DlastFive = [pts2DlastFive pointMatches{P_index(end-ncams+k)}(2:3,idx)'];
                dmask(j,k)=1;
            end
        end
    end

    % Set up the number of cameras to not modify (want to modify the last [numFramesToMod] poses)
    numCamsNoMod = ncams-numFramesToMod;
    if numCamsNoMod == 0; numCamsNoMod = 1; end;

    % Set up pose information for BA format
    r0 = [];
    cams = [];
    for j = 1:ncams
        iR = invK*P{end-ncams+j}(:,1:3);
        iq = quaternion(iR);
        it = invK*P{end-ncams+j}(:,4);
        r0 = [r0 iq'];
        cams = [cams; iq(2:4)' it'];
    end

    % Other SBA inputs
    pnp = size(pts3DlastFive,1); % number of parameters for each 3D point
    cnp=size(cams, 2); % the number of parameters for each camera
    mnp = 2; % the number of parameters for each image projections
    opts=[1E-03, 1E-12, 1E-12, 1E-12, 0.0]; % options vector
    cal = [K1(1,:), K1(2,2), K1(2,3)]; % calibration parameters
    % Shaping the structure and motion SBA input data
    p0=double([reshape(cams', 1, ncams*cnp) reshape(pts3DlastFive, 1, npts*pnp)]);
%     if frameCtr < 30
%         sbaMode = 'motstr';
%     else sbaMode = 'mot'; end
    % motion & structure BA
    
    if strcmp(sbaMode,'motstr')
        jacproj = 'jacprojRTS';
    elseif strcmp(sbaMode,'mot')
        jacproj = 'jacprojRT';
    elseif strcmp(sbaMode,'str')
        jacproj = 'jacprojS';
    end
    
    [ret, p, info]=sba(npts, numStrucPtsNoMod, ncams, numCamsNoMod, dmask, p0, cnp, pnp, double(pts2DlastFive), mnp, 'projRTS', jacproj, numIter, 1, opts, sbaMode, r0, cal);

    if  info(7) == 2  || info(2) / info(1) < .0001 
%                     if info(6) < numIter || info(2) / info(1) < .4

        % sba finished; retrieve the refined motion & structure from the output vector
        cams=reshape(p(1:ncams*cnp), cnp, ncams)';
        % derive the full (local) unit quaternions from their vector parts and combine with the initial rotation
        cams=[zeros(ncams, 1), cams]; % augment with a leading column
        for i=1:ncams
          % note that the cams column indices below are incremented by one to account for the extra column above!
          qrl=[sqrt(1 - (cams(i, 2)^2 + cams(i, 3)^2 + cams(i, 4)^2)), cams(i, 2), cams(i, 3), cams(i, 4)];
          qr0=r0((i-1)*4+1:i*4);
          % tmp=qrl*qr0;
          tmp=[qrl(1)*qr0(1) - dot(qrl(2:4), qr0(2:4)),  qrl(1)*qr0(2:4) + qr0(1)*qrl(2:4) + cross(qrl(2:4), qr0(2:4))];
          if(tmp(1)<0), tmp=-tmp; end; % ensure scalar part is non-negative
          cams(i, 1:4)=tmp;
        end
        badSba = 0;
        for j = 1:ncams-numCamsNoMod
            nP = [quaternion(cams(end-ncams+numCamsNoMod+j,1:4)), cams(end-ncams+numCamsNoMod+j,5:7)'];
            if sum(imag(nP(:))) == 0
                P{end-ncams+numCamsNoMod+j} = K1*nP;
            else
                badSba = 1;
            end
        end
        if badSba == 0
            if info(7) == 3
                % need to replot entire thing
                replot = true;
            end
                pts3D=reshape(p(ncams*cnp+1:ncams*cnp+npts*pnp), pnp, npts);
                pointCloud(2:4,pts3DlastFiveIndex) = pts3D;
        end
    else
        disp 'Bad SBA';
        badSbaCtr = badSbaCtr + 1;
    end


end

