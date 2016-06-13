% COLOURMAPPATH Plots the path of a colourmap through colourspace
%
% Usage: colourmappath(map, N, colspace, fig)
%
% Arguments:
%            map - The colourmap to be plotted
%              N - The nmber of slices through the colourspace to plot.
%       colspace - String 'rgb' or 'lab' indicating the colourspace to plot. 
%            fig - Optional figure number to use.
%
% The colourspace is represented as a series of semi-transparent 2D slices which
% hopefully lets you visualise the colourspace and the colourmap path
% simultaneously.
%
% Note that for some reason repeated calls of this function to render a
% colourmap path in RGB space seem to ultimately cause MATLAB to crash (under
% MATLAB 2013b on OSX).  Rendering paths in CIELAB space cause no problem.
%
% See also: LABMAPLIB, EQUALISECOLOURMAP, VIEWLABSPACE, SINERAMP

% Copyright (c) 2013-2014 Peter Kovesi
% Centre for Exploration Targeting
% The University of Western Australia
% peter.kovesi at uwa edu au
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in 
% all copies or substantial portions of the Software.
%
% The Software is provided "as is", without warranty of any kind.

% PK October 2013

function colourmappath(map, N, colspace, fig)
    
    mapref = map;

    if ~exist('colspace', 'var'), colspace = 'lab'; end
    if ~exist('fig', 'var'), fig = 1; end

    s = 4;  % Spacing of points on colourmap to plot
    dotcolour = [0.7 0.7 0.7]; % Dots on colourmap path
    
    %% RGB plot ---------------------------------------------------------
    if strcmpi(colspace, 'rgb')
        
        R = 255;
        im = zeros(R, R, 3, 'uint8');
        [x,y] = meshgrid(1:R,1:R);
        
        figure(fig), clf, axis([-1 256 -1 256 -1 256])
        
        % Draw N red-green planes of increasing blue value through RGB space 
        for b = 0:R/N:R
            for r = 1:R
                for g = 1:R
                    im(g, r, :) = [r; g; round(b)];
                end
            end
            
            h = warp(b*ones(R,R), im); hold on
            set(h, 'FaceAlpha', 0.9);
        end
    
        % Plot the colourmap path through the RGB colourspace
        map = map*255;
        line(map(:,1), map(:,2), map(:,3), 'linewidth', 2, 'color', [0 0 0]);
        hold on
        
        % Plot a dot for every 's' point along the colourmap to indicate the
        % spacing of colours in the colourmap
        plot3(map(1:s:end,1), map(1:s:end,2), map(1:s:end,3), '.', ...
              'Color', dotcolour, 'MarkerSize', 12);
        
        hold off
        
        xlabel('Red');
        ylabel('Green');
        zlabel('Blue');
    
        axis vis3d
        
    %    export_fig picture.png -m2 -transparent -png 
    
    %% CIELAB plot ----------------------------------------------------
    elseif strcmpi(colspace, 'lab')

        
        if iscell(mapref)  % We have a cell array of colourmaps
            figure(fig), clf,     axis([-110 110 -110 110  0 100])
            for n = 1:length(mapref)
                lab = rgb2lab(mapref{n});
                line(lab(:,2), lab(:,3), lab(:,1), 'linewidth', 2, 'color', [0 0 0])
                hold on
                plot3(lab(1:s:end,2), lab(1:s:end,3), lab(1:s:end,1), '.', ...
                      'Color', [.7 .7 .7],'MarkerSize',10)
            end
        
        elseif  ~isempty(map) % Single colourmap supplied
            lab = rgb2lab(mapref);
            
            figure(fig), clf,     axis([-110 110 -110 110  0 100])
            line(lab(:,2), lab(:,3), lab(:,1), 'linewidth', 2, 'color', [0 0 0])
            hold on
            plot3(lab(1:s:end,2), lab(1:s:end,3), lab(1:s:end,1), '.', ...
                  'Color', dotcolour, 'MarkerSize',10)
            %    line([0 0], [0 0], [0 100])  % Line up a = 0, b = 0 axis
        end
        
        %% Generate Lab image slices
    
        % Define a*b* grid for image
        scale = 1;
        [a, b] = meshgrid([-127:1/scale:127]);
        [rows,cols] = size(a);
    
        %  Generate N equi-spaced lightness levels between 5 and 95.
        for L = 5:90/(N-1):95  % For each lightness level...
            
            % Build image in lab space
            lab = zeros(rows,cols,3);    
            lab(:,:,1) = L;
            lab(:,:,2) = a;
            lab(:,:,3) = b;
            
            % Generate rgb values from lab
            rgb = applycform(lab, makecform('lab2srgb'));
            
            % Invert to reconstruct the lab values
            lab2 = applycform(rgb, makecform('srgb2lab'));
            
            % Where the reconstructed lab values differ from the specified values is
            % an indication that we have gone outside of the rgb gamut.  Apply a
            % mask to the rgb values accordingly
            mask = max(abs(lab-lab2),[],3);
            
            for n = 1:3
                rgb(:,:,n) = rgb(:,:,n).*(mask<2);  % tolerance of 2
            end
            
            [x,y,z,cdata] = labsurface(lab, rgb, L, 100);
            h = surface(x,y,z,cdata);  shading interp; hold on
            set(h, 'FaceAlpha', 0.9);
            
        end % for each lightness level
        
        % Generate axis tick values
        tickval = [-100 -50 0 50 100];
        tickcoords = tickval; %scale*tickval + cols/2;
        ticklabels = {'-100'; '-50'; '0'; '50'; '100'};
        
        set(gca, 'xtick', tickcoords);
        set(gca, 'ytick', tickcoords);
        set(gca, 'xticklabel', ticklabels);
        set(gca, 'yticklabel', ticklabels);    
        
        fontsize = 12;
        fontweight = 'bold';
        
        % Label axes.  Note option for manual placement for a and b
        manual = 0;
        if ~manual
            xlabel('a', 'Fontsize', fontsize, 'FontWeight', fontweight);
            ylabel('b', 'Fontsize', fontsize, 'FontWeight', fontweight);
        else
            text(0, -170, 0, 'a', 'Fontsize', fontsize, 'FontWeight', ...
                 fontweight);        
            text(155, 0, 0, 'b', 'Fontsize', fontsize, 'FontWeight', ...
                 fontweight);
        end
        zlabel('L', 'Fontsize', fontsize, 'FontWeight', fontweight);
        
        axis vis3d
        
        grid on, box on
        view(64, 28)
        hold off
        
    else
        error('colspace must be rgb or lab')
    end
    
%-------------------------------------------------------------------
% LABSURFACE
%
% Idea to generate lab surfaces.  Generate lab surface image, Sample a
% parametric grid of x, y, z and colour values.  Texturemap the colour values
% onto the surface.
%
% Find first and last rows of image containing valid values.  Interpolate y
% over this range.  For each value of y find first and last valid x values.
% Interpolate x over this range sampling colour values as one goes.

function [x, y, z, cdata] = labsurface(lab, rgb, zval, N)
    
    x = zeros(N,N);
    y = zeros(N,N);
    z = zeros(N,N);
    cdata = zeros(N,N,3);
    
    % Determine top and bottom edges of non-zero section of image
    gim = sum(rgb, 3);  % Sum over all colour channels
    rowsum = sum(gim, 2); % Sum over rows
    
    ind = find(rowsum);
    top = ind(1);
    bottom = ind(end);
    rowvals = round(top + (0:N-1)/(N-1)*(bottom-top));

    rowind = 1;
    for row = rowvals
       % Find first and last non-zero elements in this row
       ind = find(gim(row,:));
       left = ind(1);
       right = ind(end);
       colvals = round(left + (0:N-1)/(N-1)*(right-left));

       % Fill matrices
       colind = 1;
       for col = colvals
           x(rowind,colind) = lab(row,col,2);
           y(rowind,colind) = lab(row,col,3);          
           
           z(rowind,colind) = zval;
           cdata(rowind, colind, :) = rgb(row, col, :);
           
           colind = colind+1;
       end
        
       rowind = rowind+1;
    end