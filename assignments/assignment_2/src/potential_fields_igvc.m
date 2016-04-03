clear all
close all
addpath('ME597-8-PotentialFields')
 filename = 'IGVC_Contours_inner_flipped.dat'
% [map, map_min, map_max] = load_omap('IGVCmap.jpg');
% 
% dxy = 0.1;
% pos_start = [40/dxy 5/dxy pi];
% pos_end = [50/dxy 10/dxy];
% 
% %plot_omap(1, map, pos_start, pos_end, 0.1);
% %hold on
contour_polys = import_contours(filename);
% 
% for i = 1:length(test)
%     contour = test{i}
%     plot(contour(:,1), contour(:,2))
%     hold on   
%     
% end

%%
% Vehicle start and end position
startPos = [21.5 18.5];
endPos = [6 5];

% Set up environment

% Region Bounds
posMinBound = [400, 400];
posMaxBound = [500, 500];

% Number of obstacles
numObsts = 6;
% Size bounds on obstacles
minLen.a = 1;
maxLen.a = 3;
minLen.b = 2;
maxLen.b = 6;

% Random environment generation
obstBuffer = 0.5;
maxCount = 10000;
seedNumber = rand('state');
[aObsts,bObsts,obsPtsStore] = polygonal_world(posMinBound, posMaxBound, minLen, maxLen, numObsts, startPos, endPos, obstBuffer, maxCount);
for i=1:numObsts
    obsCentroid(i,:) = (obsPtsStore(1,2*(i-1)+1:2*i)+obsPtsStore(3,2*(i-1)+1:2*i))/2;
end

% Plot random environment
figure(1); clf;
hold on;
plotEnvironment(obsPtsStore,posMinBound, posMaxBound, startPos, endPos);
hold off

% Create potential field
Katt = 0.0; % Attractive
Krep = 500; % Repulsive
r0 = .2; % Radius of Repulsion
rc0 = 6; %
Vmax = 50; % Upper bound on potential
gVmax = 10;
gVmin = -10;
% Grid up the space
dx =1;
dy = 1;
[X,Y] = meshgrid([posMinBound(1):dx:posMaxBound(1)],[posMinBound(2):dy:posMaxBound(2)]);

% Calculate potential field at each grid point
V = zeros(size(X));
[n,m] = size(X);
gV = zeros(2,n,m);
for i=1:length(X(:,1))
    for j=1:length(Y(1,:))
        % Current robot position
          pos = [X(i,j) Y(i,j)];
%         % Attractive Potential
%         V(i,j) = 1/2*Katt*norm(pos-endPos)^2; % potential
%         gV(:,i,j) = Katt*(pos-endPos);  % gradient
        
        %Repulsive potentials
        for m=1:length(contour_polys)
            % curobs = obsPtsStore(:,2*(m-1)+1:2*m);
                curpoly = contour_polys{m};
            if (inpolygon(pos(1),pos(2),curpoly(:,1),curpoly(:,2)))
                V(i,j) = Vmax;
                gV(:,i,j) = [NaN NaN];
            else
                % Find potential based on minimum distance to obstacle
                curobs = curpoly;
                curpoly = [curobs curobs([2:end, 1],:)];
                [minD,minPt, d, pt, ind] = minDistToEdges(pos, curpoly);
                if (minD < r0)
                    V(i,j) = V(i,j) + 1/2*Krep*(1/minD-1/r0)^2;
                    gV(:,i,j) = gV(:,i,j) + Krep*(-1/minD+1/r0)*(pos'-minPt')/minD^(3);
                end
                % Add potential of distance to center, to avoid getting
                % stuck on flat walls
%                 centD = norm(pos-obsCentroid(m,:));
%                 if (centD < rc0)
%                     V(i,j) =  V(i,j) + 1/2*Krep*(1/centD-1/rc0)^2;
%                     gV(:,i,j) = gV(:,i,j) + Krep*(-1/centD+1/rc0)*(pos'-obsCentroid(m,:)')/centD^(3);
%                 end
            end
            
        end
        V(i,j) = max(0,min(V(i,j),Vmax));
        gV(1,i,j) = max(gVmin,min(gV(1,i,j),gVmax));
        gV(2,i,j) = max(gVmin,min(gV(2,i,j),gVmax));
    end
    i = i
end

figure(2); clf; hold on;
surf(X,Y,V)
axis([posMinBound(1) posMaxBound(1) posMinBound(2) posMaxBound(2) 0 Vmax])


