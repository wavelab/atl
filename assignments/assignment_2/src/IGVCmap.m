source('util.m');
source('planning.m');

I = imread('IGVCmap.jpg');
map = im2bw(I, 0.7);  % Convert to 0 - 1 image
map = 1 - flipud(map)'; % Convert to 0 free, 1 occupied and flip.
[M, N] = size(map);  % Map size

% robot start position
% dxy = 0.1;
dxy = 0.1;
pos_start = [40 5 pi];

% target location
pos_end = [50 10];

% set up the map
xMax = [M N]; % State bounds
xMin = [1 1];
xR = xMax - xMin;




%% Find shortest path
% O = [start 0 dmax 0];  % open set (node, backtrack, lower bound cost, current cost)
% C = [];  % close set (node, backtrack, lower bound cost, current cost)
% done = 0;
% t = 0;
%
% while (~done)
%     t = t + 1
%
%     % find best node in open set
%     [val, best] = min(O(:,3));
%     bestnode = O(best,:);
%
%     % move best to closed set
%     C = [C; bestnode];
%
%     % check end condition
%     if (bestnode(1) == finish)
%         done = 1;
%         continue;
%     end
%
%     % get all neighbours of best node
%     neigh = find(e(bestnode(1),:)==1);
%
%     % process each neighbour
%     for i = 1:length(neigh)
%         % if neighbour is in closed set, skip
%         found = find(C(:,1)==neigh(i),1);
%         if (length(found)==1)
%             continue;
%         end
%         dtogo = norm(nodes(neigh(i),:)-nodes(finish,:));
%         dcur = bestnode(4)+D(bestnode(1),neigh(i));
%         found = find(O(:,1)==neigh(i),1);
%
%         % if neighbour is not in open set, add it
%         if (length(found) == 0)
%             if (useAstar)
%                 % A-star
%                 O = [O; neigh(i) bestnode(1) dtogo+dcur dcur];
%             else
%                 % Dijkstra
%                 O = [O; neigh(i) bestnode(1) dcur dcur];
%             end
%
%         % if neighbour is in open set, check if new route is better
%         else
%             if (dcur < O(found,4))
%                 if (useAstar)
%                     % A-star
%                     O(found, :) = [neigh(i) bestnode(1) dtogo+dcur dcur];
%                 else
%                     % Dijkstra
%                     O(found, :) = [neigh(i) bestnode(1) dcur dcur];
%                 end
%             end
%         end
%     end
%
%     % remove best node from open set
%     O = O([1:best-1 best+1:end], :);
%
%     % % plot active nodes for this step
%     % figure(1);
%     % hold on;
%     % plot(nodes(C(:,1),1),nodes(C(:,1),2), 'ko','MarkerSize',6,'LineWidth',2);
%     % plot(nodes(bestnode(1),1),nodes(bestnode(1),2), 'go','MarkerSize',6,'LineWidth',2);
%     % for i=1:length(neigh)
%     %     plot(nodes(neigh(i),1),nodes(neigh(i),2), 'mo');
%     %     plot([nodes(bestnode(1),1) nodes(neigh(i),1)],[nodes(bestnode(1),2) nodes(neigh(i),2)], 'm');
%     % end
% end
%
% % Find and plot final path through back tracing
% % done = 0;
% % cur = finish;
% % curC = find(C(:,1)==finish);
% % prev =  C(curC,2);
% % i = 2;
% % while (~done)
% %     if (prev == start)
% %         done = 1;
% %     end
% %
% %     figure(1);
% %     hold on;
% %     plot([nodes(prev,1) nodes(cur,1)], [nodes(prev,2) nodes(cur,2)],'g','LineWidth',2)
% %     cur = prev;
% %     curC = find(C(:,1)==cur);
% %     prev = C(curC,2);
% % end
% % Cend = find(C(:,1)==finish);
% % plen = C(Cend,4)
%
% % sample map
nb_samples = 10;
[samples, milestones] = sample_map(nb_samples, map, xR, xMin, pos_start, pos_end);
[edges, dists] = create_closest_edges(milestones);

pos_start
[n, i] = closest_node(milestones, pos_start(1), pos_start(2))
% closest_node(milestones, pos_end(1), pos_end(2))
% edges
% dists




% plot results
plot_samples(1, samples, milestones);
plot_map(1, map, pos_start, pos_end, dxy);
drawnow;
pause;
