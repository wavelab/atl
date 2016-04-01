source('util.m');
source('planning.m');

I = imread('IGVCmap.jpg');
map = im2bw(I, 0.7);  % Convert to 0 - 1 image
map = 1 - flipud(map)'; % Convert to 0 free, 1 occupied and flip.
% map = zeros(100, 100);
[M, N] = size(map);  % Map size
map_min = [1 1];
map_max = [M N];

% robot start position
dxy = 0.1;
pos_start = [40 5 pi];
pos_end = [50 10];


% plot
% sample map
disp('Sample map');
tic;
nb_samples = 1000;
[samples, milestones] = sample_map(
    nb_samples,
    map,
    map_min,
    map_max,
    pos_start(1:2),
    pos_end
);
plot_samples(1, samples, milestones);
plot_map(1, map, pos_start, pos_end, 0.1);
drawnow;
toc;
disp('');

% create graph, find furthest distance between nodes and search
disp('Create edges');
tic;
edges = connect_edges_omap(map, milestones, 10);
toc;
disp('');

% save milestones.mat milestones
% save edges.mat edges
% load milestones.mat
% load edges.mat

% search graph
disp('Search graph');
tic;
[spath, sdist] = astar(milestones, edges, 1, 2);
for i = 1:length(spath) - 1
    plot(
        [spath(i, 1), spath(i + 1, 1)],
        [spath(i, 2), spath(i + 1, 2)],
        'g-',
        'LineWidth',
        3
    );
end
toc;
disp('');
save spath.mat spath
save sdist.mat sdist
% print -djpg -color planning.jpg

drawnow;
pause;
