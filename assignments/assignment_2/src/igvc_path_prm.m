source('util.m');
source('planning.m');


% parameters
nb_samples = 1000;
dxy = 0.1;
pos_start = [40/dxy 5/dxy pi];
pos_end = [50/dxy 10/dxy];

% prm
[map, map_min, map_max] = load_omap('IGVCmap.jpg');
milestones = prm_sample(
	nb_samples,
	map,
	pos_start,
	pos_end,
	map_min,
	map_max
);
[spath, sdist] = prm_search(map, milestones);

% save milestones.mat milestones
% save edges.mat edges
% save spath.mat spath
% save sdist.mat sdist

% load milestones.mat
% load edges.mat
% load spath.mat
% load sdist.mat

for i = 1:length(spath) - 1
    plot(
        [spath(i, 1), spath(i + 1, 1)],
        [spath(i, 2), spath(i + 1, 2)],
        'g-',
        'LineWidth',
        3
    );
end

% print -djpg -color planning.jpg
drawnow;
pause;
