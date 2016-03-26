I = imread('IGVCmap.jpg');
map = im2bw(I, 0.7);  % Convert to 0 - 1 image
map = 1 - flipud(map)'; % Convert to 0 free, 1 occupied and flip.
[M, N] = size(map);  % Map size

% robot start position
dxy = 0.1;
pos_start = [40 5 pi];

% target location
pos_end = [50 10];

% Set up the map
xMax = [M N]; % State bounds
xMin = [1 1];
xR = xMax - xMin;

% sample
nb_samples = 500;
samples_x = int32(xR(1) * rand(nb_samples, 1) + xMin(1));
samples_y = int32(xR(2) * rand(nb_samples, 1) + xMin(2));
samples = [samples_x, samples_y];

milestones = [pos_start(1:2); pos_end;];
for i = 1:length(samples)
    sample = samples(i, :);

    if map(sample(1), sample(2)) == 0
        map(sample(1), sample(2))
        milestones = [milestones; sample];
    end
end

% plot samples
figure(1);
hold on;
plot(samples(:, 1), samples(:, 2), 'r.');
plot(milestones(:, 1), milestones(:, 2), 'bo');
nM = length(milestones(:, 1));
disp('Time to generate milestones');
% toc;

% plot map
% figure(1);
% hold on;
colormap('gray');
imagesc(1 - map');
plot(pos_start(1)/dxy, pos_start(2)/dxy, 'ro', 'MarkerSize',10, 'LineWidth', 3);
plot(pos_end(1)/dxy, pos_end(2)/dxy, 'gx', 'MarkerSize',10, 'LineWidth', 3 );
axis equal
pause
