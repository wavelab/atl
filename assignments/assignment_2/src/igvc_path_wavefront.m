% source('lib/wavefront.m');
% source('lib/shortest_wavefront_path.m');
% source('lib/image_to_binary_map.m');
addpath('wavefront_scripts/')
map = imread('IGVCmap.jpg');
map = image_to_binary_map(map);
imshow(map);
dxy = 0.1;
start_pos = [4.0, 0.5];
end_pos = [5.0, 1.0];


% convert start and end points to grid locations
start_pos = start_pos * 10 / dxy;
end_pos = end_pos *10 / dxy;

[wavefrontmap, path] = wavefront(map, start_pos, end_pos, 0, 1);
imagesc(wavefrontmap);

set(gca, 'YDir', 'normal')
hold on
plot(path(:,1), path(:,2), '-r');
colorbar;
pause
