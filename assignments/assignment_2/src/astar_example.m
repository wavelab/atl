source('planning.m');

%% Generate a graph
n = 10; % Nodes
s = 10; % Space

% Random nodes
nodes = s * rand(n, 2);
e = sparse(zeros(n, n));
D = sparse(zeros*ones(n, n));


% add closest p edges
[e, D] = create_closest_edges(nodes);

% pick furthest apart start and end node
[dmax, start, finish] = calculate_furthest_distance_betweeen_nodes(nodes);

% plot graph
plot_nodes(1, nodes, start, finish);
astar(nodes, start, finish, dmax, e, D);

drawnow;
pause;
