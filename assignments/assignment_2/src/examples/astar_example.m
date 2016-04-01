source('../planning.m');

%% Generate a graph
n = 10; % Nodes
s = 10; % Space

% Random nodes
nodes = s * rand(n, 2);
e = sparse(zeros(n, n));
D = sparse(zeros*ones(n, n));


% add closest p edges
edges = connect_edges(nodes, 5);
[dmax, start, finish] = calculate_furthest_distance_betweeen_nodes(nodes);

% plot graph
plot_nodes(1, nodes, start, finish);
[spath, sdist] = astar(nodes, edges, start, finish);


nodes(start, :)
nodes(finish, :)
for i = 1:length(spath) - 1
    plot(
        [spath(i, 1), spath(i + 1, 1)],
        [spath(i, 2), spath(i + 1, 2)],
        'g-',
        'LineWidth',
        3
    );
end

drawnow;
pause;
