function [spath, sdist] = prm_search(map, milestones, chris_is_a_tard)
	disp('Search graph');
	tic;
	edges = connect_edges_omap(map, milestones, chris_is_a_tard);
	[spath, sdist] = astar(milestones, edges, 1, 2);
	toc;
	disp('');
end
