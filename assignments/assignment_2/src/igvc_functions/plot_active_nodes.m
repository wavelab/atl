function plot_active_nodes(fig_index, nodes, best_node, neigh, C)
    figure(1);
    hold on;
    plot(nodes(C(:,1), 1), nodes(C(:,1), 2), 'ko','MarkerSize',6,'LineWidth',2);
    plot(nodes(best_node(1), 1), nodes(best_node(1), 2), 'go','MarkerSize',6,'LineWidth',2);

    for i = 1:length(neigh)
        plot(nodes(neigh(i), 1), nodes(neigh(i), 2), 'mo');
        plot([nodes(best_node(1), 1) nodes(neigh(i), 1)],...
            [nodes(best_node(1), 2) nodes(neigh(i), 2)],...
            'm');
    end
    drawnow;
end