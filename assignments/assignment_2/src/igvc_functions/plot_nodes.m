

function plot_nodes(fig_index, nodes, start, finish)
    figure(fig_index);
    hold on;
    n = length(nodes);

    plot(nodes(:, 1), nodes(:, 2), 'ko');
    for i = 1:n
        for j = i:n
            if (e(i, j)==1)
                plot([nodes(i, 1) nodes(j, 1)], [nodes(i, 2) nodes(j, 2)], 'k');
            end
        end
    end
    plot(nodes(start, 1), nodes(start, 2), 'bo', 'MarkerSize', 6, 'LineWidth', 2);
    plot(nodes(finish, 1), nodes(finish, 2), 'ro', 'MarkerSize', 6, 'LineWidth', 2);
end