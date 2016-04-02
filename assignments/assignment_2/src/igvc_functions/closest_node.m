function [node, index] = closest_node(nodes, x, y)
    index = 0;
    i = length(nodes);
    random_node = nodes(i, :);
    dx = x - random_node(1);
    dy = y - random_node(2);
    dmin = sqrt(dx^2 + dy^2);

    for i = 1:length(nodes)
        dx = x - nodes(i, 1);
        dy = y - nodes(i, 2);
        d = sqrt(dx^2 + dy^2);

        if (d ~= 0 && d < dmin)
            dmin = d;
            index = i;
        end
    end

    node = nodes(index, :);
    index = index;
end