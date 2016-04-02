function dist = point_edge_dist(p, edge)
    % NOTE: CALCULATES PERPENDICULAR DISTANCE ONLY
    %       for euclidean distance use distancePointEdge()
    x0 = p(1);
    y0 = p(2);
    x1 = edge(1);
    y1 = edge(2);
    x2 = edge(3);
    y2 = edge(4);

    n = ((y2 - y1) * x0) - ((x2 - x1) * y0) + (x2 * y1) - (y2 * x1);
    d = sqrt((y2 - y1)^2 + (x2 - x1)^2);

    dist = abs(n) / d;
end