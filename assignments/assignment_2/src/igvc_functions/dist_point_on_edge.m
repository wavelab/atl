function p = dist_point_on_edge(d, edge)
    lp1 = edge(1);
    lp2 = edge(2);

    % calculate unit vector
    v = lp2 - lp1;
    u = v / norm(v);

    % calculate distant point along edge
    p = lp1 + d * u;
end
