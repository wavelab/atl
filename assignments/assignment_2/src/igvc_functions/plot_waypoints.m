function plot_waypoints(fig_index, waypoints)
    figure(fig_index);
    hold on;

    subplot(3, 1, 1);
    plot(waypoints(:, 1), waypoints(:, 2), 'ro-');
end