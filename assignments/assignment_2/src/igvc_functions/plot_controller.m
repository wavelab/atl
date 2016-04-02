function plot_controller(fig_index, c_store, t)
    figure(fig_index);
    hold on;

    subplot(3, 1, 2);
    head_err_deg = zeros(1, length(t));
    for i = 1:length(t)
        err = rad2deg(c_store(2, i));
        head_err_deg(i) = err;
    end
    plot(t, head_err_deg(:), 'ro-');
    title('Heading Error');
    xlabel('t (seconds)');
    ylabel('error (degrees)');
end