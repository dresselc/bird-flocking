%% Plot flock's average speed over time

function plot_velocity(model_dimension, V, timesteps)
    tspan = linspace(1, timesteps, timesteps);
    
    %% calculate average speed and spread
    n = size(V, 1); % number of agents
    v_mag = zeros(n, timesteps); 

    % compute magnitude of velocity (speed) of each agent for each timestep
    if model_dimension == 2
        v_mag(:,:) = sqrt(V(:,1,:).^2 + V(:,2,:).^2);
    else
        v_mag(:,:) = sqrt(V(:,1,:).^2 + V(:,2,:).^2 + V(:,3,:).^2);
    end

    v_avg = mean(v_mag, 1); % compute average speed of agents for each timestep
    v_std = std(v_mag, 1);  % compute standard deviation of speeds for each timestep
    
    % compute speed +/- its standard deviation
    v_upper = v_avg + v_std;
    v_lower = v_avg - v_std;
    
    %% plot average speed and spread vs time
    figure
    plot(tspan, v_avg)
    hold on
    plot(tspan, v_lower, "LineStyle", "--", "Color", "#0072BD")
    plot(tspan, v_upper, "LineStyle", "--", "Color", "#0072BD")

    % axis and label settings
    xlim([0 timesteps])
    ax = gca;
    ax.FontSize = 14;
    ylabel("Average velocity", 'FontSize', 18)
    xlabel("time", 'FontSize',18)
    l = legend("$\mu$", "$\mu \pm \sigma$", 'interpreter', 'latex');
    fontsize(l,16,'points')
    hold off


end