%% Plot flock's average angle of velocity over time

function plot_angle(model_dimension, V, timesteps)
    tspan = linspace(1, timesteps, timesteps);

    %% two dimension model
    if model_dimension == 2

        %% calculate mean and standard deviation of velocity angles
        angles = atan2(V(:,2,:), V(:,1,:)); % compute angles from X and Y velocity components of each agent for each timestep
        std_angles = squeeze(std(angles));  % compute standard deviation of angles for each timestep
        angles = squeeze(mean(angles,1));   % compute mean angle for each timestep

        % calculate +/- one standard deviation of theta
        upper = angles + std_angles;
        lower = angles - std_angles;

        %% plot mean angle and spread over time
        figure
        plot(tspan, angles)
        hold on
        plot(tspan, lower, "LineStyle", "--", "Color", "#0072BD")  
        plot(tspan, upper, "LineStyle", "--", "Color", "#0072BD")   

        % axis and label settings
        xlim([0 timesteps])
        ax = gca;
        ax.FontSize = 14;
        ylabel("Average angle of velocity", 'FontSize', 18)
        xlabel("time", 'FontSize',18)
        l = legend("$\theta$", "$\theta \pm \sigma$", 'interpreter', 'latex');
        fontsize(l,16,'points')
        hold off
            
    %% three dimension model
    else
        %% calculate mean and standard deviation of velocity angles 
        [theta, phi, ~] = cart2sph(V(:,1,:), V(:,2,:), V(:,3,:));   % convert velocity vectors of each agent into spherical coordinates for each timestep 
        std_theta = squeeze(std(theta,1));  % calculate standard deviation of angle theta (left/right)
        std_phi = squeeze(std(phi,1));      % calculate standard deviation of angle phi (up/down)
        theta = squeeze(mean(theta,1));     % calculate mean theta for each timestep
        phi = squeeze(mean(phi,1));         % calculate mean phi for each timestep
        
        % calculate +/- one standard deviation of theta and phi
        upper_theta = theta + std_theta;
        lower_theta = theta - std_theta;
        upper_phi = phi + std_phi;
        lower_phi = phi - std_phi;
        
        %% plot mean angle theta and spread over time
        figure
        plot(tspan, theta)  
        hold on
        plot(tspan, lower_theta, "LineStyle", "--", "Color", "#0072BD")
        plot(tspan, upper_theta, "LineStyle", "--", "Color", "#0072BD")

        % axis and label settings
        xlim([0 timesteps])
        ax = gca;
        ax.FontSize = 14;
        ylabel("Average theta", 'FontSize', 18)
        xlabel("time", 'FontSize',18)
        l = legend("$\theta$", "$\theta \pm \sigma$", 'interpreter', 'latex');
        fontsize(l,16,'points')
        hold off
        
        %% plot mean angle phi and spread over time
        figure
        plot(tspan, phi)    
        hold on
        plot(tspan, lower_phi, "LineStyle", "--", "Color", "#0072BD")  
        plot(tspan, upper_phi, "LineStyle", "--", "Color", "#0072BD")  

        % axis and label settings
        xlim([0 timesteps])
        ax = gca;
        ax.FontSize = 14;
        ylabel("Average phi", 'FontSize', 18)
        xlabel("time", 'FontSize',18)
        l = legend("$\phi$", "$\phi \pm \sigma$", 'interpreter', 'latex');
        fontsize(l,16,'points')
        hold off
        
        %% plot average angles theta and phi over time
        figure
        plot(tspan, theta)  
        hold on 
        plot(tspan, phi)   

        % axis and label settings
        xlim([0 timesteps])
        ax = gca;
        ax.FontSize = 14;
        ylabel("Average angle of velocity", 'FontSize', 18)
        xlabel("time", 'FontSize',18)
        l = legend("$\theta$", "$\phi$", 'interpreter', 'latex');
        fontsize(l,16,'points')
        hold off
    end
end