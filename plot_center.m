%% Plot the center of the flock over time

function flock_size = plot_center(model_dimension,X,timesteps)
    X_center = squeeze(mean(X,1));   % Central (mean) position at each time step
    flock_size = zeros(1,timesteps); % Mean distance from central position at each time step
    for i = 1:timesteps
        flock_size(i) = mean(sqrt(sum((X(:,:,i)-X_center(:,i)').^2,2)));
    end

    figure
    hold on
    
    if model_dimension == 2
        % Plot central position (color & size indicate mean distance from central position)
        scatter(X_center(1,:),X_center(2,:),flock_size,flock_size,'filled')
        colorbar
    
        % Plot starting central position (green)
        scatter(X_center(1,1),X_center(2,1),max(flock_size),'g','filled')
    
        % Plot ending central position (red)
        scatter(X_center(1,end),X_center(2,end),max(flock_size),'r','filled')
    
        % Plot an "x" every 100 time steps
        plot(X_center(1,1:100:end),X_center(2,1:100:end),'kx','MarkerSize',7,'LineWidth',2)
    
        xlabel('x')
        ylabel('y')

    elseif model_dimension == 3
        % Plot central position (color & size indicate mean distance from central position)
        scatter3(X_center(1,:),X_center(2,:),X_center(3,:),flock_size,flock_size,'filled')
        colorbar
    
        % Plot starting central position (green)
        scatter3(X_center(1,1),X_center(2,1),X_center(3,1),max(flock_size),'g','filled')
    
        % Plot ending central position (red)
        scatter3(X_center(1,end),X_center(2,end),X_center(3,end),max(flock_size),'r','filled')
    
        % Plot an "x" every 100 time steps
        plot3(X_center(1,1:100:end),X_center(2,1:100:end),X_center(3,1:100:end),'kx','MarkerSize',7,'LineWidth',2)
    
        xlabel('x')
        ylabel('y')
        zlabel('z')

        view(3)
    end
end