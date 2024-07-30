%% Animate agent-based model of the dynamics of flocking birds

function animate_agents(model_dimension,X,V,status,obstacles,dt,filename)
    video = VideoWriter(filename,'MPEG-4');
    open(video)

    figure
    
    if model_dimension == 2
        % Determine extreme positions for axis limits
        X_min_1 = min(min(X(:,1,:))); X_max_1 = max(max(X(:,1,:)));
        X_min_2 = min(min(X(:,2,:))); X_max_2 = max(max(X(:,2,:)));

        if obstacles.status == true
            X_min_1 = min(X_min_1,min(obstacles.centers(:,1)-obstacles.radii));
            X_max_1 = max(X_max_1,max(obstacles.centers(:,1)+obstacles.radii));
            X_min_2 = min(X_min_2,min(obstacles.centers(:,2)-obstacles.radii));
            X_max_2 = max(X_max_2,max(obstacles.centers(:,2)+obstacles.radii));
        end
        
        % Plot every 10th time step
        for i = 1:10:size(X,3)
            clf(gcf)
            hold on
    
            X_current_1 = X(:,1,i);
            X_current_2 = X(:,2,i);
            V_current_1 = V(:,1,i);
            V_current_2 = V(:,2,i);

            % Plot obstacles
            if obstacles.status == true
                for j = 1:size(obstacles.centers,1)
                    center = obstacles.centers(j,:);
                    radius = obstacles.radii(j);
                    rectangle('Position',[center-radius 2*radius 2*radius], ...
                        'Curvature',[1 1],'FaceColor','#FFC0CB')
                end
            end

            % Plot followers (blue)
            plot(X_current_1(status(:,i)==0), ...
                X_current_2(status(:,i)==0), ...
                'b.','MarkerSize',30);

            % Plot leaders (red)
            plot(X_current_1(status(:,i)==1), ...
                X_current_2(status(:,i)==1), ...
                'r.','MarkerSize',30);

            % Plot velocity arrows
            quiver(X_current_1,X_current_2,V_current_1,V_current_2, ...
                'k','LineWidth',1,'MarkerSize',2)

            % Set axis limits
            xlim([X_min_1 X_max_1])
            ylim([X_min_2 X_max_2])

            xlabel('x')
            ylabel('y')
            title(['t = ' num2str((i-1)*dt)]);

            % Save a frame as image at t = t_frame
            t_frame = 70;
            if (i - 1) * dt * 100 == t_frame
                saveas(gcf,strcat(filename,'_t_',int2str(t_frame),'.png'))
            end

            writeVideo(video,getframe(gcf))
        end
        
    elseif model_dimension == 3
        % Determine extreme positions for axis limits
        X_min_1 = min(min(X(:,1,:))); X_max_1 = max(max(X(:,1,:)));
        X_min_2 = min(min(X(:,2,:))); X_max_2 = max(max(X(:,2,:)));
        X_min_3 = min(min(X(:,3,:))); X_max_3 = max(max(X(:,3,:)));

        % Plot every 10th time step
        for i = 1:10:size(X,3)
            clf(gcf)
            hold on
    
            X_current_1 = X(:,1,i);
            X_current_2 = X(:,2,i);
            X_current_3 = X(:,3,i);
            V_current_1 = V(:,1,i);
            V_current_2 = V(:,2,i);
            V_current_3 = V(:,3,i);

            % Plot followers (blue)
            plot3(X_current_1(status(:,i)==0), ...
                X_current_2(status(:,i)==0), ...
                X_current_3(status(:,i)==0), ...
                'b.','MarkerSize',30);

            % Plot leaders (red)
            plot3(X_current_1(status(:,i)==1), ...
                X_current_2(status(:,i)==1), ...
                X_current_3(status(:,i)==1), ...
                'r.','MarkerSize',30);

            % Plot velocity arrows
            quiver3(X_current_1,X_current_2,X_current_3,V_current_1,V_current_2,V_current_3, ...
                'k','LineWidth',1,'MarkerSize',2)

            % Set axis limits
            xlim([X_min_1 X_max_1])
            ylim([X_min_2 X_max_2])
            zlim([X_min_3 X_max_3])

            xlabel('x', "FontSize", 18)
            ylabel('y', "FontSize", 18)
            zlabel('z', "FontSize", 18)
            title("''Unaligned'' Flight", 'FontSize',18)
            %title(['t = ' num2str((i-1)*dt)]);

            view(3)

            writeVideo(video,getframe(gcf))
        end
    end
end