%% Calculate dVdt of agents based on interaction forces

function dvdt = velocity(n_timestep,t_del,dt,X,V,N,M,C_rep,ep,C_ali,C_att,status_current)
    if n_timestep>round(t_del/dt)
        X_del = X(:,:,round(n_timestep-t_del/dt));
        V_del = V(:,:,round(n_timestep-t_del/dt));

        %% Find each agent's M nearest neighbors 
        d_agents = zeros(N);
        for i = 1:N
            d_agents(i,:) = sqrt(sum((X_del-X_del(i,:)).^2,2));
        end
        d_agents(d_agents==0) = Inf;
        [~,d_index] = sort(d_agents,2);
        nearest_neighbors = d_index(:,1:M);

        %% Compute social interactions
        repulsion  = zeros(size(V_del));
        alignment  = zeros(size(V_del));
        attraction = zeros(size(V_del));
        
        % compute forces based on velocity and postition of nearest neighbors
        for i = 1:M
            rep_1 = X_del(nearest_neighbors(:,i),:)-X_del;
            rep_2 = sqrt(sum((X_del(nearest_neighbors(:,i),:)-X_del).^2,2))+ep;
            repulsion = repulsion+rep_1./rep_2;

            alignment = alignment+(V_del(nearest_neighbors(:,i),:)-V_del);

            attraction = attraction+(X_del(nearest_neighbors(:,i),:)-X_del);
        end
        
        % multiply forces by coefficients
        repulsion  = -C_rep*repulsion;
        alignment  = C_ali/M*alignment;
        attraction = C_att*attraction;
        
        % include alignment and attraction forces only if agent is a follower
        dvdt = (1-status_current).*alignment+(1-status_current).*attraction+repulsion;
    else
        dvdt = 0;
    end
end