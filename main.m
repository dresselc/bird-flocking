%% Agent-based model of the dynamics of flocking birds

seed = randi(9999);
rng(seed)

%% Specify two- or three-dimensional model
model_dimension = 2; % 2 or 3

%% Specify presence of obstacles
obstacles.status = false; % true (obstacles) or false (no obstacles)
obstacles.status = obstacles.status & (model_dimension == 2); % obtacles not currently implemented for 3D model

if obstacles.status == true
    obstacles.centers = [1000 0];
    obstacles.radii   = [100]';
end

%% Parameters
N       = 200;  % Number of agents
M       = 7;    % Number of nearest neighbors used to compute social interactions
dt      = 0.1;  % Time step
t_f     = 3e2;  % Final time
l       = 200;  % Side length of square/cube in which the agents are initially uniformly distributed
t_ref   = 800;  % Refractory time (minimum time as a follower before transitioning to a leader)
p_f2l_0 = 2e-4; % Initial probability of follower-to-leader transition
t_per   = 700;  % Persistence time (maximum time as a leader before transitioning to a follower
d_per   = 300;   % Persistence distance (maximum distance between a leader and its nearest neighbor before transitioning to a follower)
C_rep   = 2.5;  % Repulsion interaction strength
ep      = 1e-6; % Small constant to prevent arbitrarily large repulsion interactions
C_ali   = 4;   % Alignment interaction strength
C_att   = 0.01; % Attraction interaction strength
t_del   = 0.1;  % Delay time (time required for agents to react to their surroundings)

% Initial velocity
if model_dimension == 2
    V_0 = [10 0];
elseif model_dimension == 3
    V_0 = [1 0 0];
end


tic

%% Initialize arrays
timesteps = floor(t_f/dt)+1;                    % Number of timesteps
X         = zeros(N,model_dimension,timesteps); % Position
V         = zeros(N,model_dimension,timesteps); % Velocity
status    = false(N,timesteps);                 % Leader/follower status (0 = follower, 1 = leader)

%% Initial conditions
X_current      = l*rand(N,model_dimension); X(:,:,1) = X_current; % Position (initially uniformly distributed in square/cube)
V_current      = zeros(N,model_dimension) + V_0;                  % Velocity (with initial velocity)
status_current = false(N,1); status(:,1) = status_current;        % Leader/follower status (initially all followers; 0 = follower, 1 = leader)
p_f2l          = p_f2l_0*ones(N,1);                               % Probability of follower-to-leader transition (initially all the initial probability)
n_status       = ceil(t_ref/dt)*ones(N,1);                        % Number of time steps since last leader/follower status change

%% Iterate model
n_timestep = 1;

while n_timestep<timesteps
    n_timestep = n_timestep + 1; % Increment time step

    %% Follower-to-leader transition
    f2l_1 = status_current==0;         % Agents who were followers at the previous time step
    f2l_2 = rand(N,1)<p_f2l;           % Stochastic process following geometric distribution
    f2l_3 = n_status>=floor(t_ref/dt); % Time as follower exceeds refractory time
    f2l   = f2l_1 & f2l_2 & f2l_3;     % Agents transitioning from followers to leaders

    %% Leader-to-follower transition
    l2f_1 = status_current==1;     % Agents who were leaders at the previous time step
    l2f_2 = false(N,1);            % Distance from nearest neighbor exceeds persistence distance
    for i = 1:N
        d_neighbors = sqrt(sum((X_current-X_current(i,:)).^2,2));
        d_neighbors(i) = Inf;
        if min(d_neighbors)>d_per
            l2f_2(i) = true;
        end
    end
    l2f_3 = n_status>=t_per/dt;    % Time as leader exceeds persistence time
    l2f   = l2f_1 & (l2f_2|l2f_3); % Agents transitioning from leaders to followers

    %% Interaction with obstacles
    if obstacles.status == true
        for j = 1:size(obstacles.centers,1)
            center = obstacles.centers(j,:);
            radius = obstacles.radii(j);

            dX = X_current-center;
            if model_dimension == 2
                [dX_t,dX_r] = cart2pol(dX(:,1),dX(:,2));
                collision_condition = dX_r<radius;
                dX_r(collision_condition,:) = radius*ones(size(dX_r(collision_condition)));
                
                [dX_1,dX_2] = pol2cart(dX_t,dX_r);
                 X_current = [dX_1 dX_2] + center;

            elseif model_dimension == 3
                [dX_t,dX_r,dX_z] = cart2pol(dX(:,1),dX(:,2),dX(:,3));
                collision_condition = dX_r<radius;
                dX_r(collision_condition,:) = radius*ones(size(dX_r(collision_condition)));

                [dX_1,dX_2,dX_3] = pol2cart(dX_t,dX_r,dX_z);
                X_current = [dX_1 dX_2 dX_3] + center;
            end
        end
    end

    %% Update variables
    V_current = V_current+velocity(n_timestep,t_del,dt,X,V,N,M,C_rep,ep,C_ali,C_att,status_current)*dt; % Update velocity
    X_current = X_current+V_current*dt;                                                                 % Update position

    % Update leader/follower status
    status_current(f2l) = 1; % Followers become leaders
    status_current(l2f) = 0; % Leaders become followers
    
    % Update probability of follower-to-leader transition
    p_f2l(n_status>=t_ref/dt) = p_f2l(n_status>=t_ref/dt)*(1-p_f2l_0); % Update probability according to geometric distribution
    p_f2l(f2l)                 = 0;                                    % Set new leaders' probability of follower-to-leader transition to 0
    p_f2l(l2f)                 = p_f2l_0;                              % Set new follower's probability of follower-to-leader transition to the initial probability

    % Update number of time steps since last leader/follower status change
    n_status(f2l|l2f) = 0;            % Reset time since last leader/follower status change
    n_status          = n_status + 1; % Increment time since last leader/follower status change

    %% Record position, velocity & leader/follower status
    X(:,:,n_timestep)    = X_current;      % Record position
    V(:,:,n_timestep)    = V_current;      % Record velocity
    status(:,n_timestep) = status_current; % Record leader/follower status
end

toc

close all

plot_angle(model_dimension, V, timesteps);                            % Plot flock angle
plot_velocity(model_dimension, V, timesteps);                         % Plot flock velocity
plot_center(model_dimension,X, timesteps);                            % Plot flock center
animate_agents(model_dimension,X,V,status,obstacles,dt,int2str(seed)) % Animate agents
