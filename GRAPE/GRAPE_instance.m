% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % GRAPE Algorithm for Multi-Robot Task Allocation
% % By Inmo Jang, 06.Oct.2015
% % Modified, 12.Jan.2015
% % Modified, 15.Jul.2015
% % Modified, 23.Jun.2017
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% Initialisation (1) - Generation of Random Scenario
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Tasks/Agents Location
% t_location_limit = [500 500]; % Task locations range: max(X) max(Y) (metre)
% a_location_limit = [300 300]; % Agent locations range: max(X) & max(Y) (metre)

% % Task Demand or Reward
% t_demand_mean = 1000*n/m; % 15.(Jul.2016) In order to maintain the level of individual utilities regardless of #Tasks or #Agents

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% t_location = zeros(m,2);    % task location - 2D
% a_location = zeros(n,2);    % agent location - 2D
% t_demand = zeros(m,1);      % task rewards

% Comm_distance = 50;         % Communication range of each robot
% Gap_agent = 15;             % Minimum spatial distance between any two robots
% Gap_task = 200;             % Minimum spatial distance between any two tasks

% % Generation of Task information
% % t_location(t,1) : task의 x좌표 
% % t_location(t,2) : task의 y좌표

% % for t=1:m
% %     t_demand(t) = ...
% % end

% % for t=1:m
% %     ok = 0;
% %     while ok == 0
% %         t_location(t,:) = [random('Uniform',-t_location_limit(1),t_location_limit(1)) ...
% %             random('Uniform',-t_location_limit(2),t_location_limit(2))];
% %         if t_location(t,1) > - a_location_limit(1) - 100 && t_location(t,1) < a_location_limit(1) + 100 ...
% %                 && t_location(t,2) > - a_location_limit(2) -100 && t_location(t,2) < a_location_limit(2) + 100
% %             ok = 0;
% %         else
% %             ok = 1;            
% %             for k=1:t-1
% %                 if norm(t_location(k,:) - t_location(t,:)) < Gap_task
% %                     ok = 0;
% %                 end
% %             end            
% %         end
% %     end
% %     t_demand(t) = abs(random('Uniform',t_demand_mean*1,t_demand_mean*2));
% % end

% % Generation of Agent information
% % a_location(i, 1): i-번째 에이전트의 x좌표
% % a_location(i, 2): i-번째 에이전트의 y좌표

% % for i=1:n
% %     ok = 0;
% %     while ok == 0
% %         a_location(i,:) = [random('Uniform',-a_location_limit(1), a_location_limit(1)) random('Uniform',-a_location_limit(2), a_location_limit(2))];
% %         switch Deployment
% %             % Randomely distribute agents as a circle
% %             case 1
% %                 if norm(a_location(i,:)) < a_location_limit(1) % Case (1) Circle
% %                     ok = 1;
% %                     for k=1:i-1
% %                         if norm(a_location(k,:) - a_location(i,:)) < Gap_agent
% %                             ok = 0;
% %                         end
% %                     end
                    
% %                 end                
% %             % Randomly distribute agents as a skewed circle    
% %             case 2
% %                 if abs(sum((a_location(i,:)))) < a_location_limit(1) % Case (2) Skewed Circle
% %                     ok = 1;
% %                     for k=1:i-1
% %                         if norm(a_location(k,:) - a_location(i,:)) < Gap_agent
% %                             ok = 0;
% %                         end
% %                     end
                    
% %                 end
% %             % Randomly distribute agents as a square
% %             case 3                
% %                 ok = 1;
% %                 for k=1:i-1
% %                     if norm(a_location(k,:) - a_location(i,:)) < Gap_agent
% %                         ok = 0;
% %                     end
% %                 end
% %         end
        
% %     end
% % end

% % Generation Agent Communication (Neighbour agents within communication
% % radius)
% dist_agents = zeros(n,n);
% for i=1:n
%     for j=1:n
%         dist_agents(i,j) = norm(a_location(i,:)-a_location(j,:));
%     end
% end

% % Neighbour agents within communication radius
% MST_ = (dist_agents <= Comm_distance);
% MST = MST_ - eye(n,n);
% % Note: MST will be used in Task_Allocation.m (Task_Allocation_SC_visual.m) to simulate communications between agents



% environment.t_location = t_location;
% environment.t_demand = t_demand;
% environment.a_location = a_location;


% %% Initialise task allocation & Merge and Split Algorithm
% Alloc_existing = zeros(n,1);    % Initial task assignment: every robot is assigned to void task


% input.Alloc_existing = Alloc_existing;
% input.Flag_display = Flag_display;
% input.MST = MST;
% input.n = n;
% input.m = m;
% input.environment = environment;

% %%%% Method (1): All Agents are deployed at once
% [output] = Task_Allocation_SC_visual(input); % Consiering Strongly-connected environment
% % Output : Alloc / a_utility / iteration

% Alloc = output.Alloc;
% a_utility = output.a_utility;
% iteration = output.iteration;
% flag_problem = output.flag_problem; % If the result has a problem, then 1. 




function G = GRAPE_instance(environment)
    G.t_demand = environment.t_demand;
    G.t_location = environment.t_location;
    G.a_location = environment.a_location;

    n = size(G.a_location,1);
    m = size(G.t_location,1);

    dist_agents = zeros(n,n);
    for i=1:n
        for j=1:n
            %dist_agents(i,j) = norm(G.a_location(i,:)-G.a_location(j,:));
            dist_agents(i,j) = abs(G.a_location(i,1)-G.a_location(j,1));
            %disp(dist_agents(i,j));
        end
    end

    Comm_distance = 5000000; % 250m 
    % Neighbour agents within communication radius
    MST_ = (dist_agents <= Comm_distance);
    MST = MST_ - eye(n,n);
    % Note: MST will be used in Task_Allocation.m (Task_Allocation_SC_visual.m) to simulate communications between agents

    % Alloc_existing = zeros(n,1);    % Initial task assignment: every robot is assigned to void task
    Alloc_existing = environment.Alloc_current;

    input.Alloc_existing = Alloc_existing;
    input.Flag_display = 1;
    input.MST = MST;
    input.n = n;
    input.m = m;
    input.environment = environment;

    %%%% Method (1): All Agents are deployed at once
    [output] = Task_Allocation_SC_visual(input); % Consiering Strongly-connected environment
    % Output : Alloc / a_utility / iteration

    G.Alloc = output.Alloc;
    G.a_utility = output.a_utility;
    G.iteration = output.iteration;
    G.flag_problem = output.flag_problem; % If the result has a problem, then 1. 

    Alloc = output.Alloc;
    a_utility = output.a_utility;
    iteration = output.iteration;
    flag_problem = output.flag_problem;

    %% Minimum-guaranteed Global Utility (Theorem 3)
    if n>1
        Minimum_Guaranteed_Optimality;
    end

end

