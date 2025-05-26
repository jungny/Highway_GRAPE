%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Given situation, this function is used to obtain an agent's utility 
% [Input]
% - agent_id
% - task_id
% - participants (number_of_members)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [util_value] = Get_Util(agent_id, task_id, n_participants, environment)

a_location = environment.a_location;
t_location = environment.t_location;
t_demand = environment.t_demand(:,agent_id);
%global a_location t_location t_demand table_rand_util

%% Setting Utility Type
% Util_type = 'Peaked_reward';
%Util_type = 'Logarithm_reward';
%Util_type = 'Constant_reward';
%Util_type = 'Random';
Util_type = environment.Util_type;
if strcmp(environment.Util_type, 'Hybrid')
    Util_type = 'Test';
end

if strcmp(environment.Util_type, 'Hybrid')
    Util_type = 'Max_velocity';
end

%%
switch Util_type
    % case {'Min_travel_time'}
    %     util_value = t_demand(task_id)/n_participants;

    case {'GS', 'HOS', 'FOS', 'ES'}
        % n_participants = 1;
        cost = abs(t_location(task_id, 2)- a_location(agent_id, 2));
        % cost는 environment.Parameter.Map.Tile의 정수배
        % 한 번의 차선 변경으로 닿을 수 없는 차선 <=> cost > 3.05
        if (cost - environment.Parameter.Map.Tile) > 1e-6
            util_value = (t_demand(task_id)/n_participants)*0.0001;
        elseif cost > 0
        % if cost > 0
            util_value = (t_demand(task_id)/n_participants)*0.99;
            % if min(t_demand(task_id))>0
            %     util_value = util_value * 0.01;
            % end
        else % cost = 0
            util_value = t_demand(task_id)/n_participants;
        end
    

    case 'Test'
        denominator = 4.4 * (environment.number_of_tasks)^2.2 + 6;
        % Cost 
        cost = abs(t_location(task_id, 2)- a_location(agent_id, 2)) / denominator ; % y coordinate difference
        %util_value = t_demand(task_id)/n_participants;
        util_value = t_demand(task_id)/n_participants - cost;


    case 'Peaked_reward'
        % Cost
        cost = 1*norm(t_location(task_id,:)-a_location(agent_id,:));
        
        % Relative Task Demand
        Desired_num_agent_adjust_factor = 1;
        n = size(a_location,1);
        t_desired_num_agent = round(t_demand(task_id,1)./sum(t_demand(:,1))*n*Desired_num_agent_adjust_factor);
        if t_desired_num_agent <= 1
            t_desired_num_agent = 1;
        end
        
%         util_value = t_demand(task_id)*exp(-(n_participants-1)/t_desired_num_agent) - cost;
        util_value = t_demand(task_id)*exp(-n_participants/t_desired_num_agent + (1-log(t_desired_num_agent))) - cost;
       
        
    case 'Logarithm_reward'
%         cost = 2*norm(t_location(task_id,:)-a_location(agent_id,:));
%         util_value = t_demand(task_id)*log2(n_participants+1)/n_participants - cost;
        n = size(a_location,1);
        m = size(t_location,1);
        cost = norm(t_location(task_id,:)-a_location(agent_id,:));
        util_value = t_demand(task_id)/(log2(n/m+1))*log2(n_participants+1)/n_participants - cost;        
        
    case 'Constant_reward'
        cost = norm(t_location(task_id,:)-a_location(agent_id,:));
        util_value = t_demand(task_id)/n_participants - cost;
        
        
    case 'Random'        
        % table_rand_util(#Participants, Task ID, Agent ID)
        util_value = t_demand(task_id)*prod(table_rand_util(1:n_participants,task_id,agent_id));
        
end

if util_value < 0
    util_value = 0;
end
end
