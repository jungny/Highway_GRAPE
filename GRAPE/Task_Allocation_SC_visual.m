function [output] = Task_Allocation_SC_visual(input)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GRAPE Task Allocation_newation Solver Module
% By Inmo Jang, 2.Apr.2016
% Modified, 15.Jul.2016
% Modified, 25.Oct.2017
% Modified for Asynchronous communication environment
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The following describes the name of variables; meanings;  and their matrix sizes
% Input :
%   - n;    the number of agents
%   - m;    the number of tasks
%   - environment.t_location;   Task Position(x,y);             m by 2 matrix (m = #tasks)
%   - environment.t_demand;     Task demand or reward;          m by 1 matrix
%   - environment.a_location;   Agent Posision(x,y);             n by 2 matrix (n = #agents)
%   - Alloc_existing;    Current allocation status of agents;        n by 1 matrix
%   - Flag_display; Flag for display the process;   1 by 1 matrix
% Output :
%   - Alloc;        New allocation status of agents;   n by 1 matrix
%   - a_utility;    Resulted individual utility for each agent; n by 1 matrix
%   - iteration;    Resulted number of iteration for convergence;   1 by 1   matrix

%% Debug flag
debug_log = false;  % Set to true to enable detailed logging

%% Interface (Input)
Alloc_existing = input.Alloc_existing;
Flag_display = input.Flag_display;
n = input.n;
m = input.m;
MST = input.MST;
MST_bubble = input.MST_bubble;
environment = input.environment;
Type = environment.Type;
List = environment.List;

%% For visualisation
Alloc_history = zeros(n,10);
Satisfied_history = zeros(n,10);
MAX_CASE = 10000;
%% Debug history variables
if debug_log
    MAX_CASE = 500;
    Best_task_history = zeros(n, MAX_CASE);
    % Each row represents an agent, columns are organized as:
    % [t_demand_1, n_participants_1, util_value_1, t_demand_2, n_participants_2, util_value_2, ...]
    util_history = zeros(n, MAX_CASE * m * 3);
    valid_agent_history = zeros(n, MAX_CASE);
end
iteration_history = zeros(1, MAX_CASE);
Case = 1;  % Start from 1 instead of 0



%% Initialisation

a_satisfied = 0; % # Agents who satisfy the current partition

agent(n) = struct('iteration', 0, 'time_stamp', 0, 'Alloc', [], 'satisfied_flag', 0, 'util', 0);
agentsetting(n) = struct('ID', 0);

for i=1:n
    v_id = List.Vehicle.Active(i, 1); 
    agentsetting(i).ID = v_id; % 차량 ID 저장
    agent(i).iteration = 0;
    agent(i).time_stamp = rand;
    agent(i).Alloc = Alloc_existing;
    agent(i).satisfied_flag = 0;
    agent(i).util = 0;
end

GreedyAlloc = input.Alloc_existing;

%% Neighbour agents identification (Assumming a static situation)
if startsWith(Type, 'BubbleAhead')
    Type = 'BubbleAhead';
elseif startsWith(Type, 'Bubble')
    Type = 'Bubble';
end

agent_info(n) = struct('set_neighbour_agent_id', []);

for i = 1:n
    switch Type
        case {'Default', 'Ahead'}
            agent_info(i).set_neighbour_agent_id = find(MST(i, :) > 0);

        case {'Bubble', 'BubbleAhead'}
            if environment.Setting.BubbleRadius > 200
                agent_info(i).set_neighbour_agent_id = find(MST(i, :) > 0);
            else
                agent_info(i).set_neighbour_agent_id = find(MST_bubble(i, :) > 0);
            end
    end
    % 이웃의 vehicle_id 리스트 저장
    neighbour_index_list = agent_info(i).set_neighbour_agent_id;
    agentsetting(i).neighbour_v_id = List.Vehicle.Active(neighbour_index_list, 1);  % 차량 ID 리스트
end

Iteration_agent_current = zeros(n,1);
Timestamp_agent_current = zeros(n,1);

%% GRAPE Algorithm
while a_satisfied~=n

    % Conditionally execute environment update
    if isnan(environment.Setting.tFixParam) || Case <= n * environment.Setting.tFixParam
        environment = GRAPE_Environment_Update(List,environment.Parameter,environment.Setting,environment);
        List = environment.List;
    elseif ~isnan(environment.Setting.tFixParam)
        fprintf("Case has reached %d = tFixParam %.0f x %d\n", n * environment.Setting.tFixParam, environment.Setting.tFixParam, n);
    end

    for i=1:n % For Each Agent 
        
        % if Case == 496 && i == 49
        %     disp('debug point');
        % end
        %%%%% Line 5 of Algorithm 1
        Alloc_ = agent(i).Alloc;
        current_task = Alloc_(i); % Currently-selected task
        % disp(Alloc_);
        Candidate = ones(m,1)*(-inf);
        for t=1:m
            if isfield(environment.Setting, 'k_Mode') && strcmp(environment.Setting.k_Mode, 'GapBased_test')
                n_participants = 1;
            else
                switch Type
                    case 'Default'
                        % Check member agent ID in the selected task
                        current_members = (Alloc_ == ones(n,1)*t);
                        current_members(i) = 1; % including oneself
                        % Cardinality of the coalition
                        n_participants = sum(current_members);
                    case 'Bubble'
                        % Check member agent ID in the selected task
                        current_members = (Alloc_ == ones(n,1)*t);
                        % Only consider agents who are neighbours of agent i
                        current_members = current_members & MST_bubble(:,i);
                        current_members(i) = 1; % including oneself
                        % Cardinality of the coalition
                        n_participants = sum(current_members);

                    case 'BubbleAhead'
                        % Check member agent ID in the selected task
                        current_members = (Alloc_ == ones(n,1)*t);

                        x_relation = environment.x_relation;

                        % Only consider agents who are neighbours of agent i
                        current_members = current_members & MST_bubble(:,i);
                        current_members(i) = 1; % including oneself

                        n_participants = sum(x_relation(i, current_members)) + 1; 

                    case 'Ahead'
                        % 현재 agent i가 선택한 task(차선)의 앞에 있는 차량 수, including
                        % oneself
                        x_relation = environment.x_relation;
                        task_agents_mask = (Alloc_ == t);  % 논리형 인덱싱
                        n_participants = sum(x_relation(i, task_agents_mask)) + 1;
                        % fprintf('i = %d, t = %d, n_participants = %d\n', i, t, n_participants);
                end
            end

            % Obtain possible individual utility value
            Candidate(t) = Get_Util(i, t, n_participants,environment);

        end
        
        % Select Best alternative
        [Best_utility,Best_task] = max(Candidate);
        %%%%% End of Line 5 of Algorithm 1
        
        % Log Best task and util information
        if debug_log
            Best_task_history(i, Case) = Best_task;
            
            for t = 1:m
                % Calculate n_participants for this specific task
                if isfield(environment.Setting, 'k_Mode') && strcmp(environment.Setting.k_Mode, 'GapBased_test')
                    n_participants = 1;
                else
                    switch Type
                        case 'Default'
                            current_members = (Alloc_ == ones(n,1)*t);
                            current_members(i) = 1;
                            n_participants = sum(current_members);
                        case 'Bubble'
                            current_members = (Alloc_ == ones(n,1)*t);
                            current_members = current_members & MST_bubble(:,i);
                            current_members(i) = 1;
                            n_participants = sum(current_members);
                        case 'BubbleAhead'
                            current_members = (Alloc_ == ones(n,1)*t);
                            current_members = current_members & MST_bubble(:,i);
                            current_members(i) = 1;
                            n_participants = sum(x_relation(i, current_members)) + 1;
                        case 'Ahead'
                            x_relation = environment.x_relation;
                            task_agents_mask = (Alloc_ == t);
                            n_participants = sum(x_relation(i, task_agents_mask)) + 1;
                    end
                end
                
                % Store information for this task
                col_offset = (Case-1) * m * 3 + (t-1) * 3;
                util_history(i, col_offset + 1) = environment.t_demand(t,i);  % t_demand
                util_history(i, col_offset + 2) = n_participants;           % n_participants
                util_history(i, col_offset + 3) = Candidate(t);             % util_value
            end
        end
        
        %%%%% Line 6-11 of Algorithm 1
        if Best_utility == 0
            Alloc_(i,1) = 0; % Go th the void
            if environment.Setting.GRAPEmode ~= 0 % 1(Greedy) or 2(CycleGreedy) 
                GreedyAlloc(i,1) = 0;
            end
        else
            Alloc_(i,1) = Best_task;
            if environment.Setting.GRAPEmode ~= 0 % 1(Greedy) or 2(CycleGreedy) 
                GreedyAlloc(i,1) = Best_task;
            end
        end
        agent(i).util = Best_utility;
        %
        if current_task == Alloc_(i,1) % if this choice is the same as remaining
            agent(i).satisfied_flag = 1;            
        else
            agent(i).satisfied_flag = 1;  
            agent(i).Alloc = Alloc_;
            agent(i).time_stamp = rand;
            agent(i).iteration = agent(i).iteration + 1;           
        end
        %%%%% End of Line 6-11 of Algorithm 1
        
        % For speed up when executing Algorithm 2
        Iteration_agent_current(i) = agent(i).iteration;
        Timestamp_agent_current(i) = agent(i).time_stamp;        
    end

    if environment.Setting.GRAPEmode ~= 0 % 1(Greedy) or 2(CycleGreedy) 
        break
    end
    
    %% Distributed Mutex (Algorithm 2)  

    agent_ = struct('satisfied_flag', {}, 'Alloc', {}, 'time_stamp', {}, 'iteration', {}, 'util', {});
    agent_(n) = struct('satisfied_flag', 0, 'Alloc', [], 'time_stamp', 0, 'iteration', 0, 'util', 0);

    for i=1:n
        set_neighbour_agent_id = agent_info(i).set_neighbour_agent_id;
        % Initially
        agent_(i).satisfied_flag = 1;
        agent_(i).Alloc = agent(i).Alloc;
        agent_(i).time_stamp = agent(i).time_stamp;
        agent_(i).iteration = agent(i).iteration;
        agent_(i).util = agent(i).util;
        
        % for j_=1:length(set_neighbour_agent_id)
        %     j = set_neighbour_agent_id(j_); % neighbour agent id
        %     % Send information from i to j
        %     if agent(i).iteration < agent(j).iteration % i's info is more recent
        %         % Update using i's info                
        %         agent_(i).Alloc = agent(j).Alloc;
        %         agent_(i).time_stamp = agent(j).time_stamp;
        %         agent_(i).iteration = agent(j).iteration;
        %         agent_(i).satisfied_flag = 0;
        %     elseif agent(i).iteration == agent(j).iteration % when i = j is the same 
        %         if agent(i).time_stamp < agent(j).time_stamp % if i's info is more eariler stamped
        %         agent_(i).Alloc = agent(j).Alloc;
        %         agent_(i).time_stamp = agent(j).time_stamp;
        %         agent_(i).iteration = agent(j).iteration;
        %         agent_(i).satisfied_flag = 0;                
        %         end
        %     else % j's info is more recent
        %         % Keep the current info
        %     end
        % end

%       (Revision) To find out the local "deciding agent" amongst neighbour agents
        set_neighbour_agent_id_ = [set_neighbour_agent_id i];
        % Iteratation amongst neighbour agent set
        Iteration_agent_neighbour = Iteration_agent_current(set_neighbour_agent_id_);
        % Maximum iteration amongst neighbour agent set
        max_Iteration = max(Iteration_agent_neighbour);
        % Agents who have maximum iteration
        max_Iteration_agent_neighbour = (Iteration_agent_neighbour == max_Iteration);
        
        % Timestamp amongst neighbour agent set
        Timestamp_agent_neighbour = Timestamp_agent_current(set_neighbour_agent_id_);
        % Time stamps amongst neighbour agent who have maximum iteraiton
        Timestamp_agent_maxiteration = Timestamp_agent_neighbour.*max_Iteration_agent_neighbour;
        
        % [max_Timestamp,agent_neighbour_index] = max(Timestamp_agent_maxiteration);
        [~,agent_neighbour_index] = max(Timestamp_agent_maxiteration);
        valid_agent_id = set_neighbour_agent_id_(agent_neighbour_index);  % Find out "deciding agent" 
        
        % Log valid agent information
        if debug_log
            valid_agent_history(i, Case) = valid_agent_id;
        end
        
        % Update local information from the deciding agent's local information
        agent_(i).Alloc = agent(valid_agent_id).Alloc;
        agent_(i).time_stamp = agent(valid_agent_id).time_stamp;
        agent_(i).iteration = agent(valid_agent_id).iteration;
        
        % task demand 계산 시 활용할 수 있도록 vehicle의 property에도 반영
        % i는 List.Vehicle.Active(i, 1)의 i
        vehicle_id = List.Vehicle.Active(i, 1); 
        current_vehicle = List.Vehicle.Object{vehicle_id};
        current_lane = current_vehicle.Lane;

        

        if isempty(current_vehicle.AllocLaneDuringGRAPE)
            if agent_(i).Alloc(i) ~= current_lane
                current_vehicle.AllocLaneDuringGRAPE = agent_(i).Alloc(i);
            end
        else
            if agent_(i).Alloc(i) ~= current_vehicle.AllocLaneDuringGRAPE
                % not needed but if loop for debug
                current_vehicle.AllocLaneDuringGRAPE = agent_(i).Alloc(i);
            end
        end


        if min(agent(i).Alloc == agent_(i).Alloc)==1 % If local information is changed
        else
            agent_(i).satisfied_flag = 0;
        end
    end    
    agent = agent_;
    
    %% Check the current status
    a_satisfied = 0;
    iteration = 1;
    for i=1:n
        if agent(i).satisfied_flag == 1
        a_satisfied = a_satisfied + 1;
        end
        % Check the maximum iteration
        iteration = max(agent(i).iteration,iteration);
    end
    
    %%
    
    if Flag_display == 1 
        if mod(iteration,10) == 0
        disp(['Iteration = ',num2str(iteration)])
        end
    end
    
    
    %% Save data for visualisation
    Alloc_known_ = zeros(n,1);
    Satisfied = zeros(n,1);
    for i=1:n
        Alloc_known_(i) = agent(i).Alloc(i);
        Satisfied(i) = agent(i).satisfied_flag;
    end
    Alloc_history(:,Case) = Alloc_known_;
    Satisfied_history(:,Case) = Satisfied;
    iteration_history(Case) = iteration;

    if debug_log && a_satisfied == n   %Case >= 500 && debug_log
        % 숫자 헤더 (9칸씩 같은 "CASE n")
        header1 = repelem("CASE " + string(1:Case), 9);  % 자르기 X
        
        % 문자열 헤더 (3칸씩 같은 'L1_demand', 'L1_participants', 'L1_utility', ...)
        lane_headers = ["L1_demand", "L1_participants", "L1_utility", ...
                       "L2_demand", "L2_participants", "L2_utility", ...
                       "L3_demand", "L3_participants", "L3_utility"];
        header2 = repmat(lane_headers, 1, Case);  % 9개씩 반복
        
        % 합치기 (2행 x 4500열짜리 cell)
        header_cell = [cellstr(header1); cellstr(header2)];
        
        % 데이터도 cell로 변환
        %data_cell = num2cell(util_history);  % 예: 82x4500
        data_cell = num2cell(util_history(:, 1:Case * m * 3));
        % 전체 저장용 cell 만들기
        csv_data = [header_cell; data_cell];  % 84x4500
        
        % Vehicle 정보 저장
        vehicle_info = zeros(n, 3);  % [vehicle_id, location, exit_readiness]
        for i = 1:n
            vehicle_id = List.Vehicle.Active(i, 1);
            vehicle_info(i, 1) = vehicle_id;  % vehicle_id
            vehicle_info(i, 2) = List.Vehicle.Active(i, 4)/100;  % location
            % ExitReadiness를 숫자로 변환 (Ex=1, Th=0)
            vehicle_info(i, 3) = strcmp(List.Vehicle.Object{vehicle_id}.ExitReadiness, 'Ex');
        end
        vehicle_table = array2table(vehicle_info, 'VariableNames', {'vehicle_id', 'location', 'exit_readiness'});
        
        % Excel 파일에 여러 시트로 저장
        a = environment.Setting.ParmeterCombiID;
        b = environment.Setting.RandomSeed;
        
        % Create folder path based on tFixParam
        base_folder = 'C:\\Users\\nana\\Desktop\\ExcelRecord';
        
        if isnan(environment.Setting.tFixParam)
            sub_folder = 'NoFix';
        else
            tFixValue = environment.Setting.tFixParam;
            if tFixValue == round(tFixValue)
                % Integer values
                sub_folder = sprintf('tFix%d', round(tFixValue));
            else
                % Float values - use decimal format and replace dots with underscores
                sub_folder = sprintf('tFix%.2f', tFixValue);
                % Remove trailing zeros but keep the decimal point if needed
                sub_folder = regexprep(sub_folder, '0+$', '');
                sub_folder = regexprep(sub_folder, '\.$', '');
                % Replace dots with underscores for folder name safety
                sub_folder = strrep(sub_folder, '.', '_');
            end
        end
        
        % Create full folder path
        full_folder = fullfile(base_folder, sub_folder);
        
        % Create folder if it doesn't exist
        if ~exist(full_folder, 'dir')
            mkdir(full_folder);
        end
        
        % Generate filename
        if freeze_flag
            filename = fullfile(full_folder, sprintf('ID%d_%dthRS_freeze_history.xlsx', a, b));
        else
            filename = fullfile(full_folder, sprintf('ID%d_%dthRS_no_freeze_history.xlsx', a, b));
        end
        
        writecell(csv_data, filename, 'Sheet', 'util_history');
        writematrix(valid_agent_history, filename, 'Sheet', 'valid_agent_history');
        writematrix(Best_task_history, filename, 'Sheet', 'Best_task_history');
        writetable(vehicle_table, filename, 'Sheet', 'vehicle_info');
        
        disp('check 500 case');
    end

    if Case > 5000
        disp('Case is: ' + string(Case));
    end
    Case = Case + 1;  % Increment Case after recording
end

if environment.Setting.GRAPEmode ~= 0 % 1(Greedy) or 2(CycleGreedy) 
    output.Alloc = GreedyAlloc;
else

    %% Last Check: If Alloc is consensused?
    a_utility = zeros(n,1);
    output.flag_problem = 0;

    if n==1
        Alloc_known_ = agent(1).Alloc;
        iteration = agent(1).iteration;
        % time_stamp = agent(1).time_stamp;
        a_utility(1) = agent(1).util;
    else
        for i=1:n
            if i==1
            Alloc_1 = agent(i).Alloc;
            iteration_1 = agent(i).iteration;
            time_stamp_1 = agent(i).time_stamp;
            else
                Alloc = agent(i).Alloc;        
                iteration = agent(i).iteration;
                time_stamp = agent(i).time_stamp;
                
                if (sum(Alloc_1 == Alloc) == n)&&(iteration_1 == iteration)&&(time_stamp_1 == time_stamp)
                    % Consensus OK
                else
                    %disp(['Problem: Non Consensus with Agent#1 and Agent#',num2str(i)]);
                    output.flag_problem = 1;
                end        
            end
            a_utility(i) = agent(i).util;
        end
    end


    %% Interface (Output)
    % disp(Alloc);
    output.Alloc = Alloc_known_; % Alloc_known_
    output.a_utility = a_utility;
    output.iteration = iteration;
    output.Case = Case;

    output.visual.Alloc_history = Alloc_history;
    output.visual.Satisfied_history = Satisfied_history;
    output.visual.iteration_history = iteration_history;

    if debug_log
        output.visual.Best_task_history = Best_task_history;
        output.visual.util_history = util_history;
        output.visual.valid_agent_history = valid_agent_history;
    end

end
end
