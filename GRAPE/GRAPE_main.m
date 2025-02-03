function environment = GRAPE_main(List, Parameter,Setting,testiteration)
    % a_location 생성
    a_location = zeros(size(List.Vehicle.Active, 1), 2);
    for i = 1:size(List.Vehicle.Active, 1)
        vehicle_id = List.Vehicle.Active(i, 1);  % 현재 차량 ID
        a_location(i, :) = [List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location, ...
                            (Parameter.Map.Lane-List.Vehicle.Object{List.Vehicle.Active(i,1)}.Lane+0.5) * Parameter.Map.Tile];  % 차량의 현재 (x, y) 위치
    end

    % t_location, t_demand 생성
    t_location = zeros(Parameter.Map.Lane, 2);
    for i = 1:Parameter.Map.Lane
        t_location(i, :) = [0, (Parameter.Map.Lane-i+0.5) * Parameter.Map.Tile];  % (x, y) 좌표로 정의 (x는 0으로 고정)
    end

    t_demand = zeros(Parameter.Map.Lane, size(List.Vehicle.Active,1));  
    % t_demand(:) = 100*size(List.Vehicle.Active, 1);
    
    % transition_distance = 300 + 15*Parameter.Map.Lane^2;
    raw_weights = zeros(Parameter.Map.Lane,1);

    Util_type = Setting.Util_type;
    switch Util_type
        case 'Test'
            for i = 1:size(List.Vehicle.Active, 1)
                vehicle_id = List.Vehicle.Active(i, 1);  % 차량 ID
                vehicle_lane = List.Vehicle.Object{vehicle_id}.Lane;
                transition_distance = 300 + 15*(Parameter.Map.Lane-vehicle_lane)^2;
                distance_to_exit = List.Vehicle.Object{vehicle_id}.Exit - ...
                                    List.Vehicle.Object{vehicle_id}.Location * Parameter.Map.Scale;  % Exit까지 거리

                delta_d = transition_distance - distance_to_exit;
                % Case 1: Distance greater than transition distance (uniform weights)
                if delta_d<0 % distance_to_exit > transition_distance
                    weights = ones(Parameter.Map.Lane, 1) / Parameter.Map.Lane;  % 균일 분포
                else
                    % Case 2: Distance less than or equal to transition distance
                    for lane = 1:Parameter.Map.Lane
                        k = 2; % k 작을수록 lane별 더 극단적인 차이가 발생 
                        % Weight increases as lane number increases
                        raw_weights(lane) = exp(-(transition_distance - distance_to_exit) / (k * lane));
                    end
                    
                    % Normalize weights to ensure they sum to 1
                    weights = raw_weights / sum(raw_weights);
                end
            
                % t_demand에 반영
                for lane = 1:Parameter.Map.Lane
                    normalized_weights(lane) = floor(weights(lane)*100)/100;
                    t_demand(lane, i) = size(List.Vehicle.Active, 1)*normalized_weights(lane);  % vehicle 수 곱해 비율 유지
                end
            end
        
        case 'Min_travel_time'
            for i = 1:size(List.Vehicle.Active, 1)
                vehicle_id = List.Vehicle.Active(i, 1);  % 차량 ID
                vehicle_lane = List.Vehicle.Object{vehicle_id}.Lane;
                transition_distance = 300 + 15*(Parameter.Map.Lane-vehicle_lane)^2;

                exit_list = Parameter.Map.Exit;
                vehicle_exit = List.Vehicle.Object{vehicle_id}.Exit;
                vehicle_x_position = List.Vehicle.Object{vehicle_id}.Location * Parameter.Map.Scale;
                distance_to_exit = vehicle_exit - vehicle_x_position;

                exit_idx = find(exit_list == vehicle_exit, 1);
                delta_d = transition_distance - distance_to_exit;


                if (exit_idx > 1 && delta_d >= 0 && vehicle_x_position > exit_list(exit_idx - 1)) ...
                    || (exit_idx == 1 && delta_d >= 0)
                    % Case 2: Distance less than or equal to transition distance
                    for lane = 1:Parameter.Map.Lane
                        k = 2; % k 작을수록 lane별 더 극단적인 차이가 발생 
                        % Weight increases as lane number increases
                        raw_weights(lane) = exp(-(transition_distance - distance_to_exit) / (k * lane));
                    end
                    % Normalize weights to ensure they sum to 1
                    weights = raw_weights / sum(raw_weights);

                % Case 1 (uniform weights)
                else
                    weights = ones(Parameter.Map.Lane, 1) / Parameter.Map.Lane;  % 균일 분포
                end
            
                % t_demand에 반영
                for lane = 1:Parameter.Map.Lane
                    normalized_weights(lane) = floor(weights(lane)*100)/100;
                    t_demand(lane, i) = size(List.Vehicle.Active, 1)*normalized_weights(lane);  % vehicle 수 곱해 비율 유지
                end
            end
        
        case 'Max_velocity'
            for i = 1:size(List.Vehicle.Active, 1)
                vehicle_id = List.Vehicle.Active(i, 1);  % 차량 ID
                vehicle_lane = List.Vehicle.Object{vehicle_id}.Lane;
                transition_distance = 300 + 15*(Parameter.Map.Lane-vehicle_lane)^2;
                distance_to_exit = List.Vehicle.Object{vehicle_id}.Exit - ...
                                    List.Vehicle.Object{vehicle_id}.Location * Parameter.Map.Scale;  % Exit까지 거리

                delta_d = transition_distance - distance_to_exit;

                % Case 1: Distance greater than transition distance (uniform weights)
                if delta_d<0 % distance_to_exit > transition_distance
                    for lane = 1:Parameter.Map.Lane
                        raw_weights(lane) = 1 / Parameter.Map.Lane;  % 균일 분포
                    end
                else
                    % Case 2: Distance less than or equal to transition distance
                    for lane = 1:Parameter.Map.Lane
                        k = 2; % k 작을수록 lane별 더 극단적인 차이가 발생 
                        % Weight increases as lane number increases
                        raw_weights(lane) = exp(-(transition_distance - distance_to_exit) / (k * lane));
                    end
                end

                %%%%%%%%
                safeDistanceParameter=4; %앞뒤로 4m
                penaltyFactor = 0.1;
                for lane = 1:Parameter.Map.Lane
                    if lane > vehicle_lane
                        adjacentLane = vehicle_lane+1;
                        feasible = PoliteLaneChangeFeasibility(safeDistanceParameter, List.Vehicle.Object{vehicle_id}, ...
                                                                adjacentLane, List, Parameter);
                        if ~feasible
                            raw_weights(lane) = penaltyFactor * raw_weights(lane);
                        end
                    elseif lane < vehicle_lane
                        adjacentLane = vehicle_lane-1;
                        feasible = PoliteLaneChangeFeasibility(safeDistanceParameter, List.Vehicle.Object{vehicle_id}, ...
                                                                adjacentLane, List, Parameter);
                        if ~feasible
                            raw_weights(lane) = penaltyFactor * raw_weights(lane);
                        end
                    else % lane == vehicle_lane
                    end 
                end

                % Normalize weights to ensure they sum to 1
                weights = raw_weights / sum(raw_weights);
            
                % t_demand에 반영
                for lane = 1:Parameter.Map.Lane
                    normalized_weights(lane) = floor(weights(lane)*100)/100;
                    t_demand(lane, i) = size(List.Vehicle.Active, 1)*normalized_weights(lane);  % vehicle 수 곱해 비율 유지
                end
            end
    end


    % t_demand = size(List.Vehicle.Active, 1) * 100 * ones(Parameter.Map.Lane, 1);

    % Alloc_current 생성
    Alloc_current = [];
    for i = 1:size(List.Vehicle.Active, 1)
        Alloc_current = [Alloc_current; List.Vehicle.Object{List.Vehicle.Active(i, 1)}.Lane];
    end


    environment.t_location = t_location;
    environment.a_location = a_location;
    environment.t_demand = t_demand;
    environment.Alloc_current = Alloc_current;
    environment.Type = Setting.NumberOfParticipants;
    environment.LogFile = Setting.LogFile;
    environment.test_iteration = testiteration;
    environment.x_relation = GetXRelation(List);
    environment.number_of_tasks = Parameter.Map.Lane;
    environment.Util_type = Setting.Util_type;


end

%%
%%
% Function for 'Max_velocity' utility
function feasible = PoliteLaneChangeFeasibility(safeDistanceParameter, obj, targetLane, List, Parameter)
    % 현재 차량 위치와 목표 차선의 선행/후행 차량 간 거리 계산
    feasible = false;  % 기본값: 변경 불가
    current_x = obj.Location * Parameter.Map.Scale;  % 현재 차량의 x 좌표
    lane_vehicles = List.Vehicle.Active(List.Vehicle.Active(:,3) == targetLane, :);  % 목표 차선의 차량

    % 목표 차선의 선행/후행 차량 찾기
    distances = lane_vehicles(:,4)*Parameter.Map.Scale - current_x;
    front_distances = distances(distances > 0);
    rear_distances = distances(distances < 0);

    % 안전 거리 조건
    safe_distance = safeDistanceParameter;
    if isempty(front_distances) || min(front_distances) > safe_distance
        if isempty(rear_distances) || abs(max(rear_distances)) > safe_distance
            feasible = true;  % 선행/후행 모두 안전 거리 만족
        end
    end
end