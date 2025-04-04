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
        case {'GS', 'HOS', 'FOS'}
            if strcmp(Util_type, 'GS')
                L1 = 400;
                L2 = 400;
                L3 = 400;
            elseif strcmp(Util_type, 'HOS')
                L1 = 600;
                L2 = 200;
                L3 = 600;
            elseif strcmp(Util_type, 'FOS')
                L1 = 800;
                L2 = 0;
                L3 = 800;
            end

            for i = 1:size(List.Vehicle.Active, 1)
                vehicle_id = List.Vehicle.Active(i, 1);  % 차량 ID
                vehicle_lane = List.Vehicle.Object{vehicle_id}.Lane;

                distance_to_exit = List.Vehicle.Object{vehicle_id}.Exit - ...
                                    List.Vehicle.Object{vehicle_id}.Location * Parameter.Map.Scale;  % Exit까지 거리

                

                if (distance_to_exit <= L1+L2)  && vehicle_lane == 1 % && (distance_to_exit > L2)
                    weights = [0; 1; 0];
                    % if strcmp(strategy, 'FOS')
                    %     weights = [0; 1; 1];
                    % end

                elseif (distance_to_exit <= L3) && (distance_to_exit > 0)
                    weights = [0; 0; 1];
                    % if vehicle_lane == 1 && (distance_to_exit < L2)
                    %     weights = [0; 0; 1];
                    % elseif vehicle_lane == 2
                    %     weights = [0; 0; 1];
                    % elseif vehicle_lane == 3
                    %     weights = [0; 0; 1];
                    % end

                else
                
                    % obj = List.Vehicle.Object{vehicle_id};
                    % currentLane = obj.Lane;
                    % decelflag = false;
                    % leftflag = false;
                    % rightflag = false;
                
                    % % (4) 내가 감속 중이면 decelflag
                    % if obj.Acceleration < -1 && obj.Velocity < 25
                    %     decelflag = true;
                    % end
                
                    % % 현재 차선의 선행 차량 거리
                    % [cur_front_vehicle, cur_front_dist] = GetFrontVehicle(obj, currentLane, List, Parameter);
                    % if isempty(cur_front_vehicle)
                    %     cur_front_dist = inf;
                    % end
                
                    % % 왼쪽 차선 조건
                    % if currentLane > 1
                    %     leftLane = currentLane - 1;
                    %     [left_front_vehicle, left_front_dist] = GetFrontVehicle(obj, leftLane, List, Parameter);
                    %     if isempty(left_front_vehicle)
                    %         left_front_dist = inf;
                    %     end
                
                    %     if left_front_dist > cur_front_dist  % (5)
                    %         leftflag = true;
                    %     end
                    % end
                
                    % % 오른쪽 차선 조건
                    % if currentLane < Parameter.Map.Lane
                    %     rightLane = currentLane + 1;
                    %     [right_front_vehicle, right_front_dist] = GetFrontVehicle(obj, rightLane, List, Parameter);
                    %     if isempty(right_front_vehicle)
                    %         right_front_dist = inf;
                    %     end
                
                    %     if right_front_dist > cur_front_dist  % (5)
                    %         rightflag = true;
                    %     end
                    % end
                
                    % (4)+(5): 감속 중이고, 양옆 차선 선행차가 더 멀면 → 해당 차선 weight 크게
                    weights = ones(Parameter.Map.Lane, 1);
                    % if decelflag && leftflag
                    %     weights(currentLane - 1) = 2;
                    % elseif decelflag && rightflag
                    %     weights(currentLane + 1) = 2;
                    % else
                    %     weights(currentLane) = 2;  % 조건 안 맞으면 원래 차선 유지
                    % end
                end
                

                for lane = 1:Parameter.Map.Lane
                    normalized_weights(lane) = floor(weights(lane)*100)/100;
                    t_demand(lane, i) = normalized_weights(lane); 
                end

            end



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
                        raw_weights(lane) = exp(double(-(transition_distance - distance_to_exit) / (k * lane)));
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
        
        case 'ES' % exponential strategy
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
                        raw_weights(lane) = exp(double(-(transition_distance - distance_to_exit) / (k * lane)));
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
                        raw_weights(lane) = exp(double(-transition_distance - distance_to_exit) / (k * lane));
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

        case 'Hybrid'
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

                else
                    for lane = 1:Parameter.Map.Lane
                        raw_weights(lane) = 1 / Parameter.Map.Lane;  % 균일 분포
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
    environment.LogFile = Setting.LogPath;
    environment.test_iteration = testiteration;
    environment.x_relation = GetXRelation(List);
    environment.number_of_tasks = Parameter.Map.Lane;
    environment.Util_type = Setting.Util_type;
    environment.Setting = Setting;
    environment.Parameter = Parameter;


end

%%
%%
% Function for 'Max_velocity' utility
function feasible = PoliteLaneChangeFeasibility(safeDistanceParameter, obj, targetLane, List, Parameter)
    % 현재 차량 위치와 목표 차선의 선행/후행 차량 간 거리 계산
    feasible = false;  % 기본값: 변경 불가
    current_x = double(obj.Location * Parameter.Map.Scale);  % 현재 차량의 x 좌표
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

function [front_vehicle, front_distance] = GetFrontVehicle(obj, targetLane, List, Parameter)
    % 현재 차선의 선행 차량 찾기
    current_x = double(obj.Location * Parameter.Map.Scale);

    % 목표 차선의 차량 필터링
    lane_vehicles = List.Vehicle.Active(List.Vehicle.Active(:,3) == targetLane, :);

    % 모든 차량의 거리 계산
    distances = lane_vehicles(:,4) * Parameter.Map.Scale - current_x;

    % 선행 차량 거리 필터링
    front_distances = distances(distances > 0);

    % 초기화
    front_vehicle = [];
    front_distance = inf;

    if ~isempty(front_distances)
        % 가장 가까운 선행 차량 거리와 인덱스 찾기
        [front_distance, ~] = min(front_distances);

        % front_distances 값이 distances에서의 원래 인덱스 찾기
        tolerance = 1e-6; % 부동소수점 오차 허용
        original_idx = find(abs(distances - front_distance) < tolerance, 1, 'first');

        % 해당 인덱스의 차량 정보 추출
        front_vehicle = lane_vehicles(original_idx, :);
    end
end