function environment = GRAPE_Environment_Initialize(List, Parameter,Setting)
    % AllocationLaneDuringGRAPE 초기화
    n = size(List.Vehicle.Active, 1);
    for i = 1:n
        vid = List.Vehicle.Active(i, 1); 
        vehicle = List.Vehicle.Object{vid};
        vehicle.AllocLaneDuringGRAPE = [];
    end
    
    % GRAPE_Environment_Update.m 방식처럼 차량 정보 배열화 (최적화)
    vehicle_ids = List.Vehicle.Active(:,1);
    num_vehicles = n; % size(List.Vehicle.Active, 1);
    vehicle_lanes = zeros(num_vehicles,1);
    vehicle_locations = zeros(num_vehicles,1);
    vehicle_targetlanes = nan(num_vehicles,1);
    vehicle_alloclanes = nan(num_vehicles,1);
    for i = 1:num_vehicles
        vid = vehicle_ids(i);
        obj = List.Vehicle.Object{vid};
        vehicle_lanes(i) = obj.Lane;
        vehicle_locations(i) = obj.Location;
        if ~isempty(obj.TargetLane)
            vehicle_targetlanes(i) = obj.TargetLane;
        end
        if ~isempty(obj.AllocLaneDuringGRAPE)
            vehicle_alloclanes(i) = obj.AllocLaneDuringGRAPE;
        end
    end

    % a_location 생성
    a_location = zeros(n, 2);
    for i = 1:n
        vid = List.Vehicle.Active(i, 1);  % 현재 차량 ID
        a_location(i, :) = [List.Vehicle.Object{vid}.Location, ...
                            (Parameter.Map.Lane-List.Vehicle.Object{vid}.Lane+0.5) * Parameter.Map.Tile];  % 차량의 현재 (x, y) 위치
    end

    % t_location, t_demand 생성
    t_location = zeros(Parameter.Map.Lane, 2);
    for i = 1:Parameter.Map.Lane
        t_location(i, :) = [0, (Parameter.Map.Lane-i+0.5) * Parameter.Map.Tile];  % (x, y) 좌표로 정의 (x는 0으로 고정)
    end

    t_demand = zeros(Parameter.Map.Lane, n);  
    % t_demand(:) = 100*size(List.Vehicle.Active, 1);
    
    % transition_distance = 300 + 15*Parameter.Map.Lane^2;

    Util_type = Setting.Util_type;
    switch Util_type
        case {'GS', 'HOS', 'FOS'}
            if strcmp(Util_type, 'GS')
                L1 = 200;
                L2 = 200;
                L3 = 200;
            elseif strcmp(Util_type, 'HOS')
                L1 = 300;
                L2 = 100;
                L3 = 300;
            elseif strcmp(Util_type, 'FOS')
                L1 = 400;
                L2 = 0;
                L3 = 400;
            end

            for i = 1:size(List.Vehicle.Active, 1)
                vehicle_id = List.Vehicle.Active(i, 1);  % 차량 ID
                vehicle_lane = List.Vehicle.Object{vehicle_id}.Lane;

                distance_to_exit = List.Vehicle.Object{vehicle_id}.Exit - ...
                                    List.Vehicle.Object{vehicle_id}.Location * Parameter.Map.Scale;  % Exit까지 거리

                

                if (distance_to_exit <= L1+L2) && vehicle_lane == 3
                    weights = [0; 0; 1];

                elseif (distance_to_exit <= L1+L2)  && vehicle_lane ~= 3 && (distance_to_exit > L2)
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
                
                    obj = List.Vehicle.Object{vehicle_id};
                    currentLane = obj.Lane;
                    % decelflag = false;
                    leftflag = false;
                    rightflag = false;
                    %cur_front_dist = NaN;
                    left_dist = -inf;
                    % right_dist = -inf;

                
                    % (4) 내가 감속 중이면 decelflag
                    % if obj.Acceleration < -1 && obj.Velocity < 25
                    %     decelflag = true;
                    % end
                    % ALDG의 효과가 나타나는지 확인하기 위해 조건 간소화
                    decelflag = true;
                
                    % 현재 차선의 선행 차량 거리 (최적화된 탐색 사용 -> GRAPE_Environment_Update.m 방식으로 변경)
                    % [cur_front_vehicle, cur_front_dist] = GetFrontVehicleOptimized(obj, activeVehiclesByLane{currentLane}, Parameter, Setting);
                    % [cur_rear_vehicle, cur_rear_dist] = GetRearVehicleOptimized(obj, activeVehiclesByLane{currentLane}, Parameter, Setting);
                    
                    [cur_front_vehicle, cur_front_dist] = GetFrontVehicle(obj, currentLane, List, Parameter, Setting, vehicle_ids, vehicle_lanes, vehicle_locations, vehicle_targetlanes, vehicle_alloclanes);
                    [cur_rear_vehicle, cur_rear_dist] = GetRearVehicle(obj, currentLane, List, Parameter, Setting, vehicle_ids, vehicle_lanes, vehicle_locations, vehicle_targetlanes, vehicle_alloclanes);

                    % GetFrontVehicle 및 GetRearVehicle 함수 내에서 빈 경우 200m로 설정하도록 수정 예정 (main 함수에 있는 GetVehicle 함수 참고)
                    if isinf(cur_front_dist)
                        cur_front_dist = 200;
                    end
                    if isinf(cur_rear_dist)
                        cur_rear_dist = 200;
                    end

                    % Ahead 또는 BubbleAhead 타입인 경우 rear_dist 무시
                    if startsWith(Setting.NumberOfParticipants, 'Ahead') || startsWith(Setting.NumberOfParticipants, 'BubbleAhead')
                        cur_dist = cur_front_dist;
                    else
                        cur_dist = cur_front_dist + cur_rear_dist;
                    end
                
                    % 왼쪽 차선 조건 (최적화된 탐색 사용 -> GRAPE_Environment_Update.m 방식으로 변경)
                    % [left_front_vehicle, left_front_dist] = GetFrontVehicleOptimized(obj, activeVehiclesByLane{leftLane}, Parameter, Setting);
                    % [left_rear_vehicle, left_rear_dist] = GetRearVehicleOptimized(obj, activeVehiclesByLane{leftLane}, Parameter, Setting);
                    
                    if currentLane > 1
                        leftLane = currentLane - 1;
                        [left_front_vehicle, left_front_dist] = GetFrontVehicle(obj, leftLane, List, Parameter, Setting, vehicle_ids, vehicle_lanes, vehicle_locations, vehicle_targetlanes, vehicle_alloclanes);
                        [left_rear_vehicle, left_rear_dist] = GetRearVehicle(obj, leftLane, List, Parameter, Setting, vehicle_ids, vehicle_lanes, vehicle_locations, vehicle_targetlanes, vehicle_alloclanes);

                        % GetFrontVehicle 및 GetRearVehicle 함수 내에서 빈 경우 200m로 설정하도록 수정 예정
                        if isinf(left_front_dist)
                            left_front_dist = 200;
                        end
                        if isinf(left_rear_dist)
                            left_rear_dist = 200;
                        end
                        
                        % Ahead 또는 BubbleAhead 타입인 경우 rear_dist 무시
                        if startsWith(Setting.NumberOfParticipants, 'Ahead') || startsWith(Setting.NumberOfParticipants, 'BubbleAhead')
                            left_dist = left_front_dist;
                        else
                            left_dist = left_front_dist + left_rear_dist;
                        end
                
                        if left_dist > cur_dist  % (5)
                            leftflag = true;
                        end
                    end
                
                    % 오른쪽 차선 조건 (최적화된 탐색 사용 -> GRAPE_Environment_Update.m 방식으로 변경)
                    % [right_front_vehicle, right_front_dist] = GetFrontVehicleOptimized(obj, activeVehiclesByLane{rightLane}, Parameter, Setting);
                    % [right_rear_vehicle, right_rear_dist] = GetRearVehicleOptimized(obj, activeVehiclesByLane{rightLane}, Parameter, Setting);
                    
                    if currentLane < Parameter.Map.Lane
                        rightLane = currentLane + 1;
                        [right_front_vehicle, right_front_dist] = GetFrontVehicle(obj, rightLane, List, Parameter, Setting, vehicle_ids, vehicle_lanes, vehicle_locations, vehicle_targetlanes, vehicle_alloclanes);
                        [right_rear_vehicle, right_rear_dist] = GetRearVehicle(obj, rightLane, List, Parameter, Setting, vehicle_ids, vehicle_lanes, vehicle_locations, vehicle_targetlanes, vehicle_alloclanes);
                        
                        % GetFrontVehicle 및 GetRearVehicle 함수 내에서 빈 경우 200m로 설정하도록 수정 예정
                        if isinf(right_front_dist)
                            right_front_dist = 200;
                        end
                        if isinf(right_rear_dist)
                            right_rear_dist = 200;
                        end
                        
                        % Ahead 또는 BubbleAhead 타입인 경우 rear_dist 무시
                        if startsWith(Setting.NumberOfParticipants, 'Ahead') || startsWith(Setting.NumberOfParticipants, 'BubbleAhead')
                            right_dist = right_front_dist;
                        else
                            right_dist = right_front_dist + right_rear_dist;
                        end
                
                        if right_dist > cur_dist  % (5)
                            if right_dist == left_dist
                                rightflag = true; % leftflag = already true
                            elseif right_dist > left_dist
                                rightflag = true;
                                leftflag = false;
                            % else means right_front_dist < left_front_dist -> not need to change any flag
                            end
                        end
                    end
                
                    % 각 차선의 선행차량과의 거리를 한 줄로 출력
                    % fprintf('V%d(L%d) - Cur:%.1f, L:%.1f, R:%.1f\n', vehicle_id, currentLane, cur_front_dist, left_front_dist, right_front_dist);
                
                    % (4)+(5): 감속 중이고, 양옆 차선 선행차가 더 멀면 → 해당 차선 weight 크게
                    weights = ones(Parameter.Map.Lane, 1);

                    if decelflag && leftflag && rightflag
                        weights(currentLane - 1) = Setting.k;
                        weights(currentLane + 1) = Setting.k;
                    elseif decelflag && leftflag
                        weights(currentLane - 1) = Setting.k;
                    elseif decelflag && rightflag
                        weights(currentLane + 1) = Setting.k;
                    else
                        weights(currentLane) = Setting.k;  % 조건 안 맞으면 원래 차선 유지
                    end

                    % Through vehicle인 경우에만 task demand 억제 로직 적용
                    % 현재 차선이 아닌 다른 차선들에 대해 억제 로직 적용
                    all_lanes_congested = true;  % 모든 차선이 혼잡한지 확인하는 플래그
                    for lane = 1:Parameter.Map.Lane
                        if lane ~= currentLane
                            % 이미 계산된 정보 재활용 (Optimized 함수 사용 시 불필요)
                            % if lane == currentLane - 1  % 왼쪽 차선
                            %     front_dist = left_front_dist;
                            %     rear_dist = left_rear_dist;
                            % elseif lane == currentLane + 1  % 오른쪽 차선
                            %     front_dist = right_front_dist;
                            %     rear_dist = right_rear_dist;
                            % end
                            
                            % 해당 차선의 차량 정보를 가져와서 혼잡도 판단 (최적화된 탐색 사용 -> GRAPE_Environment_Update.m 방식으로 변경)
                            if lane == currentLane - 1  % 왼쪽 차선
                                targetLaneCheck = leftLane;
                                % targetVehicles = activeVehiclesByLane{leftLane}; % 이 부분 제거
                            elseif lane == currentLane + 1  % 오른쪽 차선
                                targetLaneCheck = rightLane;
                                % targetVehicles = activeVehiclesByLane{rightLane}; % 이 부분 제거
                            else
                                % 현재 차선은 이미 위에서 계산했으므로 스킵
                                continue;
                            end

                            % [~, front_dist_check] = GetFrontVehicleOptimized(obj, targetVehicles, Parameter, Setting);
                            % [~, rear_dist_check] = GetRearVehicleOptimized(obj, targetVehicles, Parameter, Setting);
                            
                            [~, front_dist_check] = GetFrontVehicle(obj, targetLaneCheck, List, Parameter, Setting, vehicle_ids, vehicle_lanes, vehicle_locations, vehicle_targetlanes, vehicle_alloclanes);
                            [~, rear_dist_check] = GetRearVehicle(obj, targetLaneCheck, List, Parameter, Setting, vehicle_ids, vehicle_lanes, vehicle_locations, vehicle_targetlanes, vehicle_alloclanes);

                            % GetFrontVehicle 및 GetRearVehicle 함수 내에서 빈 경우 200m로 설정하도록 수정 예정
                             if isinf(front_dist_check)
                                front_dist_check = 200;
                            end
                            if isinf(rear_dist_check)
                                rear_dist_check = 200;
                            end

                            % Ahead 또는 BubbleAhead 타입인 경우 rear_dist 무시
                            if startsWith(Setting.NumberOfParticipants, 'Ahead') || startsWith(Setting.NumberOfParticipants, 'BubbleAhead')
                                % 해당 차선이 혼잡한지 확인 (front_dist만 고려)
                                if front_dist_check <= Parameter.TaskDemandCrowdedRange
                                    weights(lane) = 0;  % 혼잡한 차선의 weight을 0으로 설정
                                else
                                    all_lanes_congested = false;  % 하나라도 혼잡하지 않은 차선이 있으면 false
                                end
                            else
                                % 기존 로직: front_dist와 rear_dist 모두 고려
                                if (front_dist_check <= Parameter.TaskDemandCrowdedRange) || ...
                                    (rear_dist_check <= Parameter.TaskDemandCrowdedRange)
                                    weights(lane) = 0;  % 혼잡한 차선의 weight을 0으로 설정
                                else
                                    all_lanes_congested = false;  % 하나라도 혼잡하지 않은 차선이 있으면 false
                                end
                            end
                        end
                    end
                    
                    % 모든 차선이 혼잡한 경우 현재 차선의 weight을 1로 설정
                    if all_lanes_congested
                        weights(currentLane) = 1;
                    end
                end                

                for lane = 1:Parameter.Map.Lane
                    t_demand(lane, i) = weights(lane); 
                end

            end
    end


    % t_demand = size(List.Vehicle.Active, 1) * 100 * ones(Parameter.Map.Lane, 1);

    % Alloc_current 생성
    Alloc_current = zeros(n,1);
    for i = 1:n
        vid = List.Vehicle.Active(i,1);
        Alloc_current(i) = List.Vehicle.Object{vid}.Lane;
    end


    environment.t_location = t_location;
    environment.a_location = a_location;
    environment.t_demand = t_demand;
    environment.Alloc_current = Alloc_current;
    environment.Type = Setting.NumberOfParticipants;
    environment.LogFile = Setting.LogPath;
    environment.x_relation = GetXRelation(List);
    environment.number_of_tasks = Parameter.Map.Lane;
    environment.Util_type = Setting.Util_type;
    environment.Setting = Setting;
    environment.Parameter = Parameter;
    environment.List = List;

end

% GetFrontVehicle and GetRearVehicle functions are now in Manager folder
