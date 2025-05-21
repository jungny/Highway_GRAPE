function environment = GRAPE_Environment_Update(List, Parameter, Setting, past_env)
    % AllocationLaneDuringGRAPE 초기화--는 하면 X
    % for i = 1:size(List.Vehicle.Active, 1)
    %     vehicle_id = List.Vehicle.Active(i, 1); 
    %     vehicle = List.Vehicle.Object{vehicle_id};
    %     vehicle.AllocLaneDuringGRAPE = [];
    % end
    
    % a_location 생성
    a_location = zeros(size(List.Vehicle.Active, 1), 2);
    for i = 1:size(List.Vehicle.Active, 1)
        vehicle_id = List.Vehicle.Active(i, 1);  % 현재 차량 ID
        a_location(i, :) = [List.Vehicle.Object{vehicle_id}.Location, ...
                            (Parameter.Map.Lane-List.Vehicle.Object{vehicle_id}.Lane+0.5) * Parameter.Map.Tile];  % 차량의 현재 (x, y) 위치
    end

    % t_location, t_demand 생성 -- location은 cost 계산을 위해서 필요하므로 allocation 과정에 따라 바꿀 필요 X
    t_location = zeros(Parameter.Map.Lane, 2);
    for i = 1:Parameter.Map.Lane
        t_location(i, :) = [0, (Parameter.Map.Lane-i+0.5) * Parameter.Map.Tile];  % (x, y) 좌표로 정의 (x는 0으로 고정)
    end

    t_demand = zeros(Parameter.Map.Lane, size(List.Vehicle.Active,1));  
    % t_demand(:) = 100*size(List.Vehicle.Active, 1);
    
    % transition_distance = 300 + 15*Parameter.Map.Lane^2;
    % raw_weights = zeros(Parameter.Map.Lane,1);

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

            % 차량 정보 배열화 (캐싱)
            vehicle_ids = List.Vehicle.Active(:,1);
            num_vehicles = length(vehicle_ids);
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
                        % decelflag = true;
                    % end
                    % ALDG의 효과가 나타나는지 확인하기 위해 조건 간소화
                    decelflag = true;
                
                    % 현재 차선의 선행 차량 거리
                    [cur_front_vehicle, front_dist] = GetFrontVehicle(obj, currentLane, List, Parameter, Setting, vehicle_ids, vehicle_lanes, vehicle_locations, vehicle_targetlanes, vehicle_alloclanes);
                    [cur_rear_vehicle, rear_dist] = GetRearVehicle(obj, currentLane, List, Parameter, Setting, vehicle_ids, vehicle_lanes, vehicle_locations, vehicle_targetlanes, vehicle_alloclanes);
                    cur_front_dist = front_dist; 
                    cur_rear_dist = rear_dist;
                    if isempty(cur_front_vehicle)
                        cur_front_dist = 200;
                    end
                    if isempty(cur_rear_vehicle)
                        cur_rear_dist = 200;
                    end
                    cur_dist = cur_front_dist + cur_rear_dist;
                
                    % 왼쪽 차선 조건
                    if currentLane > 1
                        leftLane = currentLane - 1;
                        [left_front_vehicle, front_dist] = GetFrontVehicle(obj, leftLane, List, Parameter, Setting, vehicle_ids, vehicle_lanes, vehicle_locations, vehicle_targetlanes, vehicle_alloclanes);
                        [left_rear_vehicle, rear_dist] = GetRearVehicle(obj, leftLane, List, Parameter, Setting, vehicle_ids, vehicle_lanes, vehicle_locations, vehicle_targetlanes, vehicle_alloclanes);
                        left_front_dist = front_dist;
                        left_rear_dist = rear_dist;
                        if isempty(left_front_vehicle)
                            left_front_dist = 200;
                        end
                        if isempty(left_rear_vehicle)
                            left_rear_dist = 200;
                        end
                        left_dist = left_front_dist + left_rear_dist;
                
                        if left_dist > cur_dist  % (5)
                            leftflag = true;
                        end
                    end
                
                    % 오른쪽 차선 조건
                    if currentLane < Parameter.Map.Lane
                        rightLane = currentLane + 1;
                        [right_front_vehicle, front_dist] = GetFrontVehicle(obj, rightLane, List, Parameter, Setting, vehicle_ids, vehicle_lanes, vehicle_locations, vehicle_targetlanes, vehicle_alloclanes);
                        [right_rear_vehicle, rear_dist] = GetRearVehicle(obj, rightLane, List, Parameter, Setting, vehicle_ids, vehicle_lanes, vehicle_locations, vehicle_targetlanes, vehicle_alloclanes);
                        right_front_dist = front_dist;
                        right_rear_dist = rear_dist;
                        if isempty(right_front_vehicle)
                            right_front_dist = 200;
                        end
                        if isempty(right_rear_vehicle)
                            right_rear_dist = 200;
                        end
                        right_dist = right_front_dist + right_rear_dist;
                
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

                    % k = 10; % k값

                    % rand_idx = rand(Parameter.Map.Lane, 1) > 0.5;  % 50% 확률로 뽑기 (원하면 확률 조정 가능)
                    % weights(rand_idx) = k;
                end

                if vehicle_id == 17
                    %disp(weights);
                end
                

                for lane = 1:Parameter.Map.Lane
                    t_demand(lane, i) = weights(lane); 
                end

            end
    end


    % t_demand = size(List.Vehicle.Active, 1) * 100 * ones(Parameter.Map.Lane, 1);

    % Alloc_current 생성--도 필요 없음
    % Alloc_current = [];
    % for i = 1:size(List.Vehicle.Active, 1)
    %     Alloc_current = [Alloc_current; List.Vehicle.Object{List.Vehicle.Active(i, 1)}.Lane];
    % end


    % 다른 것들은 유지하고
    environment.t_location = past_env.t_location;
    environment.a_location = past_env.a_location;
    environment.Alloc_current = past_env.Alloc_current;
    environment.Type = past_env.Type;
    environment.LogFile = past_env.Setting.LogPath;
    environment.x_relation = past_env.x_relation;
    environment.number_of_tasks = past_env.number_of_tasks;
    environment.Util_type = past_env.Util_type;
    environment.Setting = Setting;
    environment.Parameter = Parameter;
    % environment.List = List;


    % Task demand만 바꾸고 차량 정보는 혹시나 해서 같이 업데이트
    environment.t_demand = t_demand;
    environment.List = List;

end

%%
%%

function [front_vehicle, front_distance] = GetFrontVehicle(obj, targetLane, List, Parameter, Setting, vehicle_ids, vehicle_lanes, vehicle_locations, vehicle_targetlanes, vehicle_alloclanes)
    % 현재 차선의 선행 차량 찾기 (벡터화)
    current_x = double(obj.Location * Parameter.Map.Scale);
    if isnan(Setting.BubbleRadius) || Setting.BubbleRadius > 200
        considerationRange = 200;
    else
        considerationRange = Setting.BubbleRadius;
    end
    % 타겟 차량 논리 인덱싱
    is_target = (vehicle_alloclanes == targetLane) | ...
                (isnan(vehicle_alloclanes) & (vehicle_targetlanes == targetLane)) | ...
                (isnan(vehicle_alloclanes) & isnan(vehicle_targetlanes) & (vehicle_lanes == targetLane));
    % 거리 계산
    target_locations = vehicle_locations(is_target);
    target_ids = vehicle_ids(is_target);
    distances = (target_locations * Parameter.Map.Scale) - current_x;
    % 선행 차량
    front_mask = distances > 0 & distances <= considerationRange;
    if any(front_mask)
        [front_distance, idx] = min(distances(front_mask));
        front_vehicle = target_ids(front_mask);
        front_vehicle = front_vehicle(idx);
    else
        front_vehicle = [];
        front_distance = inf;
    end
end

function [rear_vehicle, rear_distance] = GetRearVehicle(obj, targetLane, List, Parameter, Setting, vehicle_ids, vehicle_lanes, vehicle_locations, vehicle_targetlanes, vehicle_alloclanes)
    % 현재 차선의 후행 차량 찾기 (벡터화)
    current_x = double(obj.Location * Parameter.Map.Scale);
    if isnan(Setting.BubbleRadius) || Setting.BubbleRadius > 200
        considerationRange = 200;
    else
        considerationRange = Setting.BubbleRadius;
    end
    % 타겟 차량 논리 인덱싱
    is_target = (vehicle_alloclanes == targetLane) | ...
                (isnan(vehicle_alloclanes) & (vehicle_targetlanes == targetLane)) | ...
                (isnan(vehicle_alloclanes) & isnan(vehicle_targetlanes) & (vehicle_lanes == targetLane));
    % 거리 계산
    target_locations = vehicle_locations(is_target);
    target_ids = vehicle_ids(is_target);
    distances = (target_locations * Parameter.Map.Scale) - current_x;
    % 후행 차량
    rear_mask = distances < 0 & distances >= -considerationRange;
    if any(rear_mask)
        [rear_distance, idx] = max(distances(rear_mask));
        rear_vehicle = target_ids(rear_mask);
        rear_vehicle = rear_vehicle(idx);
        rear_distance = abs(rear_distance);
    else
        rear_vehicle = [];
        rear_distance = inf;
    end
end
