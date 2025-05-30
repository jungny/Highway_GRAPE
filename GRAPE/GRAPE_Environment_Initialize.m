function environment = GRAPE_Environment_Initialize(List, Parameter,Setting)
    % AllocationLaneDuringGRAPE 초기화
    for i = 1:size(List.Vehicle.Active, 1)
        vehicle_id = List.Vehicle.Active(i, 1); 
        vehicle = List.Vehicle.Object{vehicle_id};
        vehicle.AllocLaneDuringGRAPE = [];
    end
    
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
                    decelflag = false;
                    leftflag = false;
                    rightflag = false;
                    %cur_front_dist = NaN;
                    left_front_dist = -inf;
                    right_front_dist = -inf;

                
                    % (4) 내가 감속 중이면 decelflag
                    % if obj.Acceleration < -1 && obj.Velocity < 25
                    %     decelflag = true;
                    % end
                    % ALDG의 효과가 나타나는지 확인하기 위해 조건 간소화
                    decelflag = true;
                
                    % 현재 차선의 선행 차량 거리
                    [cur_front_vehicle, front_dist] = GetFrontVehicle(obj, currentLane, List, Parameter, Setting);
                    cur_front_dist = front_dist; 
                    if isempty(cur_front_vehicle)
                        cur_front_dist = inf;
                    end
                
                    % 왼쪽 차선 조건
                    if currentLane > 1
                        leftLane = currentLane - 1;
                        [left_front_vehicle, front_dist] = GetFrontVehicle(obj, leftLane, List, Parameter, Setting);
                        left_front_dist = front_dist;
                        if isempty(left_front_vehicle)
                            left_front_dist = inf;
                        end
                
                        if left_front_dist > cur_front_dist  % (5)
                            leftflag = true;
                        end
                    end
                
                    % 오른쪽 차선 조건
                    if currentLane < Parameter.Map.Lane
                        rightLane = currentLane + 1;
                        [right_front_vehicle, front_dist] = GetFrontVehicle(obj, rightLane, List, Parameter, Setting);
                        right_front_dist = front_dist;
                        if isempty(right_front_vehicle)
                            right_front_dist = inf;
                        end
                
                        if right_front_dist > cur_front_dist  % (5)
                            if right_front_dist == left_front_dist
                                rightflag = true; % leftflag = already true
                            elseif right_front_dist > left_front_dist
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
                        weights(currentLane - 1) = 3;
                        weights(currentLane + 1) = 3;
                    elseif decelflag && leftflag
                        weights(currentLane - 1) = 3;
                    elseif decelflag && rightflag
                        weights(currentLane + 1) = 3;
                    else
                        weights(currentLane) = 3;  % 조건 안 맞으면 원래 차선 유지
                    end
                end

                if vehicle_id == 17
                    %disp(weights);
                end
                

                for lane = 1:Parameter.Map.Lane
                    normalized_weights(lane) = floor(weights(lane)*100)/100;
                    t_demand(lane, i) = normalized_weights(lane); 
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
    environment.x_relation = GetXRelation(List);
    environment.number_of_tasks = Parameter.Map.Lane;
    environment.Util_type = Setting.Util_type;
    environment.Setting = Setting;
    environment.Parameter = Parameter;
    environment.List = List;

end

%%
%%

% Initialize에서 ALDG를 쓸 일은 없지만, 이대로도 잘 기능하는 코드이므로 그대로 두기로 함
function [front_vehicle, front_distance] = GetFrontVehicle(obj, targetLane, List, Parameter, Setting)
    % 현재 차선의 선행 차량 찾기
    current_x = double(obj.Location * Parameter.Map.Scale);

    % 목표 차선의 차량 필터링
    vehicle_ids = List.Vehicle.Active(:,1);  % 모든 vehicle id 추출
    is_target = false(size(vehicle_ids));   % 논리 인덱싱 초기화
    
    for i = 1:length(vehicle_ids)
        vid = vehicle_ids(i);
        
        if Setting.GRAPEmode == 0 % GRAPE
            if ~isempty(List.Vehicle.Object{vid}.AllocLaneDuringGRAPE) 
                if List.Vehicle.Object{vid}.AllocLaneDuringGRAPE == targetLane
                    is_target(i) = true;
                end
            else % vehicle.AllocLaneDuringGRAPE is empty. 
                 % not empty at very start of the GRAPE instance or the veh doesnot change lane during phase 2 
                if ~isempty(List.Vehicle.Object{vid}.TargetLane) % this would not happen bc all vehs
                                                                 % normally success their lc before another GRAPE
                    if List.Vehicle.Object{vid}.TargetLane == targetLane
                        is_target(i) = true;
                    end
                else % vehicle.TargetLane is empty
                    if List.Vehicle.Object{vid}.Lane == targetLane
                        is_target(i) = true;
                    end
                end
            end
        else % Greedy or CycleGreedy
            if ~isempty(List.Vehicle.Object{vid}.TargetLane)
                if List.Vehicle.Object{vid}.TargetLane == targetLane
                    is_target(i) = true;
                end
            else % vehicle.TargetLane is empty
                if List.Vehicle.Object{vid}.Lane == targetLane
                    is_target(i) = true;
                end
            end
        end
    end
    
    % 필터링된 Active 정보
    lane_vehicles = List.Vehicle.Active(is_target, :);

    % 모든 차량의 거리 계산
    distances = lane_vehicles(:,4) * Parameter.Map.Scale - current_x;

    % 선행 차량 거리 필터링
    front_distances = distances(distances > 0 & distances <= 200);

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