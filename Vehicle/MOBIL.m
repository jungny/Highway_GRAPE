function [feasible, a_c_sim] = MOBIL(vehicle, target_lane, List, Parameter)
    % MOBIL 알고리즘을 통한 차선 변경 가능 여부 평가

    % 현재 차선의 선행/후행 차량 정보 가져오기
    [front_vehicle_cur, front_distance_cur] = GetFrontVehicle(vehicle, vehicle.Lane, List, Parameter);
    [rear_vehicle_cur, ~] = GetRearVehicle(vehicle, vehicle.Lane, List, Parameter);

    % 목표 차선의 선행/후행 차량 정보 가져오기
    [front_vehicle_target, front_distance_target] = GetFrontVehicle(vehicle, target_lane, List, Parameter);
    [rear_vehicle_target, rear_distance_target] = GetRearVehicle(vehicle, target_lane, List, Parameter);

    % 현재 차선과 목표 차선의 가속도 계산
    a_c_nochange = ComputeAcceleration(vehicle, front_vehicle_cur, front_distance_cur, List, Parameter);
    a_c_sim = ComputeAcceleration(vehicle, front_vehicle_target, front_distance_target, List, Parameter);

    % 후행 차량 가속도 변화 계산
    delta_a_n = 0; % 목표 차선 후행 차량의 가속도 변화 초기화 New Follower
    if ~isempty(rear_vehicle_target)
        a_n_change = ComputeAcceleration(rear_vehicle_target, front_vehicle_target, front_distance_target, List, Parameter);
        a_n_nochange = ComputeAcceleration(rear_vehicle_target, front_vehicle_cur, front_distance_cur, List, Parameter);
        delta_a_n = a_n_change - a_n_nochange;
    end

    delta_a_o = 0; % 현재 차선 후행 차량의 가속도 변화 초기화 Old Follower
    if ~isempty(rear_vehicle_cur)
        a_o_change = ComputeAcceleration(rear_vehicle_cur, front_vehicle_cur, front_distance_cur, List, Parameter);
        a_o_nochange = ComputeAcceleration(rear_vehicle_cur, front_vehicle_cur, front_distance_target, List, Parameter);
        delta_a_o = a_o_change - a_o_nochange;
    end

    % 동기 기준 평가
    delta_a_self = a_c_sim - a_c_nochange;
    %incentive_flag_test = (delta_a_self + vehicle.PolitenessFactor * (delta_a_n + delta_a_o)) ;
    %if isnan(incentive_flag_test)
    %    disp('bomb')
    %    a_o_nochange = ComputeAcceleration(rear_vehicle_cur, front_vehicle_cur, front_distance_target, List, Parameter);
    %end
        
    incentive_flag = (delta_a_self + vehicle.PolitenessFactor * (delta_a_n + delta_a_o)) > Parameter.Veh.AccelThreshold;
    % fprintf('vehicle_id: %d, value: %.2f\n', vehicle.ID, (delta_a_self + vehicle.PolitenessFactor * (delta_a_n + delta_a_o)));


    % 안전 기준 평가
    safety_flag = CheckSafety(front_distance_target, rear_distance_target, Parameter);
    % if safety_flag==0
    %    disp('bomb')
    % end

    % 최종 판단
    feasible = safety_flag && incentive_flag;
    fprintf('Vehicle %d | to %d | feasible: %d   safety_flag: %d\n', vehicle.ID, target_lane, feasible, safety_flag);
end

function vehicle_acceleration = ComputeAcceleration(vehicle, front_vehicle, front_distance, List, Parameter)
    % 차량 가속도를 계산하는 함수
    % Inputs:
    %   - vehicle: 현재 차량 객체
    %   - front_vehicle: 선행 차량 객체
    %   - front_distance: 선행 차량과의 거리
    %   - Parameter: 파라미터
    % Output:
    %   - vehicle_acceleration: 계산된 가속도

    if ~isobject(vehicle)
        vehicle = List.Vehicle.Object{vehicle(1)};
    end

    % 1. 선행 차량에 의한 가속도 계산
    if ~isempty(front_vehicle)
        if ~isobject(front_vehicle)
            front_vehicle = List.Vehicle.Object{front_vehicle(1)};
        end
        % 속도 차이
        delta_v = vehicle.Velocity - front_vehicle.Velocity;

        % 안전 거리 계산
        safe_distance = Parameter.Veh.SafeDistance + ...
                        vehicle.Velocity * Parameter.Veh.Headway - ...
                        (vehicle.Velocity * delta_v) / (2 * sqrt(Parameter.Veh.Accel(1) * abs(Parameter.Veh.Accel(2))));

        % 안전 거리가 음수로 계산되지 않도록 보정
        safe_distance = max(safe_distance, Parameter.Veh.SafeDistance);

        % 가속도 계산
        a_front = Parameter.Veh.Accel(1) * (1 - (front_distance / safe_distance)^2);
    else
        % 선행 차량이 없으면 최대 가속도로 설정
        a_front = Parameter.Veh.Accel(1);
    end

    % 2. 현재 차량의 최소, 최대 가속도 제한
    vehicle_acceleration = min(max(a_front, Parameter.Veh.Accel(1)), Parameter.Veh.Accel(2));

   

    % 3. 감속 처리 (선행 차량과 너무 가까운 경우)
    if front_distance < Parameter.Veh.SafeDistance
        vehicle_acceleration = max(vehicle_acceleration, Parameter.Veh.Accel(2)); % 감속
    end
end



function [front_vehicle, front_distance] = GetFrontVehicle(vehicle, lane, List, Parameter)
    % 선행 차량 정보 가져오기
    current_x = vehicle.Location * Parameter.Map.Scale;
    lane_vehicles = List.Vehicle.Active(List.Vehicle.Active(:, 3) == lane, :);

    distances = lane_vehicles(:, 4) * Parameter.Map.Scale - current_x;
    front_distances = distances(distances > 0);

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

function [rear_vehicle, rear_distance] = GetRearVehicle(vehicle, lane, List, Parameter)
    % 후행 차량 정보 가져오기
    current_x = vehicle.Location * Parameter.Map.Scale;
    lane_vehicles = List.Vehicle.Active(List.Vehicle.Active(:, 3) == lane, :);

    distances = lane_vehicles(:, 4) * Parameter.Map.Scale - current_x;
    rear_distances = distances(distances < 0);

    % 초기화
    rear_vehicle = [];
    rear_distance = inf;

    if ~isempty(rear_distances)
        % rear_distances 값이 distances에서의 원래 인덱스 찾기
        [rear_distance, ~] = max(rear_distances);
        tolerance = 1e-6;
        original_idx = find(abs(distances - rear_distance) < tolerance, 1, 'first'); % 동일 거리의 원래 인덱스

        % 해당 인덱스의 차량 정보 추출
        rear_vehicle = lane_vehicles(original_idx, :);
    end
end

function safety_flag = CheckSafety(front_distance, rear_distance, Parameter)
    % 안전 기준 평가
    safe_distance = Parameter.Veh.SafeDistance;
    vehicle_length = Parameter.Veh.Size(1);
    safety_flag = (front_distance - vehicle_length > safe_distance) && (abs(rear_distance) - vehicle_length > safe_distance);
end


