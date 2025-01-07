function [feasible] = SimpleLaneChange(vehicle, desired_lane, List, Parameter)

    feasible = CheckLaneChangeFeasibility(vehicle, desired_lane, List, Parameter);

    if  ~feasible
        [~, ~, QuitFlag, objVelocity] = LaneChangeWhenNoFeasible(vehicle,desired_lane,Parameter,List);
        if QuitFlag
            disp('this never happens but added for just in case');
        else
            vehicle.Velocity = objVelocity;
        end
        
    else
        
    end
end




function feasible = CheckLaneChangeFeasibility(obj, targetLane, List, Parameter)
    % 현재 차량 위치와 목표 차선의 선행/후행 차량 간 거리 계산
    feasible = false;  % 기본값: 변경 불가
    current_x = obj.Location * Parameter.Map.Scale;  % 현재 차량의 x 좌표
    lane_vehicles = List.Vehicle.Active(List.Vehicle.Active(:,3) == targetLane, :);  % 목표 차선의 차량

    % 목표 차선의 선행/후행 차량 찾기
    distances = lane_vehicles(:,4)*Parameter.Map.Scale - current_x;
    front_distances = distances(distances > 0);
    rear_distances = distances(distances < 0);

    % 안전 거리 조건
    safe_distance = Parameter.Veh.SafeDistance;
    if isempty(front_distances) || min(front_distances) > safe_distance
        if isempty(rear_distances) || abs(max(rear_distances)) > safe_distance
            feasible = true;  % 선행/후행 모두 안전 거리 만족
        end
    end
end

function [front_vehicle, front_distance] = GetFrontVehicle(obj, targetLane, List, Parameter)
    % 현재 차선의 선행 차량 찾기
    current_x = obj.Location * Parameter.Map.Scale;

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


function [rear_vehicle, rear_distance] = GetRearVehicle(obj, targetLane, List, Parameter)
    % 현재 차선의 후행 차량 찾기
    current_x = obj.Location * Parameter.Map.Scale;

    % 목표 차선 차량 필터링
    lane_vehicles = List.Vehicle.Active(List.Vehicle.Active(:,3) == targetLane, :);

    % 모든 차량의 거리 계산
    distances = lane_vehicles(:,4) * Parameter.Map.Scale - current_x;

    % 후행 차량 거리 필터링
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

function [AccelFlag, DecelFlag, QuitFlag, objVelocity]= LaneChangeWhenNoFeasible(obj,targetLane, Parameter,List)
    AccelFlag = 0;
    DecelFlag = 0;
    QuitFlag = 0;

    % 안전 거리를 만족하지 못하면 감속/가속
    [front_vehicle, front_distance] = GetFrontVehicle(obj, targetLane, List, Parameter);
    [rear_vehicle, rear_distance] = GetRearVehicle(obj, targetLane, List, Parameter);

    if isempty(front_vehicle) || front_distance > Parameter.Veh.SafeDistance
        objVelocity = min(obj.Velocity + obj.Parameter.Accel(1), obj.Parameter.MaxVel);  % 가속해서 합류하기
        AccelFlag = 1;
    
    elseif isempty(rear_vehicle) || abs(rear_distance) > Parameter.Veh.SafeDistance
        objVelocity = max(obj.Velocity - obj.Parameter.Accel(2), obj.Parameter.MinVel);  % 감속해서 합류하기
        DecelFlag = 1;

    else
        objVelocity = obj.Velocity;
        QuitFlag = 1;
        disp("error case!");
    
        % obj.CheckLaneChangeFeasibility(targetLane, List, Parameter);
    end
end