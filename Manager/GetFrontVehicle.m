function [front_vehicle, front_distance] = GetFrontVehicle(obj, checkLane, List, Parameter, Setting, vehicle_ids, vehicle_lanes, vehicle_locations, vehicle_targetlanes, vehicle_alloclanes)
    % 현재 차선의 선행 차량 찾기 (벡터화)
    current_x = double(obj.Location * Parameter.Map.Scale);
    if isnan(Setting.BubbleRadius) || Setting.BubbleRadius > 200
        considerationRange = 200;
    else
        considerationRange = Setting.BubbleRadius;
    end
    % 타겟 차량 논리 인덱싱
    is_target = (vehicle_alloclanes == checkLane) | ...
                (isnan(vehicle_alloclanes) & (vehicle_targetlanes == checkLane)) | ...
                (isnan(vehicle_alloclanes) & isnan(vehicle_targetlanes) & (vehicle_lanes == checkLane));
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