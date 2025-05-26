function [rear_vehicle, rear_distance] = GetRearVehicle(obj, checkLane, List, Parameter, Setting, vehicle_ids, vehicle_lanes, vehicle_locations, vehicle_targetlanes, vehicle_alloclanes)
    % 현재 차선의 후행 차량 찾기 (벡터화)
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