function [FeasibleLanes] = EvaluateLaneFeasibility(vehicle, List, Parameter, Setting)

    % Determine if the vehicle is in MLC position or not
    if CheckMLC(vehicle, Parameter, Setting)~=0
        FeasibleLanes = MlcFeasibility(vehicle, List, Parameter);
    else
        FeasibleLanes = DlcFeasibility(vehicle, List, Parameter);
    end
end

function MLC_flag = CheckMLC(vehicle, Parameter, Setting)
    % L1 = 40;
    % L2 = 40;
    % L3 = 40;
    % L4 = 40;
    % MLC_flag = false;
    % if vehicle.Lane == 1 && vehicle.Location * Parameter.Map.Scale > vehicle.Exit - (L1+L3+L4)
    %     MLC_flag = true;
    % elseif vehicle.Lane == 2 && vehicle.Location * Parameter.Map.Scale > vehicle.Exit - (L3+L4)
    %     MLC_flag = true;
    % elseif vehicle.Lane == 3 && vehicle.Location * Parameter.Map.Scale > vehicle.Exit - L4
    %     MLC_flag = true;
    % end
    L1 = 80;
    L2 = 80;
    vehicle.MLC_flag = 0;
    MLC_flag = 0;
    if vehicle.Location * Parameter.Map.Scale > vehicle.Exit - (L1+L2)
        vehicle.MLC_flag = 1;
        MLC_flag = 1;
    end
    if vehicle.Lane == 1 && vehicle.Location * Parameter.Map.Scale > vehicle.Exit - (L1+L2)
        vehicle.MLC_flag = 'to2';
    elseif vehicle.Lane == 2 && vehicle.Location * Parameter.Map.Scale > vehicle.Exit - (L2)
        vehicle.MLC_flag = 'to3';
    elseif vehicle.Lane == 3 && vehicle.Location * Parameter.Map.Scale > vehicle.Exit - (L2)
        vehicle.MLC_flag = 'to_exit';
    end
end 

function [FeasibleLanes] = MlcFeasibility(vehicle, List, Parameter)
    % Initialize feasibility array: default is all lanes are not feasible (0)
    FeasibleLanes = zeros(1, Parameter.Map.Lane);
    
    for lane = 1:Parameter.Map.Lane
        % The vehicle's current lane should always be feasible
        if lane == vehicle.Lane
            FeasibleLanes(lane) = 1;
            continue;
        end

        % Skip lanes that are not adjacent (DLC only allows changing by 1 Lane at a time)
        if abs(vehicle.Lane - lane) > 1
            continue;
        end
        % 🚫 왼쪽 차선으로는 이동 불가
        if lane < vehicle.Lane
            continue; % 왼쪽 차선은 feasibility 0으로 유지
        end

        % Get front vehicle speed and gap in the target lane
        [v_nf, gap_to_front] = GetLaneConditions(vehicle, lane, List, Parameter);

        % Compute minimum safe distance (Equation 10 from [3] paper)
        d_safe_max = Parameter.Veh.MaxVel * Parameter.LaneChangeTime;
        v_n = vehicle.Velocity;
        d_safe_min = (v_nf - v_n) * Parameter.LaneChangeTime;

        % Apply RiskFactor (λ) to determine final safe distance
        d_safe_risk = Parameter.RiskFactor * d_safe_min + (1 - Parameter.RiskFactor) * d_safe_max;

        % Check if the vehicle can change to this lane
        if gap_to_front > d_safe_risk
            FeasibleLanes(lane) = 1;  % Feasible if safe gap is available
        end
    end
end


function [FeasibleLanes] = DlcFeasibility(vehicle, List, Parameter)
    % Initialize feasibility array: default is all lanes are not feasible (0)
    FeasibleLanes = zeros(1, Parameter.Map.Lane);
    
    for lane = 1:Parameter.Map.Lane
        % The vehicle's current lane should always be feasible
        if lane == vehicle.Lane
            FeasibleLanes(lane) = 1;
            continue;
        end

        % Skip lanes that are not adjacent (DLC only allows changing by 1 Lane at a time)
        if abs(vehicle.Lane - lane) > 1
            continue;
        end

        % Get front vehicle speed and gap in the target lane
        [v_nf, gap_to_front] = GetLaneConditions(vehicle, lane, List, Parameter);

        % Compute minimum safe distance (Equation 10 from [3] paper)
        d_safe_max = Parameter.Veh.MaxVel * Parameter.LaneChangeTime;
        v_n = vehicle.Velocity;
        d_safe_min = (v_nf - v_n) * Parameter.LaneChangeTime;

        % Apply RiskFactor (λ) to determine final safe distance
        d_safe_risk = Parameter.RiskFactor * d_safe_min + (1 - Parameter.RiskFactor) * d_safe_max;

        % Check if the vehicle can change to this lane
        if gap_to_front > d_safe_risk
            FeasibleLanes(lane) = 1;  % Feasible if safe gap is available
        end
    end
end

function [v_nf, gap_to_front] = GetLaneConditions(vehicle, lane, List, Parameter)
    % Initialize default values 
    v_nf = Parameter.Veh.MaxVel; % Default to max velocity if no vehicle is ahead
    gap_to_front = inf; % Default to infinite distance if no vehicle is ahead

    % Get current vehicle's x position
    current_x = double(vehicle.Location * Parameter.Map.Scale);

    % Get vehicles in the target lane
    lane_vehicles = List.Vehicle.Active(List.Vehicle.Active(:,3) == lane, :);

    % Compute distances between the current vehicle and vehicles in the target lane
    distances = lane_vehicles(:,4) * Parameter.Map.Scale - current_x;

    % Identify front vehicles (vehicles ahead in the target lane)
    front_distances = distances(distances > 0);

    if ~isempty(front_distances)
        % Find the closest front vehicle
        [gap_to_front, min_idx] = min(front_distances);

        % Get the velocity of the closest front vehicle
        v_nf = lane_vehicles(min_idx, 5);
    end
end