function [FeasibleLanes] = EvaluateLaneFeasibility(vehicle, List, Parameter)
    MLC_flag = false; % 임시로 false
    % Determine if the vehicle is in MLC position or not
    if MLC_flag
        FeasibleLanes = MlcFeasibility(vehicle, List, Parameter);
    else
        FeasibleLanes = DlcFeasibility(vehicle, List, Parameter);
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

function [FeasibleLanes] = MlcFeasibility(vehicle, List, Parameter)
    FeasibleLanes = ones(1, Parameter.Map.Lane);  % 임시
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