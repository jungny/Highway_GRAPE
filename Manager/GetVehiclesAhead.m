function vehicles_ahead_matrix = GetVehiclesAhead(List, Parameter)
    % GetVehiclesAhead
    % Calculate the number of vehicles ahead for each vehicle in each lane.
    %
    % Inputs:
    %   - List: Vehicle list containing active vehicles and their objects
    %   - Parameter: Simulation parameters, including number of lanes
    %
    % Output:
    %   - vehicles_ahead_matrix: Matrix where each row corresponds to a vehicle
    %                            and each column represents the count of vehicles
    %                            ahead in each lane.

    % Initialize the matrix
    num_vehicles = size(List.Vehicle.Active, 1);
    num_lanes = Parameter.Map.Lane;
    vehicles_ahead_matrix = zeros(num_vehicles, num_lanes);

    % Iterate over all active vehicles
    for vehicle_idx = 1:num_vehicles
        % Get current vehicle's ID, lane, and position
        vehicle_id = List.Vehicle.Active(vehicle_idx, 1);
        vehicle_lane = List.Vehicle.Object{vehicle_id}.Lane;
        vehicle_position = List.Vehicle.Object{vehicle_id}.Location;

        % Calculate the number of vehicles ahead for each lane
        for lane_idx = 1:num_lanes
            % Filter vehicles in the current lane
            vehicles_in_lane = List.Vehicle.Active(List.Vehicle.Active(:, 3) == lane_idx, :);

            % Count vehicles with positions ahead of the current vehicle
            vehicles_ahead_matrix(vehicle_idx, lane_idx) = sum(...
                arrayfun(@(v_id) List.Vehicle.Object{v_id}.Location > vehicle_position, ...
                         vehicles_in_lane(:, 1)) ...
            );
        end
    end
end
