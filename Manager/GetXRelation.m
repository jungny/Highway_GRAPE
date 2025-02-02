function x_relation = GetXRelation(List)
    % GetXRelation
    % Generate an n x n binary matrix indicating which vehicles are ahead of others.
    %
    % Inputs:
    %   - List: Vehicle list containing active vehicles and their objects
    %
    % Output:
    %   - x_relation: n x n matrix where x_relation(i, j) = 1 if vehicle i is ahead of vehicle j

    % Get the number of active vehicles
    num_vehicles = size(List.Vehicle.Active, 1);

    % Initialize the relation matrix
    x_relation = zeros(num_vehicles, num_vehicles);

    % Extract vehicle positions
    vehicle_positions = zeros(num_vehicles, 1);
    for i = 1:num_vehicles
        vehicle_id = List.Vehicle.Active(i, 1);
        vehicle_positions(i) = List.Vehicle.Object{vehicle_id}.Location;
    end

    % Compare positions to determine relative ordering
    for i = 1:num_vehicles
        for j = 1:num_vehicles
            if vehicle_positions(i) < vehicle_positions(j)  % (does not include oneself)
                x_relation(i, j) = 1;
            end
        end
    end
end
