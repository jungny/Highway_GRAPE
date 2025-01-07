function environment = GRAPE_main(List, Parameter,Setting,testiteration)
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
    
    transition_distance = 300 + 50*Parameter.Map.Lane;
    raw_weights = zeros(Parameter.Map.Lane,1);

    for i = 1:size(List.Vehicle.Active, 1)
        vehicle_id = List.Vehicle.Active(i, 1);  % 차량 ID
        distance_to_exit = List.Vehicle.Object{vehicle_id}.Exit - ...
                            List.Vehicle.Object{vehicle_id}.Location * Parameter.Map.Scale;  % Exit까지 거리

        % Case 1: Distance greater than transition distance (uniform weights)
        if distance_to_exit > transition_distance
            weights = ones(Parameter.Map.Lane, 1) / Parameter.Map.Lane;  % 균일 분포
        else
            % Case 2: Distance less than or equal to transition distance
            for lane = 1:Parameter.Map.Lane
                k = 2; % k 작을수록 lane별 더 극단적인 차이가 발생 
                % Weight increases as lane number increases
                raw_weights(lane) = exp(-(transition_distance - distance_to_exit) / (k * lane));
            end
            
            % Normalize weights to ensure they sum to 1
            weights = raw_weights / sum(raw_weights);
        end
    
        % t_demand에 반영
        for lane = 1:Parameter.Map.Lane
            normalized_weights(lane) = floor(weights(lane)*100)/100;
            t_demand(lane, i) = size(List.Vehicle.Active, 1)*normalized_weights(lane);  % vehicle 수 곱해 비율 유지
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
    environment.LogFile = Setting.LogFile;
    environment.test_iteration = testiteration;
    environment.vehicles_ahead = GetVehiclesAhead(List,Parameter);


end