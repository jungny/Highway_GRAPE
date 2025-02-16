
function [SpawnSeed, NewList] = GetSeed(Setting, Parameter, TotalVehicles, SpawnLanes, OldList)
    switch Setting.SpawnType
        case 0
            % 1: Vehicle ID
            % 2: Spawn Lane
            % 3: Exit
            % 4: Politeness Factor
            % 5: Spawn Position
            % 6: Spawn Time

            SpawnCount = length(SpawnLanes);

            SpawnSeed = zeros(6,SpawnCount);
            SpawnSeed(1,:) = TotalVehicles + 1 : TotalVehicles + SpawnCount;
            SpawnSeed(2,:) = SpawnLanes;

            NumExits = length(Parameter.Map.Exit);
            SpawnSeed(3, :) = Parameter.Map.Exit(randi(NumExits, 1, SpawnCount));
            SpawnSeed(4,:) = ones(1,SpawnCount);

            % Poisson Process 기반 다음 차량 도착 시간 업데이트
            lambda = Parameter.Flow / 3600; % [veh/hour/lane] -> [veh/sec/lane]
            for i = 1:length(SpawnLanes)
                OldList(SpawnLanes(i)) = OldList(SpawnLanes(i)) - log(rand) / lambda;
            end
            NewList = OldList;

            SpawnSeed(5,:) = 1+0*Parameter.Map.SpawnZone * rand(1, SpawnCount);
            %SpawnSeed(5,:) = Parameter.Map.SpawnZone * rand(1, SpawnCount);

            SpawnSeed(6,:) = zeros(1,SpawnCount); % redundant property
        case 1
            TotalVehicles = randi([2,50]);
            SpawnSeed = zeros(6,TotalVehicles);

            % 1: Vehicle ID
            % 2: Spawn Lane
            % 3: Exit
            % 4: Politeness Factor
            % 5: Spawn Position
            % 6: Spawn Time
            SpawnSeed(1,:) = 1:TotalVehicles;
            SpawnSeed(2,:) = randi([1, Parameter.Map.Lane], [1, TotalVehicles]);
            NumExits = length(Parameter.Map.Exit);
            SpawnSeed(3, :) = Parameter.Map.Exit(randi(NumExits, 1, TotalVehicles));
            SpawnSeed(4,:) = ones(1,TotalVehicles);
            SpawnSeed(5,:) = ones(1,TotalVehicles); % redundant property
            min_interval = 0.5;
            max_interval = 2;
            SpawnTimes = cumsum(min_interval + (max_interval - min_interval) * rand(1, TotalVehicles));
            SpawnSeed(6, :) = SpawnTimes;
            NewList = [];
            
    end
    
end
