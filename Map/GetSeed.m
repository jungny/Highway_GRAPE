
function [SpawnSeed, NewListOrTotalVehicles] = GetSeed(Setting, Parameter, TotalVehicles, SpawnLanes, OldList)
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
            NewListOrTotalVehicles = OldList;

            SpawnSeed(5,:) = 1+0*Parameter.Map.SpawnZone * rand(1, SpawnCount);
            %SpawnSeed(5,:) = Parameter.Map.SpawnZone * rand(1, SpawnCount);

            SpawnSeed(6,:) = zeros(1,SpawnCount); % redundant property
        case 1
            %TotalVehicles = randi([2,50]);
            TotalVehicles = randi([70,120]);
            %TotalVehicles = 20;
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
            NewListOrTotalVehicles = TotalVehicles;
            
        case 2  % Debug for task allocation issue
            % 차량 3대
            TotalVehicles = 3;
            SpawnSeed = zeros(6, TotalVehicles);

            % 1: Vehicle ID
            SpawnSeed(1,:) = 1:TotalVehicles;

            % 2: Spawn Lane (같은 차선에서 스폰되도록 설정)
            SpawnSeed(2,:) = ones(1, TotalVehicles);
            SpawnSeed(2,:) = [3,2,3];

            % 3: Exit (랜덤 할당)
            NumExits = length(Parameter.Map.Exit);
            %SpawnSeed(3, :) = [Parameter.Map.Exit(1),Parameter.Map.Exit(1),Parameter.Map.Exit(1)];
            SpawnSeed(3, :) = Parameter.Map.Exit(randi(NumExits, 1, TotalVehicles));

            % 4: Politeness Factor (기본값 1)
            SpawnSeed(4,:) = ones(1, TotalVehicles);

            % 5: Spawn Position (redundant, 통일)
            SpawnSeed(5,:) = ones(1, TotalVehicles); % 전부 1로 설정

            % 6: Spawn Time (간격을 200m 이상 벌리기 위해 조절)
            SpawnSeed(6,1) = 0;     % 첫 번째 차량 스폰 시간
            SpawnSeed(6,2) = 0.7;   % 두 번째 차량 (거의 동시에)
            SpawnSeed(6,3) = 7;    % 세 번째 차량 (충분히 나중에)

            NewListOrTotalVehicles = 3;

    end
    
end
