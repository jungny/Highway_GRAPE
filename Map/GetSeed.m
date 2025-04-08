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
            TotalVehicles = 30;
            SpawnSeed = zeros(6,TotalVehicles);
            
            % 1: Vehicle ID, 2: Spawn Lane
            SpawnSeed(1,:) = 1:TotalVehicles;
            temp_lanes = repmat(1:Parameter.Map.Lane, 1, ceil(TotalVehicles/Parameter.Map.Lane));
            SpawnSeed(2,:) = temp_lanes(1:TotalVehicles);  % 한 줄로 합침
            
            % 3: Exit, 4: Politeness, 5: Spawn Position
            if Setting.ExitPercent == 20
                % Exit:Through = 2:8 
                SpawnSeed(3, :) = Parameter.Map.Exit(randsample([1, 2], TotalVehicles, true, [0.2, 0.8]));
            elseif Setting.ExitPercent == 50
                % Exit:Through = 5:5
                SpawnSeed(3, :) = Parameter.Map.Exit(randsample([1, 2], TotalVehicles, true, [0.5, 0.5]));
            elseif Setting.ExitPercent == 80
                % Exit:Through = 8:2
                SpawnSeed(3, :) = Parameter.Map.Exit(randsample([1, 2], TotalVehicles, true, [0.8, 0.2]));
            else
                error('지원하지 않는 ExitPercent 값입니다: %d', Setting.ExitPercent);
            end
            
            SpawnSeed(4,:) = ones(1,TotalVehicles);
            SpawnSeed(5,:) = ones(1,TotalVehicles);
            
            % Spawn interval parameters for each group
            group1_interval = [0.4, 1.5];  % [min, max] for first 20%
            group2_interval = [1.5, 3];  % [min, max] for next 70%
            group3_interval = [4, 5.0];  % [min, max] for last 10%
            
            % Generate spawn times
            group1_count = round(TotalVehicles * 0.2);
            group2_count = round(TotalVehicles * 0.7);
            SpawnTimes = zeros(1, TotalVehicles);
            last_spawn_time = zeros(1, Parameter.Map.Lane);
            
            for i = 1:TotalVehicles
                current_lane = SpawnSeed(2,i);
                
                % Determine interval based on group
                if i <= group1_count
                    interval = group1_interval(1) + (group1_interval(2) - group1_interval(1)) * rand();
                elseif i <= (group1_count + group2_count)
                    interval = group2_interval(1) + (group2_interval(2) - group2_interval(1)) * rand();
                else
                    interval = group3_interval(1) + (group3_interval(2) - group3_interval(1)) * rand();
                end
                
                SpawnTimes(i) = last_spawn_time(current_lane) + interval;
                last_spawn_time(current_lane) = SpawnTimes(i);
                fprintf('Vehicle %d: Lane %d, Spawn Time %.2f\n', i, current_lane, SpawnTimes(i));
            end
            
            % Sort vehicles by spawn time
            [~, sort_idx] = sort(SpawnTimes);
            SpawnSeed = SpawnSeed(:, sort_idx);
            SpawnSeed(6, :) = SpawnTimes(sort_idx);
            
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

        case 3  % Debug for Vehicle Dynamics
            % 차량 3대
            TotalVehicles = 11;
            SpawnSeed = zeros(6, TotalVehicles);

            % 1: Vehicle ID
            SpawnSeed(1,:) = 1:TotalVehicles;

            % 2: Spawn Lane (같은 차선에서 스폰되도록 설정)
            SpawnSeed(2,:) = ones(1, TotalVehicles);
            %SpawnSeed(2,:) = 1;
            SpawnSeed(2,:) = randi([1, Parameter.Map.Lane], [1, TotalVehicles]);

            % 3: Exit (랜덤 할당)
            %SpawnSeed(3, :) = Parameter.Map.Exit(2);
            NumExits = length(Parameter.Map.Exit);
            SpawnSeed(3, :) = Parameter.Map.Exit(randi(NumExits, 1, TotalVehicles));

            % 4: Politeness Factor (기본값 1)
            SpawnSeed(4,:) = ones(1, TotalVehicles);

            % 5: Spawn Position (redundant, 통일)
            SpawnSeed(5,:) = ones(1, TotalVehicles); % 전부 1로 설정

            % 6: Spawn Time (간격을 200m 이상 벌리기 위해 조절)
            %SpawnSeed(6,1) = 0;     % 첫 번째 차량 스폰 시간
            % SpawnSeed(6,2) = 0.7;   % 두 번째 차량 (거의 동시에)
            % SpawnSeed(6,3) = 7;    % 세 번째 차량 (충분히 나중에)
            min_interval = 0.3;
            max_interval = 1.5;
            SpawnTimes = cumsum(min_interval + (max_interval - min_interval) * rand(1, TotalVehicles));
            SpawnSeed(6, :) = SpawnTimes;

            NewListOrTotalVehicles = TotalVehicles;

    end
    
end
