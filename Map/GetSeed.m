
function [SpawnSeed, NewList] = GetSeed(Setting, Parameter, TotalVehicles, SpawnLanes, OldList)
    % 1: Vehicle ID
    % 2: Spawn Lane
    % 3: Exit
    % 4: Politeness Factor
    % 5: Spawn Position

    SpawnCount = length(SpawnLanes);

    SpawnSeed = zeros(5,SpawnCount);
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
    
end
