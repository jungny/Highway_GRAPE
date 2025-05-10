function [List, TotalVehicles, firstCount, InVehBuffer, RandomValBCup, vehIDCounter] = ...
    SpawnAuto(List, Parameter, Time, TotalVehicles, firstCount, InVehBuffer, RandomValBCup, vehIDCounter)

    % 상수 설정
    dt = Parameter.Physics;
    Qin = Parameter.Flow / 3600;  % veh/sec
    randomAmplitude = 0.2;
    numLanes = Parameter.Map.Lane;

    % 초기화
    if isempty(InVehBuffer)
        InVehBuffer = 0;
    end
    if isempty(RandomValBCup)
        RandomValBCup = 1;
    end
    if ~isfield(List, "Vehicle") || ~isfield(List.Vehicle, "Object")
        List.Vehicle.Object = {};
    end

    % 버퍼 업데이트
    InVehBuffer = InVehBuffer + Qin * dt;

    % 생성 조건 확인
    if InVehBuffer >= RandomValBCup
        RandomValBCup = 1 + randomAmplitude * (2 * rand() - 1);
        vehIDCounter = vehIDCounter + 1;
        vehID = vehIDCounter;
        lane = randi(numLanes);
        exit = randsample(Parameter.Map.Exit, 1);
        politeness = 1;
        spawnPos = 1;
        newVeh = [vehID; lane; exit; politeness; spawnPos; Time];

        List.Vehicle.Object{vehID} = Vehicle(newVeh(1:6), Time, Parameter);

        if firstCount == 0
            List.Vehicle.Object = cat(1, List.Vehicle.Object, cell(100, 1));
            firstCount = 1;
        end

        TotalVehicles = TotalVehicles + 1;
        InVehBuffer = InVehBuffer - 1;
    end
end
