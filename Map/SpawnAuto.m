function [List, TotalVehicles, InVehBuffer, RandomValBCup, vehIDCounter] = ...
    SpawnAuto(List, Parameter, Time, TotalVehicles, InVehBuffer, RandomValBCup, vehIDCounter)

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


        % 차선별 마지막 차량 위치 조사
        lane_gaps = zeros(1, numLanes);  % 각 차선의 available gap
        for l = 1:numLanes
            valid_rows = List.Vehicle.Active(:,3) == l;
            vehicle_ids = List.Vehicle.Active(valid_rows, 1);

            if isempty(vehicle_ids)
                lane_gaps(l) = Parameter.Map.Road;  % 아무 차량 없으면 전체 도로 길이
            else
                min_loc = inf;
                for id = vehicle_ids'
                    if ~isempty(List.Vehicle.Object{id})
                        loc = List.Vehicle.Object{id}.Location;
                        if loc < min_loc
                            min_loc = loc;
                        end
                    end
                end
                lane_gaps(l) = min_loc * Parameter.Map.Scale;  % 실제 거리 기준
            end
        end

        % 가장 여유 공간이 큰 차선 선택
        [~, lane] = max(lane_gaps);


        % ExitRatio를 기반으로 exit_weights 계산 (Exit : Through 비율)
        exit_ratio = Parameter.ExitRatio / 100;  % 0.5 for 50%
        exit_weights = [exit_ratio, 1-exit_ratio];  % [Exit 확률, Through 확률]
        exit = randsample(Parameter.Map.Exit, 1, true, exit_weights);
        politeness = 1;
        spawnPos = 1;
        newVeh = [vehID; lane; exit; politeness; spawnPos; Time];

        if vehID > length(List.Vehicle.Object)
            List.Vehicle.Object = cat(1, List.Vehicle.Object, cell(100, 1));
        end

        List.Vehicle.Object{vehID} = Vehicle(newVeh(1:6), Time, Parameter);

        TotalVehicles = TotalVehicles + 1;
        InVehBuffer = InVehBuffer - 1;
    end
end