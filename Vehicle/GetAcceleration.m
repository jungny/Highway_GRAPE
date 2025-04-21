function ObjectList = GetAcceleration(ObjectList, VehicleList, Parameter)

    % ObjectData는 VehicleList만 사용 (Signal 관련 제거)
    ObjectData = VehicleList;


    % 차선 개수를 동적으로 설정
    MaxLane = max(ObjectData(:,3)); % VehicleList의 3번째 열이 차선 번호
    for idx = 1:size(ObjectData,1)
        if ObjectData(idx,1) > 0 
            ObjectList{ObjectData(idx,1)}.Acceleration = [];  % 가속도 배열 초기화
        end
    end
    for i = 1:MaxLane
        % 현재 차선(i)에 있는 차량 데이터를 필터링
        %LaneData = ObjectData(ObjectData(:,3) == i, :);
        % laneRelevantIdx = arrayfun(@(idx) ...
        %     ObjectData(idx,1) > 0 && ( ...
        %         (ObjectList{ObjectData(idx,1)}.IsChangingLane && ...
        %         ObjectList{ObjectData(idx,1)}.TargetLane == i) || ...
        %         (ObjectList{ObjectData(idx,1)}.IsChangingLane && ...
        %         ObjectList{ObjectData(idx,1)}.Lane == i) || ...
        %         (~ObjectList{ObjectData(idx,1)}.IsChangingLane && ...
        %         ObjectList{ObjectData(idx,1)}.Lane == i) ...
        %     ), ...
        %     1:size(ObjectData,1))';
        laneRelevantIdx = arrayfun(@(idx) ...
            ObjectData(idx,1) > 0 && ( ...
                (~isempty(ObjectList{ObjectData(idx,1)}.LaneIfFullyInside) && ...
                 ObjectList{ObjectData(idx,1)}.LaneIfFullyInside == i) || ...
                (isempty(ObjectList{ObjectData(idx,1)}.LaneIfFullyInside) && ...
                 ObjectList{ObjectData(idx,1)}.IsChangingLane && ...
                 (ObjectList{ObjectData(idx,1)}.Lane == i || ...
                  ObjectList{ObjectData(idx,1)}.TargetLane == i)) || ...
                (~ObjectList{ObjectData(idx,1)}.IsChangingLane && ...
                 ObjectList{ObjectData(idx,1)}.Lane == i) ...
            ), ...
        1:size(ObjectData,1))';

        LaneData = ObjectData(laneRelevantIdx, :);


        LaneData = sortrows(LaneData, 4, "descend"); % 위치 기준 내림차순 정렬

        % 차선에 차량이 여러 대 있는 경우만 처리
        if size(LaneData, 1) > 1
            for j = 2:size(LaneData, 1) % 두 번째 차량부터 처리
                if LaneData(j, 1) > 0
                    % 선두 차량 데이터를 사용해 계산
                    LeadVehicle = LaneData(j-1, :);
                    VelocitySelf = LaneData(j, 5);
                    VelocityLead = LeadVehicle(1, 5);
                    VelocityDifference = VelocityLead - VelocitySelf;
                    LocationDifference = (LeadVehicle(1, 4) - LaneData(j, 4)) / ObjectList{LaneData(j, 1)}.DistanceStep - Parameter.Size(1);
                    DesiredDistance = Parameter.Safety + VelocitySelf * Parameter.Headway - ...
                        (VelocitySelf * VelocityDifference) / (2 * sqrt(Parameter.Accel(1) * Parameter.Accel(2)));

                    % 가속도 계산
                    Acceleration = Parameter.Accel(1) * (1 - (VelocitySelf / Parameter.MaxVel)^Parameter.Exp -...
                                   (DesiredDistance / LocationDifference)^2);

                    Acceleration = max(min(Acceleration, Parameter.Accel(1)), -Parameter.Accel(2));

                    
                    if ~isreal(Acceleration) || isnan(Acceleration) || isinf(Acceleration)
                        % warning('Invalid acceleration detected. Setting acceleration to 0.');
                        Acceleration = 0;
                    end
                                
                    % ObjectList에 가속도 저장
                    if ObjectList{LaneData(j,1)}.IsChangingLane
                        ObjectList{LaneData(j, 1)}.Acceleration = [ObjectList{LaneData(j, 1)}.Acceleration, Acceleration];
                    else
                        ObjectList{LaneData(j, 1)}.Acceleration = Acceleration;
                    end
                end
            end

            for j = 1
                
                Acceleration = Parameter.Accel(1) * (1 - (VelocitySelf / Parameter.MaxVel)^Parameter.Exp);
                Acceleration = max(min(Acceleration, Parameter.Accel(1)), -Parameter.Accel(2));


                if ~isreal(Acceleration) || isnan(Acceleration) || isinf(Acceleration)
                    % warning('Invalid acceleration detected. Setting acceleration to 0.');
                    Acceleration = 0;
                end

                % ObjectList에 가속도 저장
                if ObjectList{LaneData(j,1)}.IsChangingLane
                    ObjectList{LaneData(j, 1)}.Acceleration = [ObjectList{LaneData(j, 1)}.Acceleration, Acceleration];
                else
                    ObjectList{LaneData(j, 1)}.Acceleration = Acceleration;
                end
            end
        elseif size(LaneData,1)==1
            VelocitySelf = LaneData(1, 5);
            Acceleration = Parameter.Accel(1) * (1 - (VelocitySelf / Parameter.MaxVel)^Parameter.Exp);
            Acceleration = max(min(Acceleration, Parameter.Accel(1)), -Parameter.Accel(2));

            if ~isreal(Acceleration) || isnan(Acceleration) || isinf(Acceleration)
                % warning('Invalid acceleration detected. Setting acceleration to 0.');
                Acceleration = 0;
            end
            
            % ObjectList에 가속도 저장
            if ObjectList{LaneData(1,1)}.IsChangingLane
                ObjectList{LaneData(1, 1)}.Acceleration = [ObjectList{LaneData(1, 1)}.Acceleration, Acceleration];
            else
                ObjectList{LaneData(1, 1)}.Acceleration = Acceleration;
            end  
        end
    end

    % 모든 차선 계산이 끝난 후, 차선 변경 중인 차량들의 최종 가속도 결정
    for idx = 1:size(ObjectData,1)
        if ObjectData(idx,1) > 0 && ObjectList{ObjectData(idx,1)}.IsChangingLane
            % 저장된 모든 가속도 중 최소값 선택
            ObjectList{ObjectData(idx,1)}.Acceleration = min(ObjectList{ObjectData(idx,1)}.Acceleration);
        end
    end

end
