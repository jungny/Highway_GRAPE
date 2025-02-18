function ObjectList = GetAcceleration(ObjectList, VehicleList, Parameter)

    % ObjectData는 VehicleList만 사용 (Signal 관련 제거)
    ObjectData = VehicleList;

    % 차선 개수를 동적으로 설정
    MaxLane = max(ObjectData(:,3)); % VehicleList의 3번째 열이 차선 번호
    for i = 1:MaxLane
        % 현재 차선(i)에 있는 차량 데이터를 필터링
        LaneData = ObjectData(ObjectData(:,3) == i, :);
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

                    % 🚦 MLC 차량 감속 적용
                    % if ObjectList{LaneData(j, 1)}.MLC_flag
                    %     Acceleration = max(Acceleration - Parameter.Accel(2), - Parameter.Accel(2)); % 감속 적용
                    % end
                    
                    if ~isreal(Acceleration) || isnan(Acceleration) || isinf(Acceleration)
                        % warning('Invalid acceleration detected. Setting acceleration to 0.');
                        Acceleration = 0;
                    end
                                
                    % ObjectList에 가속도 저장
                    ObjectList{LaneData(j, 1)}.Acceleration = Acceleration;
                end
            end

            for j = 1
                
                Acceleration = Parameter.Accel(1) * (1 - (VelocitySelf / Parameter.MaxVel)^Parameter.Exp);

                if ~isreal(Acceleration) || isnan(Acceleration) || isinf(Acceleration)
                    % warning('Invalid acceleration detected. Setting acceleration to 0.');
                    Acceleration = 0;
                end

                ObjectList{LaneData(j, 1)}.Acceleration = Acceleration;
            end
        elseif size(LaneData,1)==1
            VelocitySelf = LaneData(1, 5);
            Acceleration = Parameter.Accel(1) * (1 - (VelocitySelf / Parameter.MaxVel)^Parameter.Exp);
            if ~isreal(Acceleration) || isnan(Acceleration) || isinf(Acceleration)
                % warning('Invalid acceleration detected. Setting acceleration to 0.');
                Acceleration = 0;
            end
            
            ObjectList{LaneData(1, 1)}.Acceleration = Acceleration;    
        end
    end

end
