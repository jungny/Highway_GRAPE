function ObjectList = GetAcceleration(ObjectList,VehicleList,SignalList,Parameter)
    
    ObjectData = [SignalList;VehicleList];
    
    for i = 1:4
        LaneData = ObjectData(ObjectData(:,3)==i,:);
        LaneData = sortrows(LaneData,4,"descend");

        if size(LaneData,1) > 1
            for j = 2:size(LaneData,1)
                if LaneData(j,1) > 0
                    if LaneData(j,2) == 1
                        LeadVehicle = LaneData(j-1,:);
                        VelocitySelf = LaneData(j,5);
                        VelocityLead = LeadVehicle(1,5);
                        VelocityDifference = VelocityLead - VelocitySelf;
                        LocationDifference = (LeadVehicle(1,4) - LaneData(j,4))/ObjectList{LaneData(j,1)}.DistanceStep  - Parameter.Size(1);
                        DesiredDistance = Parameter.Safety + VelocitySelf*Parameter.Headway - ...
                            (VelocitySelf*VelocityDifference)/(2*sqrt(Parameter.Accel(1)*Parameter.Accel(2)));
    
                        Acceleration = Parameter.Accel(1)*(1-(VelocitySelf/Parameter.MaxVel)^Parameter.Exp - (DesiredDistance/LocationDifference)^2);
    
                        ObjectList{LaneData(j,1)}.Acceleration = Acceleration;
                    elseif LaneData(j,2) == 2
                        ObjectList{LaneData(j,1)}.Acceleration = Parameter.Accel(1);
                    end

                end
            end
        end
    end

end

