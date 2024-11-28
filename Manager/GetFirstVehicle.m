function FirstVehicles = GetFirstVehicle(VehicleList,SignalList)

    FirstVehicles = zeros(4,1);

    ObjectData = [SignalList;VehicleList];
    ObjectData(ObjectData(:,3) == 0,:) = [];
    
    for i = 1:4
        LaneData = ObjectData(ObjectData(:,3)==i,:);
        LaneData = sortrows(LaneData,4,"descend");

        LaneData(LaneData(:,4) >= ObjectData(1,4),:) = [];
        if ~isempty(LaneData)
            FirstVehicles(i) = LaneData(1,1);
        end
    end

    FirstVehicles(FirstVehicles == 0,:) = [];

end

