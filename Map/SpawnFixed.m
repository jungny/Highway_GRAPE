function [List, TotalVehicles, SpawnVehicle, firstCount] = ...
    SpawnFixed(List, Simulation, Parameter, Time, SpawnVehicle, firstCount)

if firstCount == 0
    [SpawnVehicle, TotalVehicles] = GetSeed(Simulation.Setting, Parameter);
    List.Vehicle.Object = cell(size(SpawnVehicle, 2), 1);
    firstCount = 1;
else
    TotalVehicles = [];
end

while ~isempty(SpawnVehicle) && int32(Time / Parameter.Physics) == int32(SpawnVehicle(6,1) / Parameter.Physics)
    id = SpawnVehicle(1,1);
    List.Vehicle.Object{id} = Vehicle(SpawnVehicle(:,1), Time, Parameter);

    if ~isempty(SpawnVehicle)
        SpawnVehicle = SpawnVehicle(:, 2:end);
    end
    
end

end
