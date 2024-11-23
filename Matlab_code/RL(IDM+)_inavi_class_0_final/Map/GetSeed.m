function Seed = GetSeed(Settings,Parameter,Iteration)
    
    TotalVehicles = Settings.Vehicles;
    Seed = zeros(5,TotalVehicles);
    % Vehicle ID
    % Spawn Time
    % Spawn Lane
    % Direction: 1:Through 2:Left 3:Right
    % Agent: 1:Agent 0:Environment
    if Settings.Mode == 1
        Seed(1,:) = 1:TotalVehicles;        
        Seed(2,:) = [1.8 1.3 1]%((randperm(21,TotalVehicles)-1)*Parameter.Physics);            
        Seed(3,:) = [1,2,3]%randi([1,4],[1,TotalVehicles]);    
        if Settings.Iterations(2,Iteration) == 1
            Seed(4,:) = [Settings.Iterations(3,Iteration) randi([1,3],[1,TotalVehicles-Settings.Iterations(2,Iteration)])];
        else
            Seed(4,:) = [1 1 2]%randi([1,3],[1,TotalVehicles]);
        end
        Seed(5,:) = [ones(1,Settings.Iterations(2,Iteration)) zeros(1,TotalVehicles-Settings.Iterations(2,Iteration))]; 
        Seed = sortrows(Seed',2)';
    else
        Seed = zeros(4,TotalVehicles);
        Seed(1,:) = 1:TotalVehicles;        
        Seed(2,:) = (randperm(Settings.Time/Parameter.Physics,TotalVehicles)*Parameter.Physics);              
        Seed(3,:) = randi([1,4],[1,TotalVehicles]);                
        Seed(4,:) = randi([1,3],[1,TotalVehicles]); 
        Seed(5,:) = randi([0,1],[1,TotalVehicles]);  
    end
end

