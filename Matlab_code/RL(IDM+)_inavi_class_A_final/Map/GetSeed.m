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
        Seed(2,:) = ((randperm(21,TotalVehicles)-1)*Parameter.Physics);            
        Seed(3,:) = [1,2,3]; %randi([1,4],[1,TotalVehicles]);    
        if Settings.Iterations(2,Iteration) == 1
            Seed(4,:) = [Settings.Iterations(3,Iteration) randi([1,3],[1,TotalVehicles-Settings.Iterations(2,Iteration)])];
        else
            Seed(4,:) = [1 1 2];%randi([1,3],[1,TotalVehicles]);
        end
        Seed(5,:) = [ones(1,Settings.Iterations(2,Iteration)) zeros(1,TotalVehicles-Settings.Iterations(2,Iteration))]; 
        Seed = sortrows(Seed',2)';

    elseif Settings.Mode == 3 %Simple Highway example        
        % Vehicle ID
        Seed(1,:) = 1:TotalVehicles;
        
        % Spawn Time: (  )초 간격으로 설정
        Seed(2,:) = [0, 1.5];

        % Spawn Lane: 두 차량 모두 1차선
        Seed(3,:) = [1, 1];

        % Direction: 두 차량 모두 직진(1)
        Seed(4,:) = [1, 1];

        % Agent 여부: 모두 agent(1)
        Seed(5,:) = [1, 1];

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

