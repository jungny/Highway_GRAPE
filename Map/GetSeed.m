function Seed = GetSeed(Settings,Parameter,Iteration)
    
    TotalVehicles = Settings.Vehicles;
    Seed = zeros(6,TotalVehicles);
    % Vehicle ID
    % Spawn Time
    % Spawn Lane
    % Direction: 1:Through 2:Left 3:Right
    % Agent: 1:Agent 0:Environment
    % Exit
    % ExitState
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
        Seed(2,:) = [0, 1.5, 3.5, 10, 11.5, 13];
        %Seed(2,:) = ((randperm(21,TotalVehicles)-1)*Parameter.Physics);

        % Spawn Lane: 두 차량 모두 1차선
        % Seed(3,:) = [1, 1];
        % Seed(3,:) = [1, 1, 1, 2, 2, 2];
        Seed(3,:) = randi([1,Parameter.Map.Lane],[1,TotalVehicles]);
        

        % Direction: 두 차량 모두 직진(1)
        Seed(4,:) = ones(1,TotalVehicles);

        % Agent 여부: 모두 agent(1)
        Seed(5,:) = ones(1,TotalVehicles);

        % Exit: Parameter의 Map.Exit 중 하나를 부여
        Seed(6,:) = Parameter.Map.Exit(randi(length(Parameter.Map.Exit), 1, TotalVehicles));
        Seed(6,:) = [380, 240, 380, 240, 240, 380];

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

