
function [SpawnSeed, NewList] = GetSeed(Settings, Parameter, TotalVehicles, SpawnLanes, OldList)
    
    % 1: Vehicle ID
    % 2: Spawn Lane
    % 3: Exit
    % 4: Politeness Factor
    % 5: Spawn Position
    SpawnCount = length(SpawnLanes);

    SpawnSeed = zeros(5,SpawnCount);
    SpawnSeed(1,:) = TotalVehicles + 1 : TotalVehicles + SpawnCount;
    SpawnSeed(2,:) = SpawnLanes;

    NumExits = length(Parameter.Map.Exit);
    SpawnSeed(3, :) = Parameter.Map.Exit(randi(NumExits, 1, SpawnCount));
    SpawnSeed(4,:) = ones(1,SpawnCount);

    % Poisson Process 기반 다음 차량 도착 시간 업데이트
    lambda = Parameter.Flow / 3600; % [veh/hour/lane] -> [veh/sec/lane]
    for i = 1:length(SpawnLanes)
        OldList(SpawnLanes(i)) = OldList(SpawnLanes(i)) - log(rand) / lambda;
    end
    NewList = OldList;

    SpawnSeed(5,:) = 1+0*Parameter.Map.SpawnZone * rand(1, SpawnCount);
end

%{

function Seed = GetSeed(Settings,Parameter,Iteration)
    
    TotalVehicles = Settings.Vehicles;
    Seed = zeros(7,TotalVehicles);
    % Vehicle ID
    % Spawn Time
    % Spawn Lane
    % Direction: 1:Through 2:Left 3:Right
    % Agent: 1:Agent 0:Environment
    % Exit
    % Politeness Factor

    if Settings.Mode == 4

        % Flow (veh/hour/lane) 기반으로 차량 생성 간격 설정
        FlowRate = Parameter.Flow / 3600; % [veh/hour/lane] -> [veh/sec/lane]
        SimTime = Settings.Time; % [sec]
        percentContinuing = 1; % percentage of new cars on a road continuing. 추후 0.2로 변경 예정.
        
        dt = Parameter.Physics; % Time Step
        TotalSteps = round(SimTime / dt); % 총 타임스텝 개수
        numNewCars = round(dt * FlowRate * Parameter.Map.Lane); % 각 타임스텝에서 생성되는 차량 수
        TotalVehicles = numNewCars * TotalSteps; % 전체 시뮬레이션 동안 생성될 총 차량 수


        Seed = zeros(9,TotalVehicles);
        % 1: Vehicle ID
        % 2: Spawn Time
        % 3: Spawn Lane
        % 4: Direction: 1:Through 2:Left 3:Right
        % 5: Agent: 1:Agent 0:Environment
        % 6: Exit
        % 7: Politeness Factor
        % 8: Initial Position (도로 전체에 랜덤 분포포)
        % 9: Initial Velocity


        Seed(1,:) = 1:TotalVehicles;

        SpawnTimes = [];
        for t = 1:TotalSteps
            % `dt * t`를 사용하여 전체 시뮬레이션 동안 차량을 생성
            SpawnTimes = [SpawnTimes, repmat(dt * (t-1), 1, numNewCars)];
        end
        Seed(2, :) = SpawnTimes;

        Seed(3, :) = randi([1, Parameter.Map.Lane], [1, TotalVehicles]);

        Seed(4, :) = ones(1, TotalVehicles);

        Seed(5, :) = ones(1, TotalVehicles);

        Seed(6, :) = Parameter.Map.Exit(randi(length(Parameter.Map.Exit), 1, TotalVehicles));

        Seed(7, :) = ones(1, TotalVehicles);


        % Parameter.Map.Road = 9000; %[m]
        Seed(8, :) = Parameter.Map.Road * rand(1, length(SpawnTimes)); % 도로 전체에 랜덤 분포

        v_min = Parameter.Veh.MinVel; % 최소 속도
        v_max = Parameter.Veh.MaxVel; % 최대 속도
        Seed(9, :) = v_min + (v_max - v_min) * rand(1, TotalVehicles);

        Seed = sortrows(Seed', 2)';
        


    elseif Settings.Mode == 1
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
        % 1: Vehicle ID
        % 2: Spawn Time
        % 3:  Spawn Lane
        % 4: Direction: 1:Through 2:Left 3:Right
        % 5: Agent: 1:Agent 0:Environment
        % 6: Exit
        % 7: Politeness Factor   
        
       
        TotalVehicles = randi([2,50]);
        %TotalVehicles = 30;
        Seed = zeros(7,TotalVehicles);

        % Vehicle ID
        Seed(1,:) = 1:TotalVehicles;
        
        % Spawn Time: 일정 간격(0.5초 ~ 2초) + 약간의 랜덤 오프셋
        min_interval = 0.5;
        max_interval = 2;
        SpawnTimes = cumsum(min_interval + (max_interval - min_interval) * rand(1, TotalVehicles));
        Seed(2, :) = SpawnTimes;

        % Spawn Lane: 1 ~ 총 차선 수 사이에서 랜덤하게 설정
        Seed(3, :) = randi([1, Parameter.Map.Lane], [1, TotalVehicles]);

        % Direction: 모두 직진(1)으로 설정
        Seed(4, :) = ones(1, TotalVehicles);

        % Agent 여부: 모두 agent(1)
        Seed(5, :) = ones(1, TotalVehicles);

        % Exit: Parameter의 Map.Exit 중 하나를 무작위로 선택
        NumExits = length(Parameter.Map.Exit);
        Seed(6, :) = Parameter.Map.Exit(randi(NumExits, 1, TotalVehicles));

        % Politeness Factor: 0으로 설정 (selfish)
        Seed(7, :) = zeros(1, TotalVehicles);

        % Spawn Time 기준으로 정렬
        Seed = sortrows(Seed', 2)';


    else 
        Seed = zeros(4,TotalVehicles);
        Seed(1,:) = 1:TotalVehicles;        
        Seed(2,:) = (randperm(Settings.Time/Parameter.Physics,TotalVehicles)*Parameter.Physics);              
        Seed(3,:) = randi([1,4],[1,TotalVehicles]);                
        Seed(4,:) = randi([1,3],[1,TotalVehicles]); 
        Seed(5,:) = randi([0,1],[1,TotalVehicles]);  
                % Vehicle ID
                Seed(1,:) = 1:TotalVehicles;
        
                % Spawn Time: (  )초 간격으로 설정
                Seed(2,:) = [0, 1.5, 1, 3, 5, 4.3, 7, 8, 8.2, 9];
                %Seed(2,:) = ((randperm(21,TotalVehicles)-1)*Parameter.Physics);
        
                % Spawn Lane: 두 차량 모두 1차선
                % Seed(3,:) = [1, 1];
                % Seed(3,:) = [1, 1, 1, 2, 2, 2];
                %Seed(3,:) = randi([1,Parameter.Map.Lane],[1,TotalVehicles]);
                % good example Seed(3,:) = [2,3,2,1,2,1,3,3,2,3];
                Seed(3,:) = [2,1,2,1,1,3,1,3,2,3];
        
                % Direction: 두 차량 모두 직진(1)
                Seed(4,:) = ones(1,TotalVehicles);
        
                % Agent 여부: 모두 agent(1)
                Seed(5,:) = ones(1,TotalVehicles);
        
                % Exit: Parameter의 Map.Exit 중 하나를 부여
                %Seed(6,:) = Parameter.Map.Exit(randi(length(Parameter.Map.Exit), 1, TotalVehicles));
                e1 = Parameter.Map.Exit(1);
                e2 = Parameter.Map.Exit(2);
                Seed(6,:) = [e2, e1, e1, e2, e1, e1, e2, e1, e2, e2];
        
                % Politeness Factor: degree of altruism
                % 0: selfish lane-hoppers
                Seed(7,:) = 0*ones(1,TotalVehicles);
        
                Seed = sortrows(Seed',2)';       
    end
end

%}