function P = GetParameters(Setting)
    
    P.ExitThreshold = 20;
    P.Physics = 0.1; % 한 시뮬레이션 타임스텝이 0.1초초
    P.Control = 0.1;
    P.Label = 1; % 1: label 보임, 0: 안보임
    P.ShowTraj = true;
    P.RemoveTraj = true;  % 궤적 유지 여부 설정 (기본값: false)
    P.Flow = 6000; % veh/hour, only used in spawntype = 1.
    
    % Sim
    P.Sim.Time = Setting.Time;
    P.Sim.Data = 5;
        % ID
        % State
        % Lane
        % Location
        % Velocity

    % Map
    P.Map.Color.Road = '#CDCDCD';
    P.Map.Color.Grass = '#B5E3AB';

    P.Map.Scale = 0.01;

    P.Map.Tile = 3.05; %m
    P.Map.Road = 1200; %400
    %P.Map.Road = randi([400,2500]);
    %P.Map.Road = 2005; %m
    
    %minLanes = 2;
    %maxLanes = 8+1;
    %P.Map.Lane = minLanes + floor((P.Map.Road - 400) / (2500 - 400) * (maxLanes - minLanes));
    %P.Map.Lane = max(minLanes, min(P.Map.Lane, maxLanes));  % 범위 제한
    P.Map.Lane = 3;
    

    P.Map.Margin = 5;
    P.Map.Stop = 6;
    P.Map.Center = [0;0];
    P.Map.SpawnZone = 520; % new vehicle to be spawned at anywhere between [0,200m]
    %P.Map.Exit = [1040, 1970];
    P.Map.Exit = [600,5000];
    %P.Map.Exit = RandomExitGenerator(P.Map.Road);
    %P.Map.Exit = [P.Map.Road-330, P.Map.Road-30];
    %P.Map.Exit = [1200,2000, 2805];
    P.Map.GrapeThreshold = 200;

    % Vehicle
    P.Veh.MaxVel = 33; % Original: 10
    P.Veh.DecVel = 1; % 교차로에서만 쓰이는 값
    P.Veh.MinVel = 0; % 8
    P.Veh.Accel = [3 7]; % [1.5 3]은 -3부터 1.5까지를 의미함.[a b]-a: max accel. b: max decel.
    P.Veh.Size = [4.5 1.9 1.2];
    P.Veh.Buffer = [2.5 0.5];
        P.Veh.State.Out = 0;
        P.Veh.State.Rejected = 1;
        P.Veh.State.Reserved = 2;
        P.Veh.State.Signal = 3;
    P.Veh.Safety = 2; % Original: 2 , 정적 안전 거리: 속도와 무관하게 항상 유지할 기본 간격
    P.Veh.Headway = 1.5; % Original: 1.6, 동적 반응 거리: 시간 기반 간격 (속도가 높을수록 더 멀리 떨어지도록 유도)
    P.Veh.Exp = 4; %4
    P.Veh.SafeDistance = 4; % Lane Change feasibility 판별/ GetAccel.m에 추가
    P.Veh.AccelThreshold = -20; % MOBIL에서만 사용


    % Task Demand 관련 파라미터
    P.TaskDemandCrowdedRange = 4; % Task demand 계산 시 혼잡하다고 판단하는 범위 (m)
    P.TaskDemandCrowdedPenalty = 0.001; % 혼잡한 차선에 대한 패널티 (0에 가까울수록 더 큰 패널티)

    % Signal
    P.Sig = 0;

    P.Map.Size = (P.Map.Stop + P.Map.Lane*P.Map.Tile + P.Map.Road)*2;
    
end

