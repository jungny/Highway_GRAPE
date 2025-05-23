function P = GetParameters(Setting)
    
    P.ExitThreshold = 20;
    P.Physics = 0.1; % 한 시뮬레이션 타임스텝이 0.1초초
    P.Control = 0.1;
    P.Label = 1; % 1: label 보임, 0: 안보임
    P.ShowTraj = true;
    P.RemoveTraj = true;  % 궤적 유지 여부 설정 (기본값: false)
    P.Flow = 2100; % veh/hour/lane, only used in spawntype = 1.
    
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
    P.Map.Road = 520; %400
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
    P.Map.Exit = [500,5000];
    %P.Map.Exit = RandomExitGenerator(P.Map.Road);
    %P.Map.Exit = [P.Map.Road-330, P.Map.Road-30];
    %P.Map.Exit = [1200,2000, 2805];
    P.Map.GrapeThreshold = 200;

    % Vehicle
    P.Veh.MaxVel = 33; % Original: 10
    P.Veh.DecVel = 1;
    P.Veh.MinVel = 0; % 8
    P.Veh.Accel = [3 3]; % [1.5 3]은 -3부터 1.5까지를 의미함.
    P.Veh.Size = [4.5 1.9 1.2];
    P.Veh.Buffer = [2.5 0.5];
        P.Veh.State.Out = 0;
        P.Veh.State.Rejected = 1;
        P.Veh.State.Reserved = 2;
        P.Veh.State.Signal = 3;
    P.Veh.Safety = 2; % Original: 2 , 정적 안전 거리: 속도와 무관하게 항상 유지할 기본 간격
    P.Veh.Headway = 1.5; % Original: 1.6, 동적 반응 거리: 시간 기반 간격 (속도가 높을수록 더 멀리 떨어지도록 유도)
    P.Veh.Exp = 4; %4
    P.Veh.SafeDistance = 4; % Lane Change feasibility 판별
    P.Veh.AccelThreshold = -20; % MOBIL에서만 사용

    % Signal
    P.Sig = 0;

    P.Map.Size = (P.Map.Stop + P.Map.Lane*P.Map.Tile + P.Map.Road)*2;
    
end

function ExitList = RandomExitGenerator(Road)

    % 도로 길이
    RoadLength = Road;

    % 시작점에서 최소 거리와 마지막 출구 위치 설정
    minExitDistance = 100;             % 시작점에서 최소 100m
    maxExitDistance = RoadLength - 50; % 도로 끝에서 최대 50m 이내
    minGap = 180;                      % 초기 최소 간격 설정

    % 출구 개수 랜덤 설정 (1 ~ 5개)
    numExits = randi([2, max(3, floor(RoadLength / (minGap + 50)))]);

    maxConsecutiveFails = 5000;  % 최대 연속 실패 횟수
    consecutiveFails = 0;     % 현재 연속 실패 횟수
    
    ExitList = [];  % 출구 위치 리스트 초기화

    % 마지막 출구는 도로 끝에서 10m 이내에 배치
    lastExit = maxExitDistance + rand * 10;
    ExitList = [ExitList, lastExit];

    % 마지막 출구 제외하고 랜덤하게 배치
    while length(ExitList) <= numExits
        newExit = minExitDistance + rand * (maxExitDistance - minExitDistance);
        
        % 기존 출구들과 최소 간격을 만족하는 경우 추가
        if isempty(ExitList) || all(abs(ExitList - newExit) >= minGap)
            ExitList = [ExitList, newExit];
            consecutiveFails = 0;  % 성공했으므로 실패 횟수 초기화
        else
            consecutiveFails = consecutiveFails + 1;  % 실패 횟수 증가
        end
        
        % 연속 실패 횟수가 최대치를 넘으면 포기
        if consecutiveFails >= maxConsecutiveFails
            warning('Too many consecutive fails. Returning current ExitList.');
            break;
        end
    end

    

    % 출구 위치 정렬
    ExitList = sort(ExitList);
end
