function run_single_simulation(config)
% === 기본 설정 ===
Simulation.Setting = struct();
Simulation.Setting.GRAPEmode = config.GRAPEmode;
Simulation.Setting.SaveFolder = 'C:\Users\nana\Desktop\250528_0612';
Simulation.Setting.RecordExcel = config.RecordExcel;
Simulation.Setting.RecordLog = 0;
Simulation.Setting.RecordVideo = config.RecordVideo;
Simulation.Setting.VideoSpeedMultiplier = 5;

% === 메모/비디오이름 설정 ===
memo = sprintf('%d_', config.ID);
videomemo = memo;
if config.GRAPEmode == 0
    memo = [memo ' | GRAPE'];
    videomemo = [videomemo '_GRAPE'];
elseif config.GRAPEmode == 2
    memo = [memo ' | CycleGreedy'];
    videomemo = [videomemo '_CycleGreedy'];
end

ExitRatio = config.ExitRate;
memo = [memo sprintf(' | Exit : Through = %d : %d', ExitRatio/10, 10 - ExitRatio/10)];
videomemo = [videomemo sprintf('_%d%%_', ExitRatio)];

Simulation.Setting.k = config.k;
Simulation.Setting.kList = config.k;
Simulation.Setting.k_Mode = config.k_Mode;
Simulation.Setting.BubbleRadiusList = [];

if strcmpi(config.Strategy, "Bubble") || strcmpi(config.Strategy, "BubbleAhead")
    Simulation.Setting.BubbleRadiusList = config.BubbleRadius;
end

Simulation.Setting.Window = 1000;
Simulation.Setting.Draw = 0;
Simulation.Setting.StopOnGrapeError = 1;
Simulation.Setting.PauseTime = 0;
Simulation.Setting.InitialRandomSeed = config.InitialRandomSeed;
Simulation.Setting.Iterations = config.Iterations;
cycle_GRAPE = 5; % GRAPE instance per 5 seconds
Simulation.Setting.SpawnMode = 'auto';
Simulation.Setting.FixedSpawnType = 1;
Simulation.Setting.GreedyAlloc = 0;
Simulation.Setting.Util_type = 'GS';
Simulation.Setting.LaneChangeMode = 'SimpleLaneChange';

switch Simulation.Setting.SpawnMode
    case 'fixed'
        Simulation.Setting.Time = 10000;
    case 'auto'
        Simulation.Setting.WarmupTime = 45;
        Simulation.Setting.SimulationTime = 300;
        Simulation.Setting.Time = Simulation.Setting.WarmupTime + Simulation.Setting.SimulationTime;
end

% 이제 여기부터 너의 main.m 본문 붙이면 됨 (위 설정만 외부에서 받고 내부는 동일)
% 단 memo, videomemo, ExitRatio, Simulation.Setting 전역처럼 쓸 수 있음

% 예시:
disp("🚀 Running config ID " + config.ID + " | GRAPEmode = " + config.GRAPEmode);


Simulation.Setting.FixedSpawnType = 1; 
Simulation.Setting.GreedyAlloc = 0; % 0: Distributed Mutex is applied (GRAPE), 1: Agents make fully greedy decisions (Baseline)

Simulation.Setting.Util_type = 'GS'; 
%Simulation.Setting.Util_type = 'HOS'; 
%Simulation.Setting.Util_type = 'FOS'; 
%Simulation.Setting.Util_type = 'ES'; 
Simulation.Setting.LaneChangeMode = 'SimpleLaneChange'; % 'MOBIL' or 'SimpleLaneChange'


%% Run Simulation
% Initialize Log File
if Simulation.Setting.RecordLog
    finalRandomSeed = Simulation.Setting.InitialRandomSeed + Simulation.Setting.Iterations - 1;
    %logFileName = Simulation.Setting.LogPath(finalRandomSeed);
    logFileName = fullfile(Simulation.Setting.SaveFolder, ...
        [videomemo '_log.txt']);

    % 새 로그 파일 생성 및 헤더 작성
    fileID = fopen(logFileName, 'w');
    fprintf(fileID, 'Simulation Log\n');
    fclose(fileID);
end

% GreedyAlloc 여부를 아이콘으로 변환
% if Simulation.Setting.GreedyAlloc == 1
%     greedy_status = 'GRAPE ❌';
%     greedy_status2 = 'Greedy';
% else
%     greedy_status = 'GRAPE ⭕';
%     greedy_status2 = 'GRAPE';
% end

% 🔹 엑셀 파일 경로 설정
timestamp = string(datetime('now', 'Format', 'HH-mm'));
ExcelSaveFolder = 'C:\Users\nana\Desktop\ExcelRecord';
filename = fullfile(ExcelSaveFolder, [videomemo '.xlsx']);

% 🔹 실험할 참가자 모드 설정
switch config.Strategy
    case "Default"
        participantModes = {"Default"};
    case "Ahead"
        participantModes = {"Ahead"};
    case "Bubble"
        participantModes = {sprintf('Bubble_%dm', config.BubbleRadius)};
    case "BubbleAhead"
        participantModes = {sprintf('BubbleAhead_%dm', config.BubbleRadius)};
    otherwise
        error("Unknown Strategy: %s", config.Strategy);
end


% 🔹 kList가 여러 개인 경우 participantModes는 하나만 사용
if length(Simulation.Setting.kList) > 1
    if length(participantModes) > 1
        warning('Multiple k values detected. Using only first participant mode.');
        participantModes = participantModes(1);
    end
    num_modes = length(Simulation.Setting.kList);
    mode_names = cell(1, num_modes);
    for i = 1:num_modes
        mode_names{i} = sprintf('k_%.1f', Simulation.Setting.kList(i));
    end
else
    num_modes = length(participantModes);
    mode_names = participantModes;
end

% 🔹 결과 저장을 위한 배열 초기화
if Simulation.Setting.RecordExcel
    num_simulations = Simulation.Setting.Iterations;
    % 각 mode별로 결과를 저장할 셀 배열 초기화
    results = cell(num_simulations, 9); % random seed, avg speed, std speed, road capacity, exit fail rate, exit avg speed, exit speed std, through avg speed, through speed std
end

for Iteration = 1:Simulation.Setting.Iterations
    randomSeed = Simulation.Setting.InitialRandomSeed + Iteration - 1;
    rng(randomSeed)

    % 현재 random seed에 대한 결과 저장할 행 초기화
    result_row = cell(1, 9);
    result_row{1} = randomSeed;  % 첫 번째 칸에 random seed 저장

    for mode_idx = 1:num_modes
        close all
        rng(randomSeed)
        
        % Set mode based on whether we're using kList or participantModes
        if length(Simulation.Setting.kList) > 1
            Simulation.Setting.k = Simulation.Setting.kList(mode_idx);
            Simulation.Setting.NumberOfParticipants = participantModes{1};
        else
            Simulation.Setting.NumberOfParticipants = participantModes{mode_idx};
        end

        if startsWith(Simulation.Setting.NumberOfParticipants, 'Bubble')
            radius_str = regexp(Simulation.Setting.NumberOfParticipants, '\d+', 'match');  
            if ~isempty(radius_str)
                Simulation.Setting.BubbleRadius = str2double(radius_str{1});  
            else
                error('Bubble mode detected, but no valid radius found in: %s', Simulation.Setting.NumberOfParticipants);
            end
        else
            Simulation.Setting.BubbleRadius = NaN;
        end

        disp(['Running ', mode_names{mode_idx}, ' mode, Random Seed ', num2str(randomSeed)]);

        environment = struct();
        GRAPE_output = [];
        travel_times = [];
        exit_fail = 0;
        RemovedVehicle = 0;
        exit_fail_count = 0;
        exit_success_count = 0;
        TotalVehicles = 0;
        

        Parameter = GetParameters(Simulation.Setting);
        GetWindow(Parameter.Map,Simulation.Setting)
        Parameter.Trajectory = GetTrajectory(Parameter.Map,Simulation.Setting);
        Parameter.ExitRatio = ExitRatio;

        % 차량별 속도를 기록하기 위한 배열 추가
        vehicle_speeds = [];
        exit_vehicle_speeds = [];  % Exit 차량의 속도를 저장할 배열
        through_vehicle_speeds = [];  % Through 차량의 속도를 저장할 배열
        vehicles_passed = 0;  % 도로 용량 계산을 위한 변수
        % 첫 번째 exit의 50m 이전 지점을 capacity check point로 설정
        first_exit_point = min(Parameter.Map.Exit);  % 첫 번째 exit 위치
        capacity_check_point = first_exit_point - 100;  % 첫 번째 exit의 50m 이전 지점
        counted_vehicles = [];  % capacity check point를 통과한 차량 ID를 저장할 배열

        if Simulation.Setting.RecordVideo
            timestamp = string(datetime('now', 'Format', 'HH-mm'));
            % videoFilename = Simulation.Setting.VideoPath(participantModes{mode_idx}, randomSeed, timestamp);
            % videoFilename = filename;
            videoFilename = fullfile(Simulation.Setting.SaveFolder, 'video', [videomemo '_' char(timestamp) '.mp4']);
            videoWriter = VideoWriter(videoFilename, 'MPEG-4'); %#ok<TNMLP>
            videoWriter.FrameRate = 15 * Simulation.Setting.VideoSpeedMultiplier; 
            open(videoWriter);
        end

        fprintf('%d th Iteration. Random Seed is %d.\n', Iteration, randomSeed)

        SetMap(Parameter.Map, Simulation.Setting);


        if Simulation.Setting.RecordLog
            fileID = fopen(logFileName, 'a', 'n', 'utf-8');  % append 모드로 파일 열기
            currentTime = string(datetime('now', 'Format', 'HH시 mm분 ss초'));
            fprintf(fileID, '\n=====   Random Seed  %d  ||  %s   ===== %s \n', ...
                    randomSeed, Simulation.Setting.NumberOfParticipants, currentTime);
            
            % fprintf(fileID, '\n| Lanes:  %d  | Vehicles: %d  | Exits: %d  |\n', ...
            %     Parameter.Map.Lane, TotalVehicles, length(Parameter.Map.Exit));
            fclose(fileID);
        end


        NextArrivalTime = zeros(Parameter.Map.Lane, 1); % 5차선 기준 [0;0;0;0;0]
        %NextArrivalTime = (3600 / Parameter.Flow) * rand(Parameter.Map.Lane, 1);
        %disp(NextArrivalTime);
        firstCount = 0;
        SpawnVehicle = [];
        List = struct();
        InVehBuffer = [];
        RandomValBCup = [];
        vehIDCounter = 0;

        for Time = 0:Parameter.Physics:Parameter.Sim.Time
            GRAPE_done = 0;            
            % 제목 출력
            title(sprintf('k: %.2f | Mode: %s | Inflow: %.1f | Seed: %d | Time: %.2f s', ...
                Simulation.Setting.k, ...
                strrep(Simulation.Setting.NumberOfParticipants, '_', ' '), ...
                Parameter.Flow, ...
                randomSeed, ...
                Time));


            if Simulation.Setting.SpawnMode == "fixed"
                [List, TotalVehicles_, SpawnVehicle, firstCount] = ...
                    SpawnFixed(List, Simulation, Parameter, Time, SpawnVehicle, firstCount);
                if ~isempty(TotalVehicles_ )
                    TotalVehicles = TotalVehicles_;
                end
            elseif Simulation.Setting.SpawnMode == "auto"
                [List, TotalVehicles, InVehBuffer, RandomValBCup, vehIDCounter] = ...
                    SpawnAuto(List, Parameter, Time, TotalVehicles, InVehBuffer, RandomValBCup, vehIDCounter);

            end


            % Update Vehicle Data
            List.Vehicle.Data = UpdateData(List.Vehicle.Object,Parameter.Sim.Data);
            List.Vehicle.Active = List.Vehicle.Data(List.Vehicle.Data(:,2)>0,:);
            List.Vehicle.Object = GetAcceleration(List.Vehicle.Object, List.Vehicle.Data, Parameter.Veh);

            % if Simulation.Setting.GreedyAlloc %&& mod(Time, cycle_GRAPE) == cycle_GRAPE-1
            %     environment = GRAPE_Environment_Generator(List,Parameter,Simulation.Setting);
            %     lane_alloc = GRAPE_instance(environment).Alloc;
            %     GRAPE_done = 1;

            if Simulation.Setting.GRAPEmode == 1 % Greedy (no cycle, at any time step)
                if strcmp(Simulation.Setting.SpawnMode, 'fixed') || Time >= Simulation.Setting.WarmupTime
                    environment = GRAPE_Environment_Initialize(List,Parameter,Simulation.Setting);
                    lane_alloc = GRAPE_instance(environment).Alloc;
                end

            elseif Simulation.Setting.GRAPEmode == 2 ... % CycleGreedy (yes cycle) 
                   && mod(Time, cycle_GRAPE) == cycle_GRAPE-1 && size(List.Vehicle.Active,1)>0
                if strcmp(Simulation.Setting.SpawnMode, 'fixed') || Time >= Simulation.Setting.WarmupTime
                    environment = GRAPE_Environment_Initialize(List,Parameter,Simulation.Setting);
                    lane_alloc = GRAPE_instance(environment).Alloc;
                    GRAPE_done = 1;
                end

            elseif mod(Time, cycle_GRAPE) == cycle_GRAPE-1 && size(List.Vehicle.Active,1)>0
                if strcmp(Simulation.Setting.SpawnMode, 'fixed') || Time >= Simulation.Setting.WarmupTime
                    % GRAPE (yes cycle)
                    disp("calling Grape Instance. . . | "+ Time);
                    environment = GRAPE_Environment_Initialize(List,Parameter,Simulation.Setting);
                    try
                        GRAPE_output = GRAPE_instance(environment);
                        % ex: GRAPE_output.Alloc = [1,2] -> 첫번째 차량은 1차선, 두번째 차량은 2차선 할당
                        lane_alloc = GRAPE_output.Alloc;
                        if any(lane_alloc == 0)
                            fileID = fopen(Simulation.Setting.LogFile, 'a', 'n', 'utf-8');  % append 모드로 파일 열기
                            fprintf(fileID, 'VOID TASK at %d \n', Iteration);
                            fclose(fileID);
                        end
                        GRAPE_done = 1;
                    catch ME
                        if Simulation.Setting.StopOnGrapeError
                            rethrow(ME);
                        else
                            warning(ME.identifier, 'GRAPE error occurred, ignoring and continuing: %s', ME.message);
                        end
                    end
                end
            end
        
            % Move Vehicle
            for i = 1:size(List.Vehicle.Active,1)
                v_id = List.Vehicle.Active(i, 1); 
                current_vehicle = List.Vehicle.Object{v_id};
                current_lane = List.Vehicle.Object{v_id}.Lane;
                
                % 차량이 capacity check point를 통과했는지 확인
                if strcmp(Simulation.Setting.SpawnMode, 'fixed') || Time >= Simulation.Setting.WarmupTime
                    if (strcmp(Simulation.Setting.SpawnMode, 'fixed') || ~isempty(current_vehicle) && ...
                       current_vehicle.SpawnTime >= Simulation.Setting.WarmupTime) && ...
                       current_vehicle.Location * Parameter.Map.Scale >= capacity_check_point && ...
                       ~ismember(v_id, counted_vehicles)
                        vehicles_passed = vehicles_passed + 1;
                        counted_vehicles = [counted_vehicles, v_id]; %#ok<AGROW>
                    end
                end

                if GRAPE_done == 1 || Simulation.Setting.GRAPEmode == 1 && Time >= Simulation.Setting.WarmupTime
                    lane_to_go = lane_alloc(i);
                
                    if current_lane ~= lane_to_go
                        current_vehicle.LaneAlloc = lane_to_go;
                        %List.Vehicle.Object{vehicle_id}.TargetLane = desired_lane;
                        %List.Vehicle.Object{vehicle_id}.LaneChangeFlag = 1; 
                        if abs(current_lane - lane_to_go) > 1
                            fprintf('Warning: Vehicle %d attempted consecutive lane change from lane %d to %d at time %.2f\n', ...
                                v_id, current_lane, lane_to_go, Time);
                        end
                        if current_lane > lane_to_go
                            lane_to_go = current_lane - 1;
                        elseif current_lane < lane_to_go
                            lane_to_go = current_lane + 1;
                        end

                        feasible = true;

                        if feasible %&& Simulation.Setting.GreedyAlloc
                            if current_vehicle.IsChangingLane 
                                current_vehicle.LaneChangeFlag = 0;
                            else
                                current_vehicle.TargetLane = lane_to_go;
                                current_vehicle.LaneChangeFlag = 1;
                            end
                        %elseif feasible
                        %    current_vehicle.TargetLane = desired_lane;
                        %    current_vehicle.LaneChangeFlag = 1;
                        else
                            % current_vehicle.LaneChangeFlag = 0;
                        end
                    end
                end
                
                if List.Vehicle.Object{v_id}.Exit - List.Vehicle.Active(i, 4) * Parameter.Map.Scale <= Parameter.ExitThreshold 
                    if current_lane == Parameter.Map.Lane
                        List.Vehicle.Object{v_id}.ExitState = 1;
                    else
                        List.Vehicle.Object{v_id}.ExitState = 0;
                    end
                end

                MoveVehicle(List.Vehicle.Object{v_id},Time,Parameter,Simulation.Setting)
            end
        
            for i = 1:size(List.Vehicle.Active,1)
                v_id = List.Vehicle.Active(i, 1); 
                % uistack(List.Vehicle.Object{v_id}.Object, 'top');
            end
        
            % Remove Processed Vehicles
            for i = 1:size(List.Vehicle.Active,1)
                v_id = List.Vehicle.Active(i, 1); 
                %if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location >= 300000 % exit으로 바꾸기
                %    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                %    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];

                %    % record travel time, avg speed

                %end


                if List.Vehicle.Active(i,4) * Parameter.Map.Scale > Parameter.Map.Road % through vehicles
                    spawn_time = List.Vehicle.Object{v_id}.SpawnTime;
                    exit_time = Time;
                    travel_time = exit_time - spawn_time;
                    exit_location = List.Vehicle.Active(i,4) * Parameter.Map.Scale;
                    
                    % 차량의 평균 속도 계산 및 기록 (45초 이후에 생성된 차량만)
                    if strcmp(Simulation.Setting.SpawnMode, 'fixed') || spawn_time >= Simulation.Setting.WarmupTime
                        avg_speed = exit_location / travel_time;
                        vehicle_speeds = [vehicle_speeds, avg_speed]; %#ok<AGROW>
                        through_vehicle_speeds = [through_vehicle_speeds, avg_speed]; %#ok<AGROW>
                    end

                    travel_times = [travel_times, travel_time]; %#ok<AGROW>
                    
                    if Simulation.Setting.RecordLog 
                        SaveFolder = 'C:\Users\nana\Desktop\250423_0430';
                        logFileName = fullfile(SaveFolder, ...
                            [videomemo '_log.txt']);
                        fileID = fopen(logFileName, 'a', 'n', 'utf-8');  % append 모드로 파일 열기
                        fprintf(fileID,'Through Vehicle %d exited at location %.2f m with travel time %.2f s\n', ...
                                List.Vehicle.Object{v_id}.ID, exit_location, travel_time);
                        fclose(fileID);
                    end

                    RemoveVehicle(List.Vehicle.Object{v_id})
                    List.Vehicle.Object{v_id} = [];
                    TotalVehicles = TotalVehicles - 1;
                

                elseif List.Vehicle.Object{v_id}.ExitState >= 0 && List.Vehicle.Active(i,4) * Parameter.Map.Scale >= List.Vehicle.Object{v_id}.Exit - 1 
                    % record travel time, avg speed
                    spawn_time = List.Vehicle.Object{v_id}.SpawnTime;
                    exit_time = Time;
                    travel_time = exit_time - spawn_time;
                    exit_location = List.Vehicle.Active(i,4) * Parameter.Map.Scale;

                    % 차량의 평균 속도 계산 및 기록 (45초 이후에 생성된 차량만)
                    if strcmp(Simulation.Setting.SpawnMode, 'fixed') || spawn_time >= Simulation.Setting.WarmupTime
                        avg_speed = exit_location / travel_time;
                        vehicle_speeds = [vehicle_speeds, avg_speed]; %#ok<AGROW>
                        exit_vehicle_speeds = [exit_vehicle_speeds, avg_speed]; %#ok<AGROW>

                        % 🔹 Exit 성공 차량인지 확인 (45초 이후에 생성된 차량만)
                        if List.Vehicle.Object{v_id}.Lane == Parameter.Map.Lane
                            exit_success_count = exit_success_count + 1;
                        else
                            exit_fail_count = exit_fail_count + 1;  % 🔹 최우측 차선이 아니면 exit fail로 기록
                        end
                    end

                    if Simulation.Setting.RecordLog 
                        SaveFolder = 'C:\Users\nana\Desktop\250430_0514';
                        logFileName = fullfile(SaveFolder, ...
                            [videomemo '_log.txt']);
                        fileID = fopen(logFileName, 'a', 'n', 'utf-8');  % append 모드로 파일 열기
                        fprintf(fileID,'Exit Vehicle %d exited at location %.2f m with travel time %.2f s\n', ...
                                List.Vehicle.Object{v_id}.ID, exit_location, travel_time);
                        fclose(fileID);
                    end
                    
                    
                    % remove vehicle
                    RemoveVehicle(List.Vehicle.Object{v_id})
                    List.Vehicle.Object{v_id} = [];
                end
                
                
            end

        
            % Finalize Time Step
            if Simulation.Setting.Draw == 1
                drawnow();

                pause(Simulation.Setting.PauseTime);

                if Simulation.Setting.RecordVideo == 1 && mod(Time, 1/15) < Parameter.Physics
                    frame = getframe(gcf); 
                    [H, W, ~] = size(frame.cdata);
                    H = H + mod(H, 2);  % 높이가 홀수면 +1
                    W = W + mod(W, 2);  % 너비가 홀수면 +1
                    frame.cdata = imresize(frame.cdata, [H, W]);
                    % frame.cdata = imresize(frame.cdata, 0.5);
                    writeVideo(videoWriter, frame); 
                end
            end
            if strcmp(Simulation.Setting.SpawnMode, "fixed") && ...
               isempty(List.Vehicle.Active) && isempty(SpawnVehicle)
                break
            end

        end

        total_exited_vehicles = exit_success_count + exit_fail_count;
        if TotalVehicles ~= total_exited_vehicles
            %disp("something wrong here");
        end

        avg_travel_time = round(mean(travel_times), 3);
        if isempty(travel_times)
            avg_travel_time = NaN;
        end
        
        if Simulation.Setting.RecordLog
            SaveFolder = 'C:\Users\nana\Desktop\250326_0409';
            logFileName = fullfile(SaveFolder, ...
                [videomemo '_log.txt']);
            fileID = fopen(logFileName, 'a', 'n', 'utf-8');  % append 모드로 파일 열기
            fprintf(fileID,'through avg travel time %.2f s\n', avg_travel_time);
            fclose(fileID);
        end

        % 각 mode별 결과 계산
        % 현재 도로 위에 있는 모든 차량의 속도도 포함
        for i = 1:size(List.Vehicle.Active,1)
            v_id = List.Vehicle.Active(i, 1);
            current_vehicle = List.Vehicle.Object{v_id};
            if ~isempty(current_vehicle)
                % 차량의 이동 거리와 시간으로 평균 속도 계산
                travel_time = Time - current_vehicle.SpawnTime;
                distance = current_vehicle.Location * Parameter.Map.Scale;
                avg_speed = distance / travel_time;
                vehicle_speeds = [vehicle_speeds, avg_speed]; %#ok<AGROW>
            end
        end
        
        % 평균 속도와 표준편차 계산 (모든 차량 포함)
        vehicle_speeds = vehicle_speeds(~isnan(vehicle_speeds) & ~isinf(vehicle_speeds));  % 유효한 값만 필터링
        exit_vehicle_speeds = exit_vehicle_speeds(~isnan(exit_vehicle_speeds) & ~isinf(exit_vehicle_speeds));
        through_vehicle_speeds = through_vehicle_speeds(~isnan(through_vehicle_speeds) & ~isinf(through_vehicle_speeds));
        
        if isempty(vehicle_speeds)
            avg_speed = NaN;
            std_speed = NaN;
        else
            avg_speed = mean(vehicle_speeds);
            std_speed = std(vehicle_speeds);
        end
        
        % Exit 차량의 평균 속도와 표준편차 계산
        if isempty(exit_vehicle_speeds)
            avg_exit_speed = NaN;
            std_exit_speed = NaN;
        else
            avg_exit_speed = mean(exit_vehicle_speeds);
            std_exit_speed = std(exit_vehicle_speeds);
        end
        
        % Through 차량의 평균 속도와 표준편차 계산
        if isempty(through_vehicle_speeds)
            avg_through_speed = NaN;
            std_through_speed = NaN;
        else
            avg_through_speed = mean(through_vehicle_speeds);
            std_through_speed = std(through_vehicle_speeds);
        end
        
        % 도로 용량 계산 (capacity check point를 통과한 총 차량 수)
        road_capacity = vehicles_passed;  % 차량 수로 변경
        
        % 출구 실패율 계산
        exit_fail_rate = exit_fail_count / (exit_success_count + exit_fail_count);
        
        % m/s를 km/h로 변환 (1 m/s = 3.6 km/h)
        avg_speed = avg_speed * 3.6;
        std_speed = std_speed * 3.6;
        avg_exit_speed = avg_exit_speed * 3.6;
        std_exit_speed = std_exit_speed * 3.6;
        avg_through_speed = avg_through_speed * 3.6;
        std_through_speed = std_through_speed * 3.6;
        
        % 결과 행에 저장
        result_row{2} = avg_speed;
        result_row{3} = std_speed;
        result_row{4} = road_capacity;
        result_row{5} = exit_fail_rate;
        result_row{6} = avg_exit_speed;
        result_row{7} = std_exit_speed;
        result_row{8} = avg_through_speed;
        result_row{9} = std_through_speed;
        
        % 현재 mode의 결과를 해당 시트에 저장
        if Simulation.Setting.RecordExcel
            % 시트 이름에서 특수문자 제거 및 공백을 언더스코어로 변경
            sheet_name = strrep(mode_names{mode_idx}, ' ', '_');
            % 소수점은 p로 대체
            sheet_name = regexprep(sheet_name, '\.', 'p');
            sheet_name = regexprep(sheet_name, '[^a-zA-Z0-9_]', '');
            headers = {'Random Seed', 'Average Speed (km/h)', 'Speed STD (km/h)', 'Passed Vehicles', 'Exit Fail Rate', ...
                      'Exit Avg Speed (km/h)', 'Exit Speed STD (km/h)', 'Through Avg Speed (km/h)', 'Through Speed STD (km/h)'};

            if ~isfile(filename)
                % 파일이 없으면 먼저 헤더를 써서 파일 생성
                writecell(headers, filename, 'Sheet', sheet_name, 'WriteMode', 'overwrite');
                disp(['Created new file and sheet: ', sheet_name]);
            elseif ~ismember(sheet_name, sheetnames(filename))
                % 파일이 있으면 시트가 없을 때만 헤더 작성
                writecell(headers, filename, 'Sheet', sheet_name, 'WriteMode', 'overwrite');
                disp(['Created new sheet: ', sheet_name]);
            end

            % 결과 추가
            writecell(result_row, filename, 'Sheet', sheet_name, 'WriteMode', 'append');
            disp(['Added results to sheet: ', sheet_name]);
        end

        disp("Iteration: " + Iteration)
        % if Time > Parameter.Sim.Time - Parameter.Physics
            
        % end


        if Simulation.Setting.RecordVideo
            close(videoWriter);
            disp(['Simulation video saved to: ', videoFilename]);
        end

        clear Parameter List Seed environment GRAPE_output;

    end
    result_row{2} = TotalVehicles;
    results(Iteration, :) = result_row;
end
end
