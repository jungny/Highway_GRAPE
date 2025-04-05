%===
close all
clear
clc
addpath('Map\','Vehicle\','Signal\','Manager\','v2v\','GRAPE\')

Simulation.Setting.Window = 1000;
Simulation.Setting.Draw = 1;
Simulation.Setting.StopOnGrapeError = 1;
Simulation.Setting.PauseTime = 0; % 0: No pause. >0: Pause duration in seconds (Default: 0.01)
Simulation.Setting.SaveFolder = 'C:\Users\user\Desktop\250313_0326';

Simulation.Setting.RecordLog = 0;    % 1: Record log file, 0: Do not record
Simulation.Setting.RecordVideo = 0;  % 1: Record video file, 0: Do not record
Simulation.Setting.RecordExcel = 1;  % 1: Record Excel file, 0: Do not record

Simulation.Setting.VideoPath = @(mode, randomSeed, timestamp) ...
    fullfile(Simulation.Setting.SaveFolder, 'Simulations', ...
    ['_' mode '_' num2str(randomSeed) '_' timestamp '.mp4']);

Simulation.Setting.LogPath = @(finalRandomSeed) ...
    fullfile(Simulation.Setting.SaveFolder, 'Simulations', ...
    ['log_' num2str(finalRandomSeed) '.txt']);

cycle_GRAPE =2; % GRAPE instance per 5 seconds

Simulation.Setting.InitialRandomSeed = 1;
Simulation.Setting.Iterations = 20; % number of iterations
Simulation.Setting.Time = 10000;

Simulation.Setting.SpawnType = 1; % 0: Automatically spawn vehicles based on flow rate, 1: Manually define spawn times, 2: Debug mode
Simulation.Setting.GreedyAlloc = 0; % 0: Distributed Mutex is applied (GRAPE), 1: Agents make fully greedy decisions (Baseline)

Simulation.Setting.BubbleRadiusList = [0,1,5,10,15,20,25,30,35,40,45,50,60,70,80,90,100,150,300,500,1000,2100];
%Simulation.Setting.BubbleRadiusList = [0];
Simulation.Setting.Util_type = 'GS'; 
%Simulation.Setting.Util_type = 'HOS'; 
%Simulation.Setting.Util_type = 'FOS'; 
%Simulation.Setting.Util_type = 'ES'; 
Simulation.Setting.LaneChangeMode = 'SimpleLaneChange'; % 'MOBIL' or 'SimpleLaneChange'


%% Run Simulation
% Initialize Log File
if Simulation.Setting.RecordLog
    finalRandomSeed = Simulation.Setting.InitialRandomSeed + Simulation.Setting.Iterations - 1;
    logFileName = Simulation.Setting.LogPath(finalRandomSeed);

    % 새 로그 파일 생성 및 헤더 작성
    fileID = fopen(logFileName, 'w');
    fprintf(fileID, 'Simulation Log\n');
    fclose(fileID);
end

if Simulation.Setting.SpawnType % If vehicles are spawned manually based on predefined times
    Simulation.Setting.Time = 10000; % Set a very high simulation time to allow all vehicles to spawn
end

% GreedyAlloc 여부를 아이콘으로 변환
if Simulation.Setting.GreedyAlloc == 1
    greedy_status = 'GRAPE ❌';
    greedy_status2 = 'Greedy';
else
    greedy_status = 'GRAPE ⭕';
    greedy_status2 = 'GRAPE';
end

% 🔹 엑셀 파일 경로 설정
timestamp = datestr(now, 'HH-MM');  % 현재 시간 가져오기 (시-분-초 형식)
filename = fullfile(Simulation.Setting.SaveFolder, ['nht_' greedy_status2 '_' Simulation.Setting.Util_type '_' timestamp '.xlsx']);
sheet = 'Results';

% 🔹 실험할 참가자 모드 설정
participantModes = {'Default', 'Ahead'};  % 기본 모드
%participantModes = {};
% 🔹 Bubble Radius 값에 따라 Bubble 관련 모드 추가
for r = Simulation.Setting.BubbleRadiusList
    participantModes{end+1} = sprintf('Bubble_%dm', r);
    participantModes{end+1} = sprintf('BubbleAhead_%dm', r);
end
num_modes = length(participantModes);

% 🔹 엑셀 파일이 존재하지 않으면 헤더만 추가하여 생성
if Simulation.Setting.RecordExcel && ~isfile(filename)
    if isstring(participantModes)
        participantModes = cellstr(participantModes);
    end
    header = [{'Random Seed'}, {'Total Vehicles'}, participantModes, strcat("ExitFail_", participantModes)];
    writematrix(header, filename, 'Sheet', sheet, 'WriteMode', 'overwrite')
    disp(['New Excel file created: ', filename]);
end

% 🔹 결과 저장을 위한 배열 초기화
if Simulation.Setting.RecordExcel
    num_simulations = Simulation.Setting.Iterations;
    results = cell(num_simulations, (num_modes * 2) + 2); % random seed, total vehicles, <avg travel time, fail rate> each mode
end

for Iteration = 1:Simulation.Setting.Iterations
    randomSeed = Simulation.Setting.InitialRandomSeed + Iteration - 1;
    rng(randomSeed)
    %Simulation.Setting.RandomSeed = randomSeed;

    % 현재 random seed에 대한 결과 저장할 행 초기화
    result_row = cell(1, (num_modes * 2) + 2);
    result_row{1} = randomSeed;  % 첫 번째 칸에 random seed 저장


    for mode_idx = 1:num_modes
        close all
        rng(randomSeed)
        Simulation.Setting.NumberOfParticipants = participantModes{mode_idx};

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

        disp(['Running ', participantModes{mode_idx}, ' mode, Random Seed ', num2str(randomSeed)]);

        environment = struct();
        GRAPE_output = [];
        travel_times = [];
        exit_fail = 0;
        RemovedVehicle = 0;
        exit_fail_count = 0;
        exit_success_count = 0;

        Parameter = GetParameters(Simulation.Setting);
        GetWindow(Parameter.Map,Simulation.Setting)
        Parameter.Trajectory = GetTrajectory(Parameter.Map,Simulation.Setting);

        if Simulation.Setting.RecordVideo
            timestamp = datestr(now, 'HH-MM');
            % videoFilename = Simulation.Setting.VideoPath(participantModes{mode_idx}, randomSeed, timestamp);
            % videoFilename = filename;
            videoFilename = fullfile(Simulation.Setting.SaveFolder, ['\video\Seed5_' greedy_status2 '_' Simulation.Setting.Util_type '_' timestamp '_' Simulation.Setting.NumberOfParticipants]);
            videoWriter = VideoWriter(videoFilename, 'MPEG-4');
            videoWriter.FrameRate = 30; 
            open(videoWriter);
        end

        fprintf('%d th Iteration. Random Seed is %d.\n', Iteration, randomSeed)

        SetMap(Parameter.Map, Simulation.Setting);


        if Simulation.Setting.RecordLog
            fileID = fopen(logFileName, 'a', 'n', 'utf-8');  % append 모드로 파일 열기
            fprintf(fileID, '\n=====   Random Seed  %d  ||  %s   ===== %s \n', ...
                    randomSeed, Simulation.Setting.NumberOfParticipants, datestr(now, 'HH시 MM분 SS초'));
            
            % fprintf(fileID, '\n| Lanes:  %d  | Vehicles: %d  | Exits: %d  |\n', ...
            %     Parameter.Map.Lane, TotalVehicles, length(Parameter.Map.Exit));
            fclose(fileID);
        end


        NextArrivalTime = zeros(Parameter.Map.Lane, 1); % 5차선 기준 [0;0;0;0;0]
        %NextArrivalTime = (3600 / Parameter.Flow) * rand(Parameter.Map.Lane, 1);
        %disp(NextArrivalTime);
        TotalVehicles = 0;
        firstCount = 0;
        SpawnVehicle = [];
        SpawnLanes = [];

        for Time = 0:Parameter.Physics:Parameter.Sim.Time
            GRAPE_done = 0;            
            % 제목 출력
            title(sprintf('Random Seed: %d   |   %s   |   Participants Mode: %s   |   Time: %.2f s', ...
                randomSeed, greedy_status, strrep(participantModes{mode_idx}, '_', ' '), Time));


            if Simulation.Setting.SpawnType == 0 
                SpawnLanes = find(NextArrivalTime < Time+Parameter.Physics); % 차량 스폰 필요한 차선
                SpawnCount = length(SpawnLanes); % 차량 스폰 필요한 차선의 개수

                if SpawnCount > 0
                    [SpawnVehicle, NextArrivalTime] = GetSeed(Simulation.Setting, Parameter, TotalVehicles, SpawnLanes, NextArrivalTime);
                    TotalVehicles = TotalVehicles + SpawnCount;
                    if firstCount == 0
                        List.Vehicle.Object = cell(size(SpawnVehicle,2),1);
                        firstCount = 1;
                    end
                    List.Vehicle.Object = cat(1, List.Vehicle.Object, cell(size(SpawnVehicle,2),1));
                end

                % Generate Vehicles
                while ~isempty(SpawnVehicle)
                    List.Vehicle.Object{SpawnVehicle(1,1)} = Vehicle(SpawnVehicle(:,1),Time,Parameter);
                    SpawnVehicle = SpawnVehicle(:,2:end);  % 생성된 차량 삭제
                end

            elseif Simulation.Setting.SpawnType == 1 || Simulation.Setting.SpawnType == 2
                if firstCount == 0
                    [SpawnVehicle, TotalVehicles] = GetSeed(Simulation.Setting, Parameter, TotalVehicles, SpawnLanes, NextArrivalTime);
                    List.Vehicle.Object = cell(size(SpawnVehicle,2),1);
                    firstCount = 1;
                end
                if ~isempty(SpawnVehicle) 
                    if int32(Time/Parameter.Physics) == int32(SpawnVehicle(6,1)/Parameter.Physics)
                        List.Vehicle.Object{SpawnVehicle(1,1)} = Vehicle(SpawnVehicle(:,1),Time,Parameter);
                        if ~isempty(SpawnVehicle)
                            SpawnVehicle = SpawnVehicle(:,2:end);
                        end
                    end
                end
            end

            % Update Vehicle Data
            List.Vehicle.Data = UpdateData(List.Vehicle.Object,Parameter.Sim.Data);
            List.Vehicle.Active = List.Vehicle.Data(List.Vehicle.Data(:,2)>0,:);
            List.Vehicle.Object = GetAcceleration(List.Vehicle.Object, List.Vehicle.Data, Parameter.Veh);

            if Simulation.Setting.GreedyAlloc
                environment = GRAPE_main(List,Parameter,Simulation.Setting,Iteration);
                lane_alloc = GRAPE_instance(environment).Alloc;

            elseif mod(Time, cycle_GRAPE) == cycle_GRAPE-1 && size(List.Vehicle.Active,1)>0  %&& Time > 8
                disp("calling Grape Instance. . . | "+ Time);
                environment = GRAPE_main(List,Parameter,Simulation.Setting,Iteration);
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
        
            % Move Vehicle
            for i = 1:size(List.Vehicle.Active,1)
                vehicle_id = List.Vehicle.Active(i, 1); 
                current_vehicle = List.Vehicle.Object{vehicle_id};
                current_lane = List.Vehicle.Object{vehicle_id}.Lane; 
                
                if GRAPE_done == 1 || Simulation.Setting.GreedyAlloc
                    desired_lane = lane_alloc(i);
                
                    if current_lane ~= desired_lane 
                        %List.Vehicle.Object{vehicle_id}.TargetLane = desired_lane;
                        %List.Vehicle.Object{vehicle_id}.LaneChangeFlag = 1; 
                        if current_lane > desired_lane
                            desired_lane = current_lane - 1;
                        elseif current_lane < desired_lane
                            desired_lane = current_lane + 1;
                        end

                        if strcmp(Simulation.Setting.LaneChangeMode, 'MOBIL')
                            [feasible, a_c_sim] = MOBIL(current_vehicle, desired_lane, List, Parameter);
                        elseif strcmp(Simulation.Setting.LaneChangeMode, 'SimpleLaneChange')
                            [feasible] = SimpleLaneChange(current_vehicle, desired_lane, List, Parameter);
                        end
                        
                        % warm up 구간 동안은 차선 변경 안 되게 설정정
                        if current_vehicle.Location * Parameter.Map.Scale < 200
                            feasible = false;
                        end

                        if feasible %&& Simulation.Setting.GreedyAlloc
                            if current_vehicle.IsChangingLane 
                                current_vehicle.LaneChangeFlag = 0;
                            else
                                current_vehicle.TargetLane = desired_lane;
                                current_vehicle.LaneChangeFlag = 1;
                            end
                        %elseif feasible
                        %    current_vehicle.TargetLane = desired_lane;
                        %    current_vehicle.LaneChangeFlag = 1;
                        else
                            current_vehicle.LaneChangeFlag = 0;
                        end
                    end
                end
                
                if List.Vehicle.Object{vehicle_id}.Exit - List.Vehicle.Object{vehicle_id}.Location * Parameter.Map.Scale <= Parameter.ExitThreshold 
                    if current_lane == Parameter.Map.Lane
                        List.Vehicle.Object{vehicle_id}.ExitState = 1;
                    else
                        List.Vehicle.Object{vehicle_id}.ExitState = 0;
                    end
                end

                MoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)},Time,Parameter,List)
            end
        
            % Remove Processed Vehicles
            for i = 1:size(List.Vehicle.Active,1)
                %if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location >= 300000 % exit으로 바꾸기
                %    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                %    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];

                %    % record travel time, avg speed

                %end
                if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale > Parameter.Map.Road % through vehicles
                    spawn_time = List.Vehicle.Object{List.Vehicle.Active(i,1)}.EntryTime;
                    exit_time = Time;
                    travel_time = exit_time - spawn_time;
                    exit_location = List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale;

                    travel_times = [travel_times, travel_time];
                    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];
                    TotalVehicles = TotalVehicles - 1;
                

                elseif List.Vehicle.Object{List.Vehicle.Active(i,1)}.ExitState >= 0 && List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale >= List.Vehicle.Object{List.Vehicle.Active(i,1)}.Exit - 2 
                    % record travel time, avg speed
                    spawn_time = List.Vehicle.Object{List.Vehicle.Active(i,1)}.EntryTime;
                    exit_time = Time;
                    travel_time = exit_time - spawn_time;
                    exit_location = List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale;

                    travel_times = [travel_times, travel_time];

                    % 🔹 Exit 성공 차량인지 확인
                    if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Lane == Parameter.Map.Lane
                        exit_success_count = exit_success_count + 1;
                    else
                        exit_fail_count = exit_fail_count + 1;  % 🔹 최우측 차선이 아니면 exit fail로 기록
                    end

                    % remove vehicle
                    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];
                end
                
            end

        
            % Finalize Time Step
            if Simulation.Setting.Draw == 1
                drawnow();

                pause(Simulation.Setting.PauseTime);

                if Simulation.Setting.RecordVideo == 1 % && mod(int32(Time/Parameter.Physics), 2) == 0
                    frame = getframe(gcf); 
                    % frame.cdata = imresize(frame.cdata, 0.5);
                    writeVideo(videoWriter, frame); 
                end
            end
            if Simulation.Setting.SpawnType && isempty(List.Vehicle.Active) && isempty(SpawnVehicle)
                break
            end

        end

        total_exited_vehicles = exit_success_count + exit_fail_count;
        if TotalVehicles ~= total_exited_vehicles
            disp("something wrong here");
        end
        if total_exited_vehicles > 0
            exit_fail_rate = round(exit_fail_count / total_exited_vehicles, 3); % 🔹 소수점 3자리까지
        else
            exit_fail_rate = NaN; % 🔹 차량이 하나도 없으면 NaN
        end

        avg_travel_time = round(mean(travel_times), 3);
        if isempty(travel_times)
            avg_travel_time = NaN;
        end

        % 🔹 각 mode_idx마다 결과 저장 (Avg Travel Time + Exit Fail Rate)
        result_row{2+mode_idx} = avg_travel_time;
        result_row{2+num_modes+mode_idx} = exit_fail_rate;

        disp("Iteration: " + Iteration)
        if Time > Parameter.Sim.Time - Parameter.Physics
            ;
        end


        if Simulation.Setting.RecordVideo
            close(videoWriter);
            disp(['Simulation video saved to: ', videoFilename]);
        end

        clear Parameter List Seed environment GRAPE_output;

    end
    result_row{2} = TotalVehicles;
    results(Iteration, :) = result_row;
end
% 🔹 모든 Iteration이 끝난 후, 엑셀 파일에 한 번에 결과 저장
if Simulation.Setting.RecordExcel
    writecell(results, filename, 'Sheet', sheet, 'WriteMode', 'append');
    disp(['✅ Simulation results saved to: ', filename]);
end

%33333333333333333

%Simulation.Setting.Util_type = 'GS'; 
Simulation.Setting.Util_type = 'HOS'; 
%Simulation.Setting.Util_type = 'FOS'; 
%Simulation.Setting.Util_type = 'ES'; 
Simulation.Setting.LaneChangeMode = 'SimpleLaneChange'; % 'MOBIL' or 'SimpleLaneChange'


%% Run Simulation
% Initialize Log File
if Simulation.Setting.RecordLog
    finalRandomSeed = Simulation.Setting.InitialRandomSeed + Simulation.Setting.Iterations - 1;
    logFileName = Simulation.Setting.LogPath(finalRandomSeed);

    % 새 로그 파일 생성 및 헤더 작성
    fileID = fopen(logFileName, 'w');
    fprintf(fileID, 'Simulation Log\n');
    fclose(fileID);
end

if Simulation.Setting.SpawnType % If vehicles are spawned manually based on predefined times
    Simulation.Setting.Time = 10000; % Set a very high simulation time to allow all vehicles to spawn
end

% GreedyAlloc 여부를 아이콘으로 변환
if Simulation.Setting.GreedyAlloc == 1
    greedy_status = 'GRAPE ❌';
    greedy_status2 = 'Greedy';
else
    greedy_status = 'GRAPE ⭕';
    greedy_status2 = 'GRAPE';
end

% 🔹 엑셀 파일 경로 설정
timestamp = datestr(now, 'HH-MM');  % 현재 시간 가져오기 (시-분-초 형식)
filename = fullfile(Simulation.Setting.SaveFolder, ['nht_' greedy_status2 '_' Simulation.Setting.Util_type '_' timestamp '.xlsx']);
sheet = 'Results';

% 🔹 실험할 참가자 모드 설정
participantModes = {'Default', 'Ahead'};  % 기본 모드
%participantModes = {};
% 🔹 Bubble Radius 값에 따라 Bubble 관련 모드 추가
for r = Simulation.Setting.BubbleRadiusList
    participantModes{end+1} = sprintf('Bubble_%dm', r);
    participantModes{end+1} = sprintf('BubbleAhead_%dm', r);
end
num_modes = length(participantModes);

% 🔹 엑셀 파일이 존재하지 않으면 헤더만 추가하여 생성
if Simulation.Setting.RecordExcel && ~isfile(filename)
    if isstring(participantModes)
        participantModes = cellstr(participantModes);
    end
    header = [{'Random Seed'}, {'Total Vehicles'}, participantModes, strcat("ExitFail_", participantModes)];
    writematrix(header, filename, 'Sheet', sheet, 'WriteMode', 'overwrite')
    disp(['New Excel file created: ', filename]);
end

% 🔹 결과 저장을 위한 배열 초기화
if Simulation.Setting.RecordExcel
    num_simulations = Simulation.Setting.Iterations;
    results = cell(num_simulations, (num_modes * 2) + 2); % random seed, total vehicles, <avg travel time, fail rate> each mode
end

for Iteration = 1:Simulation.Setting.Iterations
    randomSeed = Simulation.Setting.InitialRandomSeed + Iteration - 1;
    rng(randomSeed)
    %Simulation.Setting.RandomSeed = randomSeed;

    % 현재 random seed에 대한 결과 저장할 행 초기화
    result_row = cell(1, (num_modes * 2) + 2);
    result_row{1} = randomSeed;  % 첫 번째 칸에 random seed 저장


    for mode_idx = 1:num_modes
        close all
        rng(randomSeed)
        Simulation.Setting.NumberOfParticipants = participantModes{mode_idx};

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

        disp(['Running ', participantModes{mode_idx}, ' mode, Random Seed ', num2str(randomSeed)]);

        environment = struct();
        GRAPE_output = [];
        travel_times = [];
        exit_fail = 0;
        RemovedVehicle = 0;
        exit_fail_count = 0;
        exit_success_count = 0;

        Parameter = GetParameters(Simulation.Setting);
        GetWindow(Parameter.Map,Simulation.Setting)
        Parameter.Trajectory = GetTrajectory(Parameter.Map,Simulation.Setting);

        if Simulation.Setting.RecordVideo
            timestamp = datestr(now, 'HH-MM');
            % videoFilename = Simulation.Setting.VideoPath(participantModes{mode_idx}, randomSeed, timestamp);
            % videoFilename = filename;
            videoFilename = fullfile(Simulation.Setting.SaveFolder, ['\video\Seed5_' greedy_status2 '_' Simulation.Setting.Util_type '_' timestamp '_' Simulation.Setting.NumberOfParticipants]);
            videoWriter = VideoWriter(videoFilename, 'MPEG-4');
            videoWriter.FrameRate = 30; 
            open(videoWriter);
        end

        fprintf('%d th Iteration. Random Seed is %d.\n', Iteration, randomSeed)

        SetMap(Parameter.Map, Simulation.Setting);


        if Simulation.Setting.RecordLog
            fileID = fopen(logFileName, 'a', 'n', 'utf-8');  % append 모드로 파일 열기
            fprintf(fileID, '\n=====   Random Seed  %d  ||  %s   ===== %s \n', ...
                    randomSeed, Simulation.Setting.NumberOfParticipants, datestr(now, 'HH시 MM분 SS초'));
            
            % fprintf(fileID, '\n| Lanes:  %d  | Vehicles: %d  | Exits: %d  |\n', ...
            %     Parameter.Map.Lane, TotalVehicles, length(Parameter.Map.Exit));
            fclose(fileID);
        end


        NextArrivalTime = zeros(Parameter.Map.Lane, 1); % 5차선 기준 [0;0;0;0;0]
        %NextArrivalTime = (3600 / Parameter.Flow) * rand(Parameter.Map.Lane, 1);
        %disp(NextArrivalTime);
        TotalVehicles = 0;
        firstCount = 0;
        SpawnVehicle = [];
        SpawnLanes = [];

        for Time = 0:Parameter.Physics:Parameter.Sim.Time
            GRAPE_done = 0;            
            % 제목 출력
            title(sprintf('Random Seed: %d   |   %s   |   Participants Mode: %s   |   Time: %.2f s', ...
                randomSeed, greedy_status, strrep(participantModes{mode_idx}, '_', ' '), Time));


            if Simulation.Setting.SpawnType == 0 
                SpawnLanes = find(NextArrivalTime < Time+Parameter.Physics); % 차량 스폰 필요한 차선
                SpawnCount = length(SpawnLanes); % 차량 스폰 필요한 차선의 개수

                if SpawnCount > 0
                    [SpawnVehicle, NextArrivalTime] = GetSeed(Simulation.Setting, Parameter, TotalVehicles, SpawnLanes, NextArrivalTime);
                    TotalVehicles = TotalVehicles + SpawnCount;
                    if firstCount == 0
                        List.Vehicle.Object = cell(size(SpawnVehicle,2),1);
                        firstCount = 1;
                    end
                    List.Vehicle.Object = cat(1, List.Vehicle.Object, cell(size(SpawnVehicle,2),1));
                end

                % Generate Vehicles
                while ~isempty(SpawnVehicle)
                    List.Vehicle.Object{SpawnVehicle(1,1)} = Vehicle(SpawnVehicle(:,1),Time,Parameter);
                    SpawnVehicle = SpawnVehicle(:,2:end);  % 생성된 차량 삭제
                end

            elseif Simulation.Setting.SpawnType == 1 || Simulation.Setting.SpawnType == 2
                if firstCount == 0
                    [SpawnVehicle, TotalVehicles] = GetSeed(Simulation.Setting, Parameter, TotalVehicles, SpawnLanes, NextArrivalTime);
                    List.Vehicle.Object = cell(size(SpawnVehicle,2),1);
                    firstCount = 1;
                end
                if ~isempty(SpawnVehicle) 
                    if int32(Time/Parameter.Physics) == int32(SpawnVehicle(6,1)/Parameter.Physics)
                        List.Vehicle.Object{SpawnVehicle(1,1)} = Vehicle(SpawnVehicle(:,1),Time,Parameter);
                        if ~isempty(SpawnVehicle)
                            SpawnVehicle = SpawnVehicle(:,2:end);
                        end
                    end
                end
            end

            % Update Vehicle Data
            List.Vehicle.Data = UpdateData(List.Vehicle.Object,Parameter.Sim.Data);
            List.Vehicle.Active = List.Vehicle.Data(List.Vehicle.Data(:,2)>0,:);
            List.Vehicle.Object = GetAcceleration(List.Vehicle.Object, List.Vehicle.Data, Parameter.Veh);

            if Simulation.Setting.GreedyAlloc
                environment = GRAPE_main(List,Parameter,Simulation.Setting,Iteration);
                lane_alloc = GRAPE_instance(environment).Alloc;

            elseif mod(Time, cycle_GRAPE) == cycle_GRAPE-1 && size(List.Vehicle.Active,1)>0  %&& Time > 8
                disp("calling Grape Instance. . . | "+ Time);
                environment = GRAPE_main(List,Parameter,Simulation.Setting,Iteration);
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
        
            % Move Vehicle
            for i = 1:size(List.Vehicle.Active,1)
                vehicle_id = List.Vehicle.Active(i, 1); 
                current_vehicle = List.Vehicle.Object{vehicle_id};
                current_lane = List.Vehicle.Object{vehicle_id}.Lane; 
                
                if GRAPE_done == 1 || Simulation.Setting.GreedyAlloc
                    desired_lane = lane_alloc(i);
                
                    if current_lane ~= desired_lane 
                        %List.Vehicle.Object{vehicle_id}.TargetLane = desired_lane;
                        %List.Vehicle.Object{vehicle_id}.LaneChangeFlag = 1; 
                        if current_lane > desired_lane
                            desired_lane = current_lane - 1;
                        elseif current_lane < desired_lane
                            desired_lane = current_lane + 1;
                        end

                        if strcmp(Simulation.Setting.LaneChangeMode, 'MOBIL')
                            [feasible, a_c_sim] = MOBIL(current_vehicle, desired_lane, List, Parameter);
                        elseif strcmp(Simulation.Setting.LaneChangeMode, 'SimpleLaneChange')
                            [feasible] = SimpleLaneChange(current_vehicle, desired_lane, List, Parameter);
                        end
                        
                        % warm up 구간 동안은 차선 변경 안 되게 설정정
                        if current_vehicle.Location * Parameter.Map.Scale < 200
                            feasible = false;
                        end

                        if feasible %&& Simulation.Setting.GreedyAlloc
                            if current_vehicle.IsChangingLane 
                                current_vehicle.LaneChangeFlag = 0;
                            else
                                current_vehicle.TargetLane = desired_lane;
                                current_vehicle.LaneChangeFlag = 1;
                            end
                        %elseif feasible
                        %    current_vehicle.TargetLane = desired_lane;
                        %    current_vehicle.LaneChangeFlag = 1;
                        else
                            current_vehicle.LaneChangeFlag = 0;
                        end
                    end
                end
                
                if List.Vehicle.Object{vehicle_id}.Exit - List.Vehicle.Object{vehicle_id}.Location * Parameter.Map.Scale <= Parameter.ExitThreshold 
                    if current_lane == Parameter.Map.Lane
                        List.Vehicle.Object{vehicle_id}.ExitState = 1;
                    else
                        List.Vehicle.Object{vehicle_id}.ExitState = 0;
                    end
                end

                MoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)},Time,Parameter,List)
            end
        
            % Remove Processed Vehicles
            for i = 1:size(List.Vehicle.Active,1)
                %if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location >= 300000 % exit으로 바꾸기
                %    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                %    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];

                %    % record travel time, avg speed

                %end
                if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale > Parameter.Map.Road % through vehicles
                    spawn_time = List.Vehicle.Object{List.Vehicle.Active(i,1)}.EntryTime;
                    exit_time = Time;
                    travel_time = exit_time - spawn_time;
                    exit_location = List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale;

                    travel_times = [travel_times, travel_time];
                    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];
                    TotalVehicles = TotalVehicles - 1;
                

                elseif List.Vehicle.Object{List.Vehicle.Active(i,1)}.ExitState >= 0 && List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale >= List.Vehicle.Object{List.Vehicle.Active(i,1)}.Exit - 2 
                    % record travel time, avg speed
                    spawn_time = List.Vehicle.Object{List.Vehicle.Active(i,1)}.EntryTime;
                    exit_time = Time;
                    travel_time = exit_time - spawn_time;
                    exit_location = List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale;

                    travel_times = [travel_times, travel_time];

                    % 🔹 Exit 성공 차량인지 확인
                    if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Lane == Parameter.Map.Lane
                        exit_success_count = exit_success_count + 1;
                    else
                        exit_fail_count = exit_fail_count + 1;  % 🔹 최우측 차선이 아니면 exit fail로 기록
                    end

                    % remove vehicle
                    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];
                end
                
            end

        
            % Finalize Time Step
            if Simulation.Setting.Draw == 1
                drawnow();

                pause(Simulation.Setting.PauseTime);

                if Simulation.Setting.RecordVideo == 1 % && mod(int32(Time/Parameter.Physics), 2) == 0
                    frame = getframe(gcf); 
                    % frame.cdata = imresize(frame.cdata, 0.5);
                    writeVideo(videoWriter, frame); 
                end
            end
            if Simulation.Setting.SpawnType && isempty(List.Vehicle.Active) && isempty(SpawnVehicle)
                break
            end

        end

        total_exited_vehicles = exit_success_count + exit_fail_count;
        if TotalVehicles ~= total_exited_vehicles
            disp("something wrong here");
        end
        if total_exited_vehicles > 0
            exit_fail_rate = round(exit_fail_count / total_exited_vehicles, 3); % 🔹 소수점 3자리까지
        else
            exit_fail_rate = NaN; % 🔹 차량이 하나도 없으면 NaN
        end

        avg_travel_time = round(mean(travel_times), 3);
        if isempty(travel_times)
            avg_travel_time = NaN;
        end

        % 🔹 각 mode_idx마다 결과 저장 (Avg Travel Time + Exit Fail Rate)
        result_row{2+mode_idx} = avg_travel_time;
        result_row{2+num_modes+mode_idx} = exit_fail_rate;

        disp("Iteration: " + Iteration)
        if Time > Parameter.Sim.Time - Parameter.Physics
            ;
        end


        if Simulation.Setting.RecordVideo
            close(videoWriter);
            disp(['Simulation video saved to: ', videoFilename]);
        end

        clear Parameter List Seed environment GRAPE_output;

    end
    result_row{2} = TotalVehicles;
    results(Iteration, :) = result_row;
end
% 🔹 모든 Iteration이 끝난 후, 엑셀 파일에 한 번에 결과 저장
if Simulation.Setting.RecordExcel
    writecell(results, filename, 'Sheet', sheet, 'WriteMode', 'append');
    disp(['✅ Simulation results saved to: ', filename]);
end

%3333333333333333

%Simulation.Setting.Util_type = 'GS'; 
%Simulation.Setting.Util_type = 'HOS'; 
Simulation.Setting.Util_type = 'FOS'; 
%Simulation.Setting.Util_type = 'ES'; 
Simulation.Setting.LaneChangeMode = 'SimpleLaneChange'; % 'MOBIL' or 'SimpleLaneChange'


%% Run Simulation
% Initialize Log File
if Simulation.Setting.RecordLog
    finalRandomSeed = Simulation.Setting.InitialRandomSeed + Simulation.Setting.Iterations - 1;
    logFileName = Simulation.Setting.LogPath(finalRandomSeed);

    % 새 로그 파일 생성 및 헤더 작성
    fileID = fopen(logFileName, 'w');
    fprintf(fileID, 'Simulation Log\n');
    fclose(fileID);
end

if Simulation.Setting.SpawnType % If vehicles are spawned manually based on predefined times
    Simulation.Setting.Time = 10000; % Set a very high simulation time to allow all vehicles to spawn
end

% GreedyAlloc 여부를 아이콘으로 변환
if Simulation.Setting.GreedyAlloc == 1
    greedy_status = 'GRAPE ❌';
    greedy_status2 = 'Greedy';
else
    greedy_status = 'GRAPE ⭕';
    greedy_status2 = 'GRAPE';
end

% 🔹 엑셀 파일 경로 설정
timestamp = datestr(now, 'HH-MM');  % 현재 시간 가져오기 (시-분-초 형식)
filename = fullfile(Simulation.Setting.SaveFolder, ['nht_' greedy_status2 '_' Simulation.Setting.Util_type '_' timestamp '.xlsx']);
sheet = 'Results';

% 🔹 실험할 참가자 모드 설정
participantModes = {'Default', 'Ahead'};  % 기본 모드
%participantModes = {};
% 🔹 Bubble Radius 값에 따라 Bubble 관련 모드 추가
for r = Simulation.Setting.BubbleRadiusList
    participantModes{end+1} = sprintf('Bubble_%dm', r);
    participantModes{end+1} = sprintf('BubbleAhead_%dm', r);
end
num_modes = length(participantModes);

% 🔹 엑셀 파일이 존재하지 않으면 헤더만 추가하여 생성
if Simulation.Setting.RecordExcel && ~isfile(filename)
    if isstring(participantModes)
        participantModes = cellstr(participantModes);
    end
    header = [{'Random Seed'}, {'Total Vehicles'}, participantModes, strcat("ExitFail_", participantModes)];
    writematrix(header, filename, 'Sheet', sheet, 'WriteMode', 'overwrite')
    disp(['New Excel file created: ', filename]);
end

% 🔹 결과 저장을 위한 배열 초기화
if Simulation.Setting.RecordExcel
    num_simulations = Simulation.Setting.Iterations;
    results = cell(num_simulations, (num_modes * 2) + 2); % random seed, total vehicles, <avg travel time, fail rate> each mode
end

for Iteration = 1:Simulation.Setting.Iterations
    randomSeed = Simulation.Setting.InitialRandomSeed + Iteration - 1;
    rng(randomSeed)
    %Simulation.Setting.RandomSeed = randomSeed;

    % 현재 random seed에 대한 결과 저장할 행 초기화
    result_row = cell(1, (num_modes * 2) + 2);
    result_row{1} = randomSeed;  % 첫 번째 칸에 random seed 저장


    for mode_idx = 1:num_modes
        close all
        rng(randomSeed)
        Simulation.Setting.NumberOfParticipants = participantModes{mode_idx};

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

        disp(['Running ', participantModes{mode_idx}, ' mode, Random Seed ', num2str(randomSeed)]);

        environment = struct();
        GRAPE_output = [];
        travel_times = [];
        exit_fail = 0;
        RemovedVehicle = 0;
        exit_fail_count = 0;
        exit_success_count = 0;

        Parameter = GetParameters(Simulation.Setting);
        GetWindow(Parameter.Map,Simulation.Setting)
        Parameter.Trajectory = GetTrajectory(Parameter.Map,Simulation.Setting);

        if Simulation.Setting.RecordVideo
            timestamp = datestr(now, 'HH-MM');
            % videoFilename = Simulation.Setting.VideoPath(participantModes{mode_idx}, randomSeed, timestamp);
            % videoFilename = filename;
            videoFilename = fullfile(Simulation.Setting.SaveFolder, ['\video\Seed5_' greedy_status2 '_' Simulation.Setting.Util_type '_' timestamp '_' Simulation.Setting.NumberOfParticipants]);
            videoWriter = VideoWriter(videoFilename, 'MPEG-4');
            videoWriter.FrameRate = 30; 
            open(videoWriter);
        end

        fprintf('%d th Iteration. Random Seed is %d.\n', Iteration, randomSeed)

        SetMap(Parameter.Map, Simulation.Setting);


        if Simulation.Setting.RecordLog
            fileID = fopen(logFileName, 'a', 'n', 'utf-8');  % append 모드로 파일 열기
            fprintf(fileID, '\n=====   Random Seed  %d  ||  %s   ===== %s \n', ...
                    randomSeed, Simulation.Setting.NumberOfParticipants, datestr(now, 'HH시 MM분 SS초'));
            
            % fprintf(fileID, '\n| Lanes:  %d  | Vehicles: %d  | Exits: %d  |\n', ...
            %     Parameter.Map.Lane, TotalVehicles, length(Parameter.Map.Exit));
            fclose(fileID);
        end


        NextArrivalTime = zeros(Parameter.Map.Lane, 1); % 5차선 기준 [0;0;0;0;0]
        %NextArrivalTime = (3600 / Parameter.Flow) * rand(Parameter.Map.Lane, 1);
        %disp(NextArrivalTime);
        TotalVehicles = 0;
        firstCount = 0;
        SpawnVehicle = [];
        SpawnLanes = [];

        for Time = 0:Parameter.Physics:Parameter.Sim.Time
            GRAPE_done = 0;            
            % 제목 출력
            title(sprintf('Random Seed: %d   |   %s   |   Participants Mode: %s   |   Time: %.2f s', ...
                randomSeed, greedy_status, strrep(participantModes{mode_idx}, '_', ' '), Time));


            if Simulation.Setting.SpawnType == 0 
                SpawnLanes = find(NextArrivalTime < Time+Parameter.Physics); % 차량 스폰 필요한 차선
                SpawnCount = length(SpawnLanes); % 차량 스폰 필요한 차선의 개수

                if SpawnCount > 0
                    [SpawnVehicle, NextArrivalTime] = GetSeed(Simulation.Setting, Parameter, TotalVehicles, SpawnLanes, NextArrivalTime);
                    TotalVehicles = TotalVehicles + SpawnCount;
                    if firstCount == 0
                        List.Vehicle.Object = cell(size(SpawnVehicle,2),1);
                        firstCount = 1;
                    end
                    List.Vehicle.Object = cat(1, List.Vehicle.Object, cell(size(SpawnVehicle,2),1));
                end

                % Generate Vehicles
                while ~isempty(SpawnVehicle)
                    List.Vehicle.Object{SpawnVehicle(1,1)} = Vehicle(SpawnVehicle(:,1),Time,Parameter);
                    SpawnVehicle = SpawnVehicle(:,2:end);  % 생성된 차량 삭제
                end

            elseif Simulation.Setting.SpawnType == 1 || Simulation.Setting.SpawnType == 2
                if firstCount == 0
                    [SpawnVehicle, TotalVehicles] = GetSeed(Simulation.Setting, Parameter, TotalVehicles, SpawnLanes, NextArrivalTime);
                    List.Vehicle.Object = cell(size(SpawnVehicle,2),1);
                    firstCount = 1;
                end
                if ~isempty(SpawnVehicle) 
                    if int32(Time/Parameter.Physics) == int32(SpawnVehicle(6,1)/Parameter.Physics)
                        List.Vehicle.Object{SpawnVehicle(1,1)} = Vehicle(SpawnVehicle(:,1),Time,Parameter);
                        if ~isempty(SpawnVehicle)
                            SpawnVehicle = SpawnVehicle(:,2:end);
                        end
                    end
                end
            end

            % Update Vehicle Data
            List.Vehicle.Data = UpdateData(List.Vehicle.Object,Parameter.Sim.Data);
            List.Vehicle.Active = List.Vehicle.Data(List.Vehicle.Data(:,2)>0,:);
            List.Vehicle.Object = GetAcceleration(List.Vehicle.Object, List.Vehicle.Data, Parameter.Veh);

            if Simulation.Setting.GreedyAlloc
                environment = GRAPE_main(List,Parameter,Simulation.Setting,Iteration);
                lane_alloc = GRAPE_instance(environment).Alloc;

            elseif mod(Time, cycle_GRAPE) == cycle_GRAPE-1 && size(List.Vehicle.Active,1)>0  %&& Time > 8
                disp("calling Grape Instance. . . | "+ Time);
                environment = GRAPE_main(List,Parameter,Simulation.Setting,Iteration);
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
        
            % Move Vehicle
            for i = 1:size(List.Vehicle.Active,1)
                vehicle_id = List.Vehicle.Active(i, 1); 
                current_vehicle = List.Vehicle.Object{vehicle_id};
                current_lane = List.Vehicle.Object{vehicle_id}.Lane; 
                
                if GRAPE_done == 1 || Simulation.Setting.GreedyAlloc
                    desired_lane = lane_alloc(i);
                
                    if current_lane ~= desired_lane 
                        %List.Vehicle.Object{vehicle_id}.TargetLane = desired_lane;
                        %List.Vehicle.Object{vehicle_id}.LaneChangeFlag = 1; 
                        if current_lane > desired_lane
                            desired_lane = current_lane - 1;
                        elseif current_lane < desired_lane
                            desired_lane = current_lane + 1;
                        end

                        if strcmp(Simulation.Setting.LaneChangeMode, 'MOBIL')
                            [feasible, a_c_sim] = MOBIL(current_vehicle, desired_lane, List, Parameter);
                        elseif strcmp(Simulation.Setting.LaneChangeMode, 'SimpleLaneChange')
                            [feasible] = SimpleLaneChange(current_vehicle, desired_lane, List, Parameter);
                        end
                        
                        % warm up 구간 동안은 차선 변경 안 되게 설정정
                        if current_vehicle.Location * Parameter.Map.Scale < 200
                            feasible = false;
                        end

                        if feasible %&& Simulation.Setting.GreedyAlloc
                            if current_vehicle.IsChangingLane 
                                current_vehicle.LaneChangeFlag = 0;
                            else
                                current_vehicle.TargetLane = desired_lane;
                                current_vehicle.LaneChangeFlag = 1;
                            end
                        %elseif feasible
                        %    current_vehicle.TargetLane = desired_lane;
                        %    current_vehicle.LaneChangeFlag = 1;
                        else
                            current_vehicle.LaneChangeFlag = 0;
                        end
                    end
                end
                
                if List.Vehicle.Object{vehicle_id}.Exit - List.Vehicle.Object{vehicle_id}.Location * Parameter.Map.Scale <= Parameter.ExitThreshold 
                    if current_lane == Parameter.Map.Lane
                        List.Vehicle.Object{vehicle_id}.ExitState = 1;
                    else
                        List.Vehicle.Object{vehicle_id}.ExitState = 0;
                    end
                end

                MoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)},Time,Parameter,List)
            end
        
            % Remove Processed Vehicles
            for i = 1:size(List.Vehicle.Active,1)
                %if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location >= 300000 % exit으로 바꾸기
                %    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                %    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];

                %    % record travel time, avg speed

                %end
                if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale > Parameter.Map.Road % through vehicles
                    spawn_time = List.Vehicle.Object{List.Vehicle.Active(i,1)}.EntryTime;
                    exit_time = Time;
                    travel_time = exit_time - spawn_time;
                    exit_location = List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale;

                    travel_times = [travel_times, travel_time];
                    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];
                    TotalVehicles = TotalVehicles - 1;
                

                elseif List.Vehicle.Object{List.Vehicle.Active(i,1)}.ExitState >= 0 && List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale >= List.Vehicle.Object{List.Vehicle.Active(i,1)}.Exit - 2 
                    % record travel time, avg speed
                    spawn_time = List.Vehicle.Object{List.Vehicle.Active(i,1)}.EntryTime;
                    exit_time = Time;
                    travel_time = exit_time - spawn_time;
                    exit_location = List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale;

                    travel_times = [travel_times, travel_time];

                    % 🔹 Exit 성공 차량인지 확인
                    if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Lane == Parameter.Map.Lane
                        exit_success_count = exit_success_count + 1;
                    else
                        exit_fail_count = exit_fail_count + 1;  % 🔹 최우측 차선이 아니면 exit fail로 기록
                    end

                    % remove vehicle
                    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];
                end
                
            end

        
            % Finalize Time Step
            if Simulation.Setting.Draw == 1
                drawnow();

                pause(Simulation.Setting.PauseTime);

                if Simulation.Setting.RecordVideo == 1 % && mod(int32(Time/Parameter.Physics), 2) == 0
                    frame = getframe(gcf); 
                    % frame.cdata = imresize(frame.cdata, 0.5);
                    writeVideo(videoWriter, frame); 
                end
            end
            if Simulation.Setting.SpawnType && isempty(List.Vehicle.Active) && isempty(SpawnVehicle)
                break
            end

        end

        total_exited_vehicles = exit_success_count + exit_fail_count;
        if TotalVehicles ~= total_exited_vehicles
            disp("something wrong here");
        end
        if total_exited_vehicles > 0
            exit_fail_rate = round(exit_fail_count / total_exited_vehicles, 3); % 🔹 소수점 3자리까지
        else
            exit_fail_rate = NaN; % 🔹 차량이 하나도 없으면 NaN
        end

        avg_travel_time = round(mean(travel_times), 3);
        if isempty(travel_times)
            avg_travel_time = NaN;
        end

        % 🔹 각 mode_idx마다 결과 저장 (Avg Travel Time + Exit Fail Rate)
        result_row{2+mode_idx} = avg_travel_time;
        result_row{2+num_modes+mode_idx} = exit_fail_rate;

        disp("Iteration: " + Iteration)
        if Time > Parameter.Sim.Time - Parameter.Physics
            ;
        end


        if Simulation.Setting.RecordVideo
            close(videoWriter);
            disp(['Simulation video saved to: ', videoFilename]);
        end

        clear Parameter List Seed environment GRAPE_output;

    end
    result_row{2} = TotalVehicles;
    results(Iteration, :) = result_row;
end
% 🔹 모든 Iteration이 끝난 후, 엑셀 파일에 한 번에 결과 저장
if Simulation.Setting.RecordExcel
    writecell(results, filename, 'Sheet', sheet, 'WriteMode', 'append');
    disp(['✅ Simulation results saved to: ', filename]);
end


%GGGGGGGGGGGGGGGGGGGGGGGGG

Simulation.Setting.SpawnType = 1; % 0: Automatically spawn vehicles based on flow rate, 1: Manually define spawn times, 2: Debug mode
Simulation.Setting.GreedyAlloc = 1; % 0: Distributed Mutex is applied (GRAPE), 1: Agents make fully greedy decisions (Baseline)

Simulation.Setting.Util_type = 'GS'; 
%Simulation.Setting.Util_type = 'HOS'; 
%Simulation.Setting.Util_type = 'FOS'; 
%Simulation.Setting.Util_type = 'ES'; 
Simulation.Setting.LaneChangeMode = 'SimpleLaneChange'; % 'MOBIL' or 'SimpleLaneChange'


%% Run Simulation
% Initialize Log File
if Simulation.Setting.RecordLog
    finalRandomSeed = Simulation.Setting.InitialRandomSeed + Simulation.Setting.Iterations - 1;
    logFileName = Simulation.Setting.LogPath(finalRandomSeed);

    % 새 로그 파일 생성 및 헤더 작성
    fileID = fopen(logFileName, 'w');
    fprintf(fileID, 'Simulation Log\n');
    fclose(fileID);
end

if Simulation.Setting.SpawnType % If vehicles are spawned manually based on predefined times
    Simulation.Setting.Time = 10000; % Set a very high simulation time to allow all vehicles to spawn
end

% GreedyAlloc 여부를 아이콘으로 변환
if Simulation.Setting.GreedyAlloc == 1
    greedy_status = 'GRAPE ❌';
    greedy_status2 = 'Greedy';
else
    greedy_status = 'GRAPE ⭕';
    greedy_status2 = 'GRAPE';
end

% 🔹 엑셀 파일 경로 설정
timestamp = datestr(now, 'HH-MM');  % 현재 시간 가져오기 (시-분-초 형식)
filename = fullfile(Simulation.Setting.SaveFolder, ['nht_' greedy_status2 '_' Simulation.Setting.Util_type '_' timestamp '.xlsx']);
sheet = 'Results';

% 🔹 실험할 참가자 모드 설정
participantModes = {'Default', 'Ahead'};  % 기본 모드
%participantModes = {};
% 🔹 Bubble Radius 값에 따라 Bubble 관련 모드 추가
for r = Simulation.Setting.BubbleRadiusList
    participantModes{end+1} = sprintf('Bubble_%dm', r);
    participantModes{end+1} = sprintf('BubbleAhead_%dm', r);
end
num_modes = length(participantModes);

% 🔹 엑셀 파일이 존재하지 않으면 헤더만 추가하여 생성
if Simulation.Setting.RecordExcel && ~isfile(filename)
    if isstring(participantModes)
        participantModes = cellstr(participantModes);
    end
    header = [{'Random Seed'}, {'Total Vehicles'}, participantModes, strcat("ExitFail_", participantModes)];
    writematrix(header, filename, 'Sheet', sheet, 'WriteMode', 'overwrite')
    disp(['New Excel file created: ', filename]);
end

% 🔹 결과 저장을 위한 배열 초기화
if Simulation.Setting.RecordExcel
    num_simulations = Simulation.Setting.Iterations;
    results = cell(num_simulations, (num_modes * 2) + 2); % random seed, total vehicles, <avg travel time, fail rate> each mode
end

for Iteration = 1:Simulation.Setting.Iterations
    randomSeed = Simulation.Setting.InitialRandomSeed + Iteration - 1;
    rng(randomSeed)
    %Simulation.Setting.RandomSeed = randomSeed;

    % 현재 random seed에 대한 결과 저장할 행 초기화
    result_row = cell(1, (num_modes * 2) + 2);
    result_row{1} = randomSeed;  % 첫 번째 칸에 random seed 저장


    for mode_idx = 1:num_modes
        close all
        rng(randomSeed)
        Simulation.Setting.NumberOfParticipants = participantModes{mode_idx};

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

        disp(['Running ', participantModes{mode_idx}, ' mode, Random Seed ', num2str(randomSeed)]);

        environment = struct();
        GRAPE_output = [];
        travel_times = [];
        exit_fail = 0;
        RemovedVehicle = 0;
        exit_fail_count = 0;
        exit_success_count = 0;

        Parameter = GetParameters(Simulation.Setting);
        GetWindow(Parameter.Map,Simulation.Setting)
        Parameter.Trajectory = GetTrajectory(Parameter.Map,Simulation.Setting);

        if Simulation.Setting.RecordVideo
            timestamp = datestr(now, 'HH-MM');
            % videoFilename = Simulation.Setting.VideoPath(participantModes{mode_idx}, randomSeed, timestamp);
            % videoFilename = filename;
            videoFilename = fullfile(Simulation.Setting.SaveFolder, ['\video\Seed5_' greedy_status2 '_' Simulation.Setting.Util_type '_' timestamp '_' Simulation.Setting.NumberOfParticipants]);
            videoWriter = VideoWriter(videoFilename, 'MPEG-4');
            videoWriter.FrameRate = 30; 
            open(videoWriter);
        end

        fprintf('%d th Iteration. Random Seed is %d.\n', Iteration, randomSeed)

        SetMap(Parameter.Map, Simulation.Setting);


        if Simulation.Setting.RecordLog
            fileID = fopen(logFileName, 'a', 'n', 'utf-8');  % append 모드로 파일 열기
            fprintf(fileID, '\n=====   Random Seed  %d  ||  %s   ===== %s \n', ...
                    randomSeed, Simulation.Setting.NumberOfParticipants, datestr(now, 'HH시 MM분 SS초'));
            
            % fprintf(fileID, '\n| Lanes:  %d  | Vehicles: %d  | Exits: %d  |\n', ...
            %     Parameter.Map.Lane, TotalVehicles, length(Parameter.Map.Exit));
            fclose(fileID);
        end


        NextArrivalTime = zeros(Parameter.Map.Lane, 1); % 5차선 기준 [0;0;0;0;0]
        %NextArrivalTime = (3600 / Parameter.Flow) * rand(Parameter.Map.Lane, 1);
        %disp(NextArrivalTime);
        TotalVehicles = 0;
        firstCount = 0;
        SpawnVehicle = [];
        SpawnLanes = [];

        for Time = 0:Parameter.Physics:Parameter.Sim.Time
            GRAPE_done = 0;            
            % 제목 출력
            title(sprintf('Random Seed: %d   |   %s   |   Participants Mode: %s   |   Time: %.2f s', ...
                randomSeed, greedy_status, strrep(participantModes{mode_idx}, '_', ' '), Time));


            if Simulation.Setting.SpawnType == 0 
                SpawnLanes = find(NextArrivalTime < Time+Parameter.Physics); % 차량 스폰 필요한 차선
                SpawnCount = length(SpawnLanes); % 차량 스폰 필요한 차선의 개수

                if SpawnCount > 0
                    [SpawnVehicle, NextArrivalTime] = GetSeed(Simulation.Setting, Parameter, TotalVehicles, SpawnLanes, NextArrivalTime);
                    TotalVehicles = TotalVehicles + SpawnCount;
                    if firstCount == 0
                        List.Vehicle.Object = cell(size(SpawnVehicle,2),1);
                        firstCount = 1;
                    end
                    List.Vehicle.Object = cat(1, List.Vehicle.Object, cell(size(SpawnVehicle,2),1));
                end

                % Generate Vehicles
                while ~isempty(SpawnVehicle)
                    List.Vehicle.Object{SpawnVehicle(1,1)} = Vehicle(SpawnVehicle(:,1),Time,Parameter);
                    SpawnVehicle = SpawnVehicle(:,2:end);  % 생성된 차량 삭제
                end

            elseif Simulation.Setting.SpawnType == 1 || Simulation.Setting.SpawnType == 2
                if firstCount == 0
                    [SpawnVehicle, TotalVehicles] = GetSeed(Simulation.Setting, Parameter, TotalVehicles, SpawnLanes, NextArrivalTime);
                    List.Vehicle.Object = cell(size(SpawnVehicle,2),1);
                    firstCount = 1;
                end
                if ~isempty(SpawnVehicle) 
                    if int32(Time/Parameter.Physics) == int32(SpawnVehicle(6,1)/Parameter.Physics)
                        List.Vehicle.Object{SpawnVehicle(1,1)} = Vehicle(SpawnVehicle(:,1),Time,Parameter);
                        if ~isempty(SpawnVehicle)
                            SpawnVehicle = SpawnVehicle(:,2:end);
                        end
                    end
                end
            end

            % Update Vehicle Data
            List.Vehicle.Data = UpdateData(List.Vehicle.Object,Parameter.Sim.Data);
            List.Vehicle.Active = List.Vehicle.Data(List.Vehicle.Data(:,2)>0,:);
            List.Vehicle.Object = GetAcceleration(List.Vehicle.Object, List.Vehicle.Data, Parameter.Veh);

            if Simulation.Setting.GreedyAlloc
                environment = GRAPE_main(List,Parameter,Simulation.Setting,Iteration);
                lane_alloc = GRAPE_instance(environment).Alloc;

            elseif mod(Time, cycle_GRAPE) == cycle_GRAPE-1 && size(List.Vehicle.Active,1)>0  %&& Time > 8
                disp("calling Grape Instance. . . | "+ Time);
                environment = GRAPE_main(List,Parameter,Simulation.Setting,Iteration);
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
        
            % Move Vehicle
            for i = 1:size(List.Vehicle.Active,1)
                vehicle_id = List.Vehicle.Active(i, 1); 
                current_vehicle = List.Vehicle.Object{vehicle_id};
                current_lane = List.Vehicle.Object{vehicle_id}.Lane; 
                
                if GRAPE_done == 1 || Simulation.Setting.GreedyAlloc
                    desired_lane = lane_alloc(i);
                
                    if current_lane ~= desired_lane 
                        %List.Vehicle.Object{vehicle_id}.TargetLane = desired_lane;
                        %List.Vehicle.Object{vehicle_id}.LaneChangeFlag = 1; 
                        if current_lane > desired_lane
                            desired_lane = current_lane - 1;
                        elseif current_lane < desired_lane
                            desired_lane = current_lane + 1;
                        end

                        if strcmp(Simulation.Setting.LaneChangeMode, 'MOBIL')
                            [feasible, a_c_sim] = MOBIL(current_vehicle, desired_lane, List, Parameter);
                        elseif strcmp(Simulation.Setting.LaneChangeMode, 'SimpleLaneChange')
                            [feasible] = SimpleLaneChange(current_vehicle, desired_lane, List, Parameter);
                        end
                        
                        % warm up 구간 동안은 차선 변경 안 되게 설정정
                        if current_vehicle.Location * Parameter.Map.Scale < 200
                            feasible = false;
                        end

                        if feasible %&& Simulation.Setting.GreedyAlloc
                            if current_vehicle.IsChangingLane 
                                current_vehicle.LaneChangeFlag = 0;
                            else
                                current_vehicle.TargetLane = desired_lane;
                                current_vehicle.LaneChangeFlag = 1;
                            end
                        %elseif feasible
                        %    current_vehicle.TargetLane = desired_lane;
                        %    current_vehicle.LaneChangeFlag = 1;
                        else
                            current_vehicle.LaneChangeFlag = 0;
                        end
                    end
                end
                
                if List.Vehicle.Object{vehicle_id}.Exit - List.Vehicle.Object{vehicle_id}.Location * Parameter.Map.Scale <= Parameter.ExitThreshold 
                    if current_lane == Parameter.Map.Lane
                        List.Vehicle.Object{vehicle_id}.ExitState = 1;
                    else
                        List.Vehicle.Object{vehicle_id}.ExitState = 0;
                    end
                end

                MoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)},Time,Parameter,List)
            end
        
            % Remove Processed Vehicles
            for i = 1:size(List.Vehicle.Active,1)
                %if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location >= 300000 % exit으로 바꾸기
                %    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                %    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];

                %    % record travel time, avg speed

                %end
                if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale > Parameter.Map.Road % through vehicles
                    spawn_time = List.Vehicle.Object{List.Vehicle.Active(i,1)}.EntryTime;
                    exit_time = Time;
                    travel_time = exit_time - spawn_time;
                    exit_location = List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale;

                    travel_times = [travel_times, travel_time];
                    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];
                    TotalVehicles = TotalVehicles - 1;
                

                elseif List.Vehicle.Object{List.Vehicle.Active(i,1)}.ExitState >= 0 && List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale >= List.Vehicle.Object{List.Vehicle.Active(i,1)}.Exit - 2 
                    % record travel time, avg speed
                    spawn_time = List.Vehicle.Object{List.Vehicle.Active(i,1)}.EntryTime;
                    exit_time = Time;
                    travel_time = exit_time - spawn_time;
                    exit_location = List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale;

                    travel_times = [travel_times, travel_time];

                    % 🔹 Exit 성공 차량인지 확인
                    if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Lane == Parameter.Map.Lane
                        exit_success_count = exit_success_count + 1;
                    else
                        exit_fail_count = exit_fail_count + 1;  % 🔹 최우측 차선이 아니면 exit fail로 기록
                    end

                    % remove vehicle
                    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];
                end
                
            end

        
            % Finalize Time Step
            if Simulation.Setting.Draw == 1
                drawnow();

                pause(Simulation.Setting.PauseTime);

                if Simulation.Setting.RecordVideo == 1 % && mod(int32(Time/Parameter.Physics), 2) == 0
                    frame = getframe(gcf); 
                    % frame.cdata = imresize(frame.cdata, 0.5);
                    writeVideo(videoWriter, frame); 
                end
            end
            if Simulation.Setting.SpawnType && isempty(List.Vehicle.Active) && isempty(SpawnVehicle)
                break
            end

        end

        total_exited_vehicles = exit_success_count + exit_fail_count;
        if TotalVehicles ~= total_exited_vehicles
            disp("something wrong here");
        end
        if total_exited_vehicles > 0
            exit_fail_rate = round(exit_fail_count / total_exited_vehicles, 3); % 🔹 소수점 3자리까지
        else
            exit_fail_rate = NaN; % 🔹 차량이 하나도 없으면 NaN
        end

        avg_travel_time = round(mean(travel_times), 3);
        if isempty(travel_times)
            avg_travel_time = NaN;
        end

        % 🔹 각 mode_idx마다 결과 저장 (Avg Travel Time + Exit Fail Rate)
        result_row{2+mode_idx} = avg_travel_time;
        result_row{2+num_modes+mode_idx} = exit_fail_rate;

        disp("Iteration: " + Iteration)
        if Time > Parameter.Sim.Time - Parameter.Physics
            ;
        end


        if Simulation.Setting.RecordVideo
            close(videoWriter);
            disp(['Simulation video saved to: ', videoFilename]);
        end

        clear Parameter List Seed environment GRAPE_output;

    end
    result_row{2} = TotalVehicles;
    results(Iteration, :) = result_row;
end
% 🔹 모든 Iteration이 끝난 후, 엑셀 파일에 한 번에 결과 저장
if Simulation.Setting.RecordExcel
    writecell(results, filename, 'Sheet', sheet, 'WriteMode', 'append');
    disp(['✅ Simulation results saved to: ', filename]);
end

%33333333333333333

%Simulation.Setting.Util_type = 'GS'; 
Simulation.Setting.Util_type = 'HOS'; 
%Simulation.Setting.Util_type = 'FOS'; 
%Simulation.Setting.Util_type = 'ES'; 
Simulation.Setting.LaneChangeMode = 'SimpleLaneChange'; % 'MOBIL' or 'SimpleLaneChange'


%% Run Simulation
% Initialize Log File
if Simulation.Setting.RecordLog
    finalRandomSeed = Simulation.Setting.InitialRandomSeed + Simulation.Setting.Iterations - 1;
    logFileName = Simulation.Setting.LogPath(finalRandomSeed);

    % 새 로그 파일 생성 및 헤더 작성
    fileID = fopen(logFileName, 'w');
    fprintf(fileID, 'Simulation Log\n');
    fclose(fileID);
end

if Simulation.Setting.SpawnType % If vehicles are spawned manually based on predefined times
    Simulation.Setting.Time = 10000; % Set a very high simulation time to allow all vehicles to spawn
end

% GreedyAlloc 여부를 아이콘으로 변환
if Simulation.Setting.GreedyAlloc == 1
    greedy_status = 'GRAPE ❌';
    greedy_status2 = 'Greedy';
else
    greedy_status = 'GRAPE ⭕';
    greedy_status2 = 'GRAPE';
end

% 🔹 엑셀 파일 경로 설정
timestamp = datestr(now, 'HH-MM');  % 현재 시간 가져오기 (시-분-초 형식)
filename = fullfile(Simulation.Setting.SaveFolder, ['nht_' greedy_status2 '_' Simulation.Setting.Util_type '_' timestamp '.xlsx']);
sheet = 'Results';

% 🔹 실험할 참가자 모드 설정
participantModes = {'Default', 'Ahead'};  % 기본 모드
%participantModes = {};
% 🔹 Bubble Radius 값에 따라 Bubble 관련 모드 추가
for r = Simulation.Setting.BubbleRadiusList
    participantModes{end+1} = sprintf('Bubble_%dm', r);
    participantModes{end+1} = sprintf('BubbleAhead_%dm', r);
end
num_modes = length(participantModes);

% 🔹 엑셀 파일이 존재하지 않으면 헤더만 추가하여 생성
if Simulation.Setting.RecordExcel && ~isfile(filename)
    if isstring(participantModes)
        participantModes = cellstr(participantModes);
    end
    header = [{'Random Seed'}, {'Total Vehicles'}, participantModes, strcat("ExitFail_", participantModes)];
    writematrix(header, filename, 'Sheet', sheet, 'WriteMode', 'overwrite')
    disp(['New Excel file created: ', filename]);
end

% 🔹 결과 저장을 위한 배열 초기화
if Simulation.Setting.RecordExcel
    num_simulations = Simulation.Setting.Iterations;
    results = cell(num_simulations, (num_modes * 2) + 2); % random seed, total vehicles, <avg travel time, fail rate> each mode
end

for Iteration = 1:Simulation.Setting.Iterations
    randomSeed = Simulation.Setting.InitialRandomSeed + Iteration - 1;
    rng(randomSeed)
    %Simulation.Setting.RandomSeed = randomSeed;

    % 현재 random seed에 대한 결과 저장할 행 초기화
    result_row = cell(1, (num_modes * 2) + 2);
    result_row{1} = randomSeed;  % 첫 번째 칸에 random seed 저장


    for mode_idx = 1:num_modes
        close all
        rng(randomSeed)
        Simulation.Setting.NumberOfParticipants = participantModes{mode_idx};

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

        disp(['Running ', participantModes{mode_idx}, ' mode, Random Seed ', num2str(randomSeed)]);

        environment = struct();
        GRAPE_output = [];
        travel_times = [];
        exit_fail = 0;
        RemovedVehicle = 0;
        exit_fail_count = 0;
        exit_success_count = 0;

        Parameter = GetParameters(Simulation.Setting);
        GetWindow(Parameter.Map,Simulation.Setting)
        Parameter.Trajectory = GetTrajectory(Parameter.Map,Simulation.Setting);

        if Simulation.Setting.RecordVideo
            timestamp = datestr(now, 'HH-MM');
            % videoFilename = Simulation.Setting.VideoPath(participantModes{mode_idx}, randomSeed, timestamp);
            % videoFilename = filename;
            videoFilename = fullfile(Simulation.Setting.SaveFolder, ['\video\Seed5_' greedy_status2 '_' Simulation.Setting.Util_type '_' timestamp '_' Simulation.Setting.NumberOfParticipants]);
            videoWriter = VideoWriter(videoFilename, 'MPEG-4');
            videoWriter.FrameRate = 30; 
            open(videoWriter);
        end

        fprintf('%d th Iteration. Random Seed is %d.\n', Iteration, randomSeed)

        SetMap(Parameter.Map, Simulation.Setting);


        if Simulation.Setting.RecordLog
            fileID = fopen(logFileName, 'a', 'n', 'utf-8');  % append 모드로 파일 열기
            fprintf(fileID, '\n=====   Random Seed  %d  ||  %s   ===== %s \n', ...
                    randomSeed, Simulation.Setting.NumberOfParticipants, datestr(now, 'HH시 MM분 SS초'));
            
            % fprintf(fileID, '\n| Lanes:  %d  | Vehicles: %d  | Exits: %d  |\n', ...
            %     Parameter.Map.Lane, TotalVehicles, length(Parameter.Map.Exit));
            fclose(fileID);
        end


        NextArrivalTime = zeros(Parameter.Map.Lane, 1); % 5차선 기준 [0;0;0;0;0]
        %NextArrivalTime = (3600 / Parameter.Flow) * rand(Parameter.Map.Lane, 1);
        %disp(NextArrivalTime);
        TotalVehicles = 0;
        firstCount = 0;
        SpawnVehicle = [];
        SpawnLanes = [];

        for Time = 0:Parameter.Physics:Parameter.Sim.Time
            GRAPE_done = 0;            
            % 제목 출력
            title(sprintf('Random Seed: %d   |   %s   |   Participants Mode: %s   |   Time: %.2f s', ...
                randomSeed, greedy_status, strrep(participantModes{mode_idx}, '_', ' '), Time));


            if Simulation.Setting.SpawnType == 0 
                SpawnLanes = find(NextArrivalTime < Time+Parameter.Physics); % 차량 스폰 필요한 차선
                SpawnCount = length(SpawnLanes); % 차량 스폰 필요한 차선의 개수

                if SpawnCount > 0
                    [SpawnVehicle, NextArrivalTime] = GetSeed(Simulation.Setting, Parameter, TotalVehicles, SpawnLanes, NextArrivalTime);
                    TotalVehicles = TotalVehicles + SpawnCount;
                    if firstCount == 0
                        List.Vehicle.Object = cell(size(SpawnVehicle,2),1);
                        firstCount = 1;
                    end
                    List.Vehicle.Object = cat(1, List.Vehicle.Object, cell(size(SpawnVehicle,2),1));
                end

                % Generate Vehicles
                while ~isempty(SpawnVehicle)
                    List.Vehicle.Object{SpawnVehicle(1,1)} = Vehicle(SpawnVehicle(:,1),Time,Parameter);
                    SpawnVehicle = SpawnVehicle(:,2:end);  % 생성된 차량 삭제
                end

            elseif Simulation.Setting.SpawnType == 1 || Simulation.Setting.SpawnType == 2
                if firstCount == 0
                    [SpawnVehicle, TotalVehicles] = GetSeed(Simulation.Setting, Parameter, TotalVehicles, SpawnLanes, NextArrivalTime);
                    List.Vehicle.Object = cell(size(SpawnVehicle,2),1);
                    firstCount = 1;
                end
                if ~isempty(SpawnVehicle) 
                    if int32(Time/Parameter.Physics) == int32(SpawnVehicle(6,1)/Parameter.Physics)
                        List.Vehicle.Object{SpawnVehicle(1,1)} = Vehicle(SpawnVehicle(:,1),Time,Parameter);
                        if ~isempty(SpawnVehicle)
                            SpawnVehicle = SpawnVehicle(:,2:end);
                        end
                    end
                end
            end

            % Update Vehicle Data
            List.Vehicle.Data = UpdateData(List.Vehicle.Object,Parameter.Sim.Data);
            List.Vehicle.Active = List.Vehicle.Data(List.Vehicle.Data(:,2)>0,:);
            List.Vehicle.Object = GetAcceleration(List.Vehicle.Object, List.Vehicle.Data, Parameter.Veh);

            if Simulation.Setting.GreedyAlloc
                environment = GRAPE_main(List,Parameter,Simulation.Setting,Iteration);
                lane_alloc = GRAPE_instance(environment).Alloc;

            elseif mod(Time, cycle_GRAPE) == cycle_GRAPE-1 && size(List.Vehicle.Active,1)>0  %&& Time > 8
                disp("calling Grape Instance. . . | "+ Time);
                environment = GRAPE_main(List,Parameter,Simulation.Setting,Iteration);
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
        
            % Move Vehicle
            for i = 1:size(List.Vehicle.Active,1)
                vehicle_id = List.Vehicle.Active(i, 1); 
                current_vehicle = List.Vehicle.Object{vehicle_id};
                current_lane = List.Vehicle.Object{vehicle_id}.Lane; 
                
                if GRAPE_done == 1 || Simulation.Setting.GreedyAlloc
                    desired_lane = lane_alloc(i);
                
                    if current_lane ~= desired_lane 
                        %List.Vehicle.Object{vehicle_id}.TargetLane = desired_lane;
                        %List.Vehicle.Object{vehicle_id}.LaneChangeFlag = 1; 
                        if current_lane > desired_lane
                            desired_lane = current_lane - 1;
                        elseif current_lane < desired_lane
                            desired_lane = current_lane + 1;
                        end

                        if strcmp(Simulation.Setting.LaneChangeMode, 'MOBIL')
                            [feasible, a_c_sim] = MOBIL(current_vehicle, desired_lane, List, Parameter);
                        elseif strcmp(Simulation.Setting.LaneChangeMode, 'SimpleLaneChange')
                            [feasible] = SimpleLaneChange(current_vehicle, desired_lane, List, Parameter);
                        end
                        
                        % warm up 구간 동안은 차선 변경 안 되게 설정정
                        if current_vehicle.Location * Parameter.Map.Scale < 200
                            feasible = false;
                        end

                        if feasible %&& Simulation.Setting.GreedyAlloc
                            if current_vehicle.IsChangingLane 
                                current_vehicle.LaneChangeFlag = 0;
                            else
                                current_vehicle.TargetLane = desired_lane;
                                current_vehicle.LaneChangeFlag = 1;
                            end
                        %elseif feasible
                        %    current_vehicle.TargetLane = desired_lane;
                        %    current_vehicle.LaneChangeFlag = 1;
                        else
                            current_vehicle.LaneChangeFlag = 0;
                        end
                    end
                end
                
                if List.Vehicle.Object{vehicle_id}.Exit - List.Vehicle.Object{vehicle_id}.Location * Parameter.Map.Scale <= Parameter.ExitThreshold 
                    if current_lane == Parameter.Map.Lane
                        List.Vehicle.Object{vehicle_id}.ExitState = 1;
                    else
                        List.Vehicle.Object{vehicle_id}.ExitState = 0;
                    end
                end

                MoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)},Time,Parameter,List)
            end
        
            % Remove Processed Vehicles
            for i = 1:size(List.Vehicle.Active,1)
                %if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location >= 300000 % exit으로 바꾸기
                %    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                %    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];

                %    % record travel time, avg speed

                %end
                if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale > Parameter.Map.Road % through vehicles
                    spawn_time = List.Vehicle.Object{List.Vehicle.Active(i,1)}.EntryTime;
                    exit_time = Time;
                    travel_time = exit_time - spawn_time;
                    exit_location = List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale;

                    travel_times = [travel_times, travel_time];
                    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];
                    TotalVehicles = TotalVehicles - 1;
                

                elseif List.Vehicle.Object{List.Vehicle.Active(i,1)}.ExitState >= 0 && List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale >= List.Vehicle.Object{List.Vehicle.Active(i,1)}.Exit - 2 
                    % record travel time, avg speed
                    spawn_time = List.Vehicle.Object{List.Vehicle.Active(i,1)}.EntryTime;
                    exit_time = Time;
                    travel_time = exit_time - spawn_time;
                    exit_location = List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale;

                    travel_times = [travel_times, travel_time];

                    % 🔹 Exit 성공 차량인지 확인
                    if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Lane == Parameter.Map.Lane
                        exit_success_count = exit_success_count + 1;
                    else
                        exit_fail_count = exit_fail_count + 1;  % 🔹 최우측 차선이 아니면 exit fail로 기록
                    end

                    % remove vehicle
                    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];
                end
                
            end

        
            % Finalize Time Step
            if Simulation.Setting.Draw == 1
                drawnow();

                pause(Simulation.Setting.PauseTime);

                if Simulation.Setting.RecordVideo == 1 % && mod(int32(Time/Parameter.Physics), 2) == 0
                    frame = getframe(gcf); 
                    % frame.cdata = imresize(frame.cdata, 0.5);
                    writeVideo(videoWriter, frame); 
                end
            end
            if Simulation.Setting.SpawnType && isempty(List.Vehicle.Active) && isempty(SpawnVehicle)
                break
            end

        end

        total_exited_vehicles = exit_success_count + exit_fail_count;
        if TotalVehicles ~= total_exited_vehicles
            disp("something wrong here");
        end
        if total_exited_vehicles > 0
            exit_fail_rate = round(exit_fail_count / total_exited_vehicles, 3); % 🔹 소수점 3자리까지
        else
            exit_fail_rate = NaN; % 🔹 차량이 하나도 없으면 NaN
        end

        avg_travel_time = round(mean(travel_times), 3);
        if isempty(travel_times)
            avg_travel_time = NaN;
        end

        % 🔹 각 mode_idx마다 결과 저장 (Avg Travel Time + Exit Fail Rate)
        result_row{2+mode_idx} = avg_travel_time;
        result_row{2+num_modes+mode_idx} = exit_fail_rate;

        disp("Iteration: " + Iteration)
        if Time > Parameter.Sim.Time - Parameter.Physics
            ;
        end


        if Simulation.Setting.RecordVideo
            close(videoWriter);
            disp(['Simulation video saved to: ', videoFilename]);
        end

        clear Parameter List Seed environment GRAPE_output;

    end
    result_row{2} = TotalVehicles;
    results(Iteration, :) = result_row;
end
% 🔹 모든 Iteration이 끝난 후, 엑셀 파일에 한 번에 결과 저장
if Simulation.Setting.RecordExcel
    writecell(results, filename, 'Sheet', sheet, 'WriteMode', 'append');
    disp(['✅ Simulation results saved to: ', filename]);
end

%3333333333333333

%Simulation.Setting.Util_type = 'GS'; 
%Simulation.Setting.Util_type = 'HOS'; 
Simulation.Setting.Util_type = 'FOS'; 
%Simulation.Setting.Util_type = 'ES'; 
Simulation.Setting.LaneChangeMode = 'SimpleLaneChange'; % 'MOBIL' or 'SimpleLaneChange'


%% Run Simulation
% Initialize Log File
if Simulation.Setting.RecordLog
    finalRandomSeed = Simulation.Setting.InitialRandomSeed + Simulation.Setting.Iterations - 1;
    logFileName = Simulation.Setting.LogPath(finalRandomSeed);

    % 새 로그 파일 생성 및 헤더 작성
    fileID = fopen(logFileName, 'w');
    fprintf(fileID, 'Simulation Log\n');
    fclose(fileID);
end

if Simulation.Setting.SpawnType % If vehicles are spawned manually based on predefined times
    Simulation.Setting.Time = 10000; % Set a very high simulation time to allow all vehicles to spawn
end

% GreedyAlloc 여부를 아이콘으로 변환
if Simulation.Setting.GreedyAlloc == 1
    greedy_status = 'GRAPE ❌';
    greedy_status2 = 'Greedy';
else
    greedy_status = 'GRAPE ⭕';
    greedy_status2 = 'GRAPE';
end

% 🔹 엑셀 파일 경로 설정
timestamp = datestr(now, 'HH-MM');  % 현재 시간 가져오기 (시-분-초 형식)
filename = fullfile(Simulation.Setting.SaveFolder, ['nht_' greedy_status2 '_' Simulation.Setting.Util_type '_' timestamp '.xlsx']);
sheet = 'Results';

% 🔹 실험할 참가자 모드 설정
participantModes = {'Default', 'Ahead'};  % 기본 모드
%participantModes = {};
% 🔹 Bubble Radius 값에 따라 Bubble 관련 모드 추가
for r = Simulation.Setting.BubbleRadiusList
    participantModes{end+1} = sprintf('Bubble_%dm', r);
    participantModes{end+1} = sprintf('BubbleAhead_%dm', r);
end
num_modes = length(participantModes);

% 🔹 엑셀 파일이 존재하지 않으면 헤더만 추가하여 생성
if Simulation.Setting.RecordExcel && ~isfile(filename)
    if isstring(participantModes)
        participantModes = cellstr(participantModes);
    end
    header = [{'Random Seed'}, {'Total Vehicles'}, participantModes, strcat("ExitFail_", participantModes)];
    writematrix(header, filename, 'Sheet', sheet, 'WriteMode', 'overwrite')
    disp(['New Excel file created: ', filename]);
end

% 🔹 결과 저장을 위한 배열 초기화
if Simulation.Setting.RecordExcel
    num_simulations = Simulation.Setting.Iterations;
    results = cell(num_simulations, (num_modes * 2) + 2); % random seed, total vehicles, <avg travel time, fail rate> each mode
end

for Iteration = 1:Simulation.Setting.Iterations
    randomSeed = Simulation.Setting.InitialRandomSeed + Iteration - 1;
    rng(randomSeed)
    %Simulation.Setting.RandomSeed = randomSeed;

    % 현재 random seed에 대한 결과 저장할 행 초기화
    result_row = cell(1, (num_modes * 2) + 2);
    result_row{1} = randomSeed;  % 첫 번째 칸에 random seed 저장


    for mode_idx = 1:num_modes
        close all
        rng(randomSeed)
        Simulation.Setting.NumberOfParticipants = participantModes{mode_idx};

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

        disp(['Running ', participantModes{mode_idx}, ' mode, Random Seed ', num2str(randomSeed)]);

        environment = struct();
        GRAPE_output = [];
        travel_times = [];
        exit_fail = 0;
        RemovedVehicle = 0;
        exit_fail_count = 0;
        exit_success_count = 0;

        Parameter = GetParameters(Simulation.Setting);
        GetWindow(Parameter.Map,Simulation.Setting)
        Parameter.Trajectory = GetTrajectory(Parameter.Map,Simulation.Setting);

        if Simulation.Setting.RecordVideo
            timestamp = datestr(now, 'HH-MM');
            % videoFilename = Simulation.Setting.VideoPath(participantModes{mode_idx}, randomSeed, timestamp);
            % videoFilename = filename;
            videoFilename = fullfile(Simulation.Setting.SaveFolder, ['\video\Seed5_' greedy_status2 '_' Simulation.Setting.Util_type '_' timestamp '_' Simulation.Setting.NumberOfParticipants]);
            videoWriter = VideoWriter(videoFilename, 'MPEG-4');
            videoWriter.FrameRate = 30; 
            open(videoWriter);
        end

        fprintf('%d th Iteration. Random Seed is %d.\n', Iteration, randomSeed)

        SetMap(Parameter.Map, Simulation.Setting);


        if Simulation.Setting.RecordLog
            fileID = fopen(logFileName, 'a', 'n', 'utf-8');  % append 모드로 파일 열기
            fprintf(fileID, '\n=====   Random Seed  %d  ||  %s   ===== %s \n', ...
                    randomSeed, Simulation.Setting.NumberOfParticipants, datestr(now, 'HH시 MM분 SS초'));
            
            % fprintf(fileID, '\n| Lanes:  %d  | Vehicles: %d  | Exits: %d  |\n', ...
            %     Parameter.Map.Lane, TotalVehicles, length(Parameter.Map.Exit));
            fclose(fileID);
        end


        NextArrivalTime = zeros(Parameter.Map.Lane, 1); % 5차선 기준 [0;0;0;0;0]
        %NextArrivalTime = (3600 / Parameter.Flow) * rand(Parameter.Map.Lane, 1);
        %disp(NextArrivalTime);
        TotalVehicles = 0;
        firstCount = 0;
        SpawnVehicle = [];
        SpawnLanes = [];

        for Time = 0:Parameter.Physics:Parameter.Sim.Time
            GRAPE_done = 0;            
            % 제목 출력
            title(sprintf('Random Seed: %d   |   %s   |   Participants Mode: %s   |   Time: %.2f s', ...
                randomSeed, greedy_status, strrep(participantModes{mode_idx}, '_', ' '), Time));


            if Simulation.Setting.SpawnType == 0 
                SpawnLanes = find(NextArrivalTime < Time+Parameter.Physics); % 차량 스폰 필요한 차선
                SpawnCount = length(SpawnLanes); % 차량 스폰 필요한 차선의 개수

                if SpawnCount > 0
                    [SpawnVehicle, NextArrivalTime] = GetSeed(Simulation.Setting, Parameter, TotalVehicles, SpawnLanes, NextArrivalTime);
                    TotalVehicles = TotalVehicles + SpawnCount;
                    if firstCount == 0
                        List.Vehicle.Object = cell(size(SpawnVehicle,2),1);
                        firstCount = 1;
                    end
                    List.Vehicle.Object = cat(1, List.Vehicle.Object, cell(size(SpawnVehicle,2),1));
                end

                % Generate Vehicles
                while ~isempty(SpawnVehicle)
                    List.Vehicle.Object{SpawnVehicle(1,1)} = Vehicle(SpawnVehicle(:,1),Time,Parameter);
                    SpawnVehicle = SpawnVehicle(:,2:end);  % 생성된 차량 삭제
                end

            elseif Simulation.Setting.SpawnType == 1 || Simulation.Setting.SpawnType == 2
                if firstCount == 0
                    [SpawnVehicle, TotalVehicles] = GetSeed(Simulation.Setting, Parameter, TotalVehicles, SpawnLanes, NextArrivalTime);
                    List.Vehicle.Object = cell(size(SpawnVehicle,2),1);
                    firstCount = 1;
                end
                if ~isempty(SpawnVehicle) 
                    if int32(Time/Parameter.Physics) == int32(SpawnVehicle(6,1)/Parameter.Physics)
                        List.Vehicle.Object{SpawnVehicle(1,1)} = Vehicle(SpawnVehicle(:,1),Time,Parameter);
                        if ~isempty(SpawnVehicle)
                            SpawnVehicle = SpawnVehicle(:,2:end);
                        end
                    end
                end
            end

            % Update Vehicle Data
            List.Vehicle.Data = UpdateData(List.Vehicle.Object,Parameter.Sim.Data);
            List.Vehicle.Active = List.Vehicle.Data(List.Vehicle.Data(:,2)>0,:);
            List.Vehicle.Object = GetAcceleration(List.Vehicle.Object, List.Vehicle.Data, Parameter.Veh);

            if Simulation.Setting.GreedyAlloc
                environment = GRAPE_main(List,Parameter,Simulation.Setting,Iteration);
                lane_alloc = GRAPE_instance(environment).Alloc;

            elseif mod(Time, cycle_GRAPE) == cycle_GRAPE-1 && size(List.Vehicle.Active,1)>0  %&& Time > 8
                disp("calling Grape Instance. . . | "+ Time);
                environment = GRAPE_main(List,Parameter,Simulation.Setting,Iteration);
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
        
            % Move Vehicle
            for i = 1:size(List.Vehicle.Active,1)
                vehicle_id = List.Vehicle.Active(i, 1); 
                current_vehicle = List.Vehicle.Object{vehicle_id};
                current_lane = List.Vehicle.Object{vehicle_id}.Lane; 
                
                if GRAPE_done == 1 || Simulation.Setting.GreedyAlloc
                    desired_lane = lane_alloc(i);
                
                    if current_lane ~= desired_lane 
                        %List.Vehicle.Object{vehicle_id}.TargetLane = desired_lane;
                        %List.Vehicle.Object{vehicle_id}.LaneChangeFlag = 1; 
                        if current_lane > desired_lane
                            desired_lane = current_lane - 1;
                        elseif current_lane < desired_lane
                            desired_lane = current_lane + 1;
                        end

                        if strcmp(Simulation.Setting.LaneChangeMode, 'MOBIL')
                            [feasible, a_c_sim] = MOBIL(current_vehicle, desired_lane, List, Parameter);
                        elseif strcmp(Simulation.Setting.LaneChangeMode, 'SimpleLaneChange')
                            [feasible] = SimpleLaneChange(current_vehicle, desired_lane, List, Parameter);
                        end
                        
                        % warm up 구간 동안은 차선 변경 안 되게 설정정
                        if current_vehicle.Location * Parameter.Map.Scale < 200
                            feasible = false;
                        end

                        if feasible %&& Simulation.Setting.GreedyAlloc
                            if current_vehicle.IsChangingLane 
                                current_vehicle.LaneChangeFlag = 0;
                            else
                                current_vehicle.TargetLane = desired_lane;
                                current_vehicle.LaneChangeFlag = 1;
                            end
                        %elseif feasible
                        %    current_vehicle.TargetLane = desired_lane;
                        %    current_vehicle.LaneChangeFlag = 1;
                        else
                            current_vehicle.LaneChangeFlag = 0;
                        end
                    end
                end
                
                if List.Vehicle.Object{vehicle_id}.Exit - List.Vehicle.Object{vehicle_id}.Location * Parameter.Map.Scale <= Parameter.ExitThreshold 
                    if current_lane == Parameter.Map.Lane
                        List.Vehicle.Object{vehicle_id}.ExitState = 1;
                    else
                        List.Vehicle.Object{vehicle_id}.ExitState = 0;
                    end
                end

                MoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)},Time,Parameter,List)
            end
        
            % Remove Processed Vehicles
            for i = 1:size(List.Vehicle.Active,1)
                %if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location >= 300000 % exit으로 바꾸기
                %    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                %    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];

                %    % record travel time, avg speed

                %end
                if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale > Parameter.Map.Road % through vehicles
                    spawn_time = List.Vehicle.Object{List.Vehicle.Active(i,1)}.EntryTime;
                    exit_time = Time;
                    travel_time = exit_time - spawn_time;
                    exit_location = List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale;

                    travel_times = [travel_times, travel_time];
                    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];
                    TotalVehicles = TotalVehicles - 1;
                

                elseif List.Vehicle.Object{List.Vehicle.Active(i,1)}.ExitState >= 0 && List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale >= List.Vehicle.Object{List.Vehicle.Active(i,1)}.Exit - 2 
                    % record travel time, avg speed
                    spawn_time = List.Vehicle.Object{List.Vehicle.Active(i,1)}.EntryTime;
                    exit_time = Time;
                    travel_time = exit_time - spawn_time;
                    exit_location = List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale;

                    travel_times = [travel_times, travel_time];

                    % 🔹 Exit 성공 차량인지 확인
                    if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Lane == Parameter.Map.Lane
                        exit_success_count = exit_success_count + 1;
                    else
                        exit_fail_count = exit_fail_count + 1;  % 🔹 최우측 차선이 아니면 exit fail로 기록
                    end

                    % remove vehicle
                    RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                    List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];
                end
                
            end

        
            % Finalize Time Step
            if Simulation.Setting.Draw == 1
                drawnow();

                pause(Simulation.Setting.PauseTime);

                if Simulation.Setting.RecordVideo == 1 % && mod(int32(Time/Parameter.Physics), 2) == 0
                    frame = getframe(gcf); 
                    % frame.cdata = imresize(frame.cdata, 0.5);
                    writeVideo(videoWriter, frame); 
                end
            end
            if Simulation.Setting.SpawnType && isempty(List.Vehicle.Active) && isempty(SpawnVehicle)
                break
            end

        end

        total_exited_vehicles = exit_success_count + exit_fail_count;
        if TotalVehicles ~= total_exited_vehicles
            disp("something wrong here");
        end
        if total_exited_vehicles > 0
            exit_fail_rate = round(exit_fail_count / total_exited_vehicles, 3); % 🔹 소수점 3자리까지
        else
            exit_fail_rate = NaN; % 🔹 차량이 하나도 없으면 NaN
        end

        avg_travel_time = round(mean(travel_times), 3);
        if isempty(travel_times)
            avg_travel_time = NaN;
        end

        % 🔹 각 mode_idx마다 결과 저장 (Avg Travel Time + Exit Fail Rate)
        result_row{2+mode_idx} = avg_travel_time;
        result_row{2+num_modes+mode_idx} = exit_fail_rate;

        disp("Iteration: " + Iteration)
        if Time > Parameter.Sim.Time - Parameter.Physics
            ;
        end


        if Simulation.Setting.RecordVideo
            close(videoWriter);
            disp(['Simulation video saved to: ', videoFilename]);
        end

        clear Parameter List Seed environment GRAPE_output;

    end
    result_row{2} = TotalVehicles;
    results(Iteration, :) = result_row;
end
% 🔹 모든 Iteration이 끝난 후, 엑셀 파일에 한 번에 결과 저장
if Simulation.Setting.RecordExcel
    writecell(results, filename, 'Sheet', sheet, 'WriteMode', 'append');
    disp(['✅ Simulation results saved to: ', filename]);
end
