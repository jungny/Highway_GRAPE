close all
clear
clc
addpath('Map\','Vehicle\','Signal\','Manager\','v2v\','GRAPE\')

Simulation.Setting.Window = 1000;
Simulation.Setting.Draw = 1;
Simulation.Setting.StopOnGrapeError = 1;
Simulation.Setting.PauseTime = 0.03; % 0: No pause. >0: Pause duration in seconds (Default: 0.01)
Simulation.Setting.SaveFolder = 'C:\Users\user\Desktop\250211_0220';

Simulation.Setting.RecordLog = 1;    % 1: Record log file, 0: Do not record
Simulation.Setting.RecordVideo = 0;  % 1: Record video file, 0: Do not record
% Simulation.Setting.RecordExcel = 1;  % 1: Record Excel file, 0: Do not record

Simulation.Setting.VideoPath = @(randomSeed, timestamp) ...
    fullfile(Simulation.Setting.SaveFolder, 'Simulations', ...
    ['Label1_' num2str(randomSeed) '_' Simulation.Setting.Util_type '_' timestamp '.mp4']);

Simulation.Setting.LogPath = @(finalRandomSeed) ...
    fullfile(Simulation.Setting.SaveFolder, 'Simulations', ...
    ['log_' num2str(finalRandomSeed) '.txt']);

cycle_GRAPE = 5; % GRAPE instance per 5 seconds

Simulation.Setting.InitialRandomSeed = 0;
Simulation.Setting.Iterations = 1; % number of iterations
Simulation.Setting.Time = 200;

Simulation.Setting.SpawnType = 1; %0: spawn by flow rate. 1: spawn manually


%Simulation.Setting.Util_type = 'Max_velocity'; % 'Test' or 'Min_travel_time' or 'Max_velocity'
%Simulation.Setting.Util_type = 'Min_travel_time';
Simulation.Setting.Util_type = 'Test';
%Simulation.Setting.Util_type = 'Hybrid';
Simulation.Setting.NumberOfParticipants = 'Ahead'; % 'Default' or 'Ahead'
%Simulation.Setting.NumberOfParticipants = 'Ahead'; % 'Default' or 'Ahead'
% Simulation.Setting.LaneChangeMode = 'MOBIL'; % 'MOBIL' or 'SimpleLaneChange'
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


environment = struct();
GRAPE_output = [];
travel_times = [];
RemovedVehicle = 0;

for Iteration = 1:Simulation.Setting.Iterations
    close all;
    %rng(46)
    randomSeed = 4;
    rng(randomSeed);
    %randomSeed = Simulation.Setting.InitialRandomSeed + Iteration;
    %rng(randomSeed)

    Parameter = GetParameters(Simulation.Setting);
    GetWindow(Parameter.Map,Simulation.Setting)
    Parameter.Trajectory = GetTrajectory(Parameter.Map,Simulation.Setting);

    if Simulation.Setting.RecordVideo
        timestamp = datestr(now, 'HH-MM');
        videoFilename = Simulation.Setting.VideoPath(randomSeed, timestamp);
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
        title(sprintf('Time: %0.2f s', Time));

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

        elseif Simulation.Setting.SpawnType == 1
            if firstCount == 0
                [SpawnVehicle, ~] = GetSeed(Simulation.Setting, Parameter, TotalVehicles, SpawnLanes, NextArrivalTime);
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

        % Call GRAPE_instance every cycle_GRAPE seconds.
        if mod(Time, cycle_GRAPE) == cycle_GRAPE-1 && size(List.Vehicle.Active,1)>0  %&& Time > 2000
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
            
            if GRAPE_done == 1
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

                    if feasible
                        current_vehicle.TargetLane = desired_lane;
                        current_vehicle.LaneChangeFlag = 1;
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
            if List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location >= 300000 % exit으로 바꾸기
                RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];

                % record travel time, avg speed

            end

            if List.Vehicle.Object{List.Vehicle.Active(i,1)}.ExitState >= 0 && List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale >= List.Vehicle.Object{List.Vehicle.Active(i,1)}.Exit - 5 
                % record travel time, avg speed
                spawn_time = List.Vehicle.Object{List.Vehicle.Active(i,1)}.EntryTime;
                exit_time = Time;
                travel_time = exit_time - spawn_time;
                exit_location = List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location * Parameter.Map.Scale;

                travel_times = [travel_times, travel_time];

                
                % remove vehicle
                RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];
            end
            % if List.Vehicle.Object{List.Vehicle.Active(i,1)}.State == 0
            %     RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
            %     List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];
            % end
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

    end

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

