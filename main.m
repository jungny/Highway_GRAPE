close all
clear
clc
addpath('Map\','Vehicle\','Signal\','Manager\','v2v\','GRAPE\')

Simulation.Setting.Window = 1000;
Simulation.Setting.Draw = 1;
Simulation.Setting.Debug = 0;
Simulation.Setting.Mode = 3;

Simulation.Setting.LogFile = 'C:\Users\user\Desktop\250203_0211\Simulations\log.txt';  % 파일 경로

    % 1: Dataset Generation
    % 2: Evaluation
    % 3: Highway

Simulation.Setting.Vehicles = 10;
cycle_GRAPE = 5;
Simulation.Setting.Time = 200;
Simulation.Setting.Datasets = 1; % number of iterations
Simulation.Setting.Agents = 3;
Simulation.Setting.Turns = 1;


Simulation.Setting.Iterations(1,:) = 1:Simulation.Setting.Datasets;
Simulation.Setting.Iterations(2,:) = Simulation.Setting.Agents*ones(1,Simulation.Setting.Datasets);
Simulation.Setting.Iterations(3,:) = [ones(1,1)]; % 2*ones(1,300) 3*ones(1,300) 4*ones(1,300) 5*ones(1,300) 6*ones(1,300) 7*ones(1,300) 8*ones(1,300) 9*ones(1,300) 10*ones(1,300) 11*ones(1,300) 12*ones(1,300) 13*ones(1,300) 14*ones(1,300) 15*ones(1,300) 16*ones(1,300) 17*ones(1,300) 18*ones(1,300) 19*ones(1,300) 20*ones(1,300)];
Simulation.Setting.Iterations(4,:) = randperm(1000000,Simulation.Setting.Datasets);



%Simulation.Setting.Util_type = 'Max_velocity'; % 'Test' or 'Min_travel_time' or 'Max_velocity'
%Simulation.Setting.Util_type = 'Min_travel_time';
Simulation.Setting.Util_type = 'Test';
%Simulation.Setting.Util_type = 'Hybrid';
Simulation.Setting.NumberOfParticipants = ''; % 'Default' or 'Ahead'
%Simulation.Setting.NumberOfParticipants = 'Ahead'; % 'Default' or 'Ahead'
% Simulation.Setting.LaneChangeMode = 'MOBIL'; % 'MOBIL' or 'SimpleLaneChange'
Simulation.Setting.LaneChangeMode = 'SimpleLaneChange'; % 'MOBIL' or 'SimpleLaneChange'
Simulation.Setting.Record = 1;
    % 1: start recording
Simulation.Setting.ExcelRecord = 0;

%% Set Simulation Parameters


%% Run Simulation
% 새 파일을 만들어서 첫 줄에 헤더 기록 (w 모드는 새로 파일을 만듦)
fileID = fopen(Simulation.Setting.LogFile, 'w');
fprintf(fileID, 'a');
fclose(fileID);
fileID = fopen(Simulation.Setting.LogFile, 'w');

fprintf(fileID, 'Simulation Log\n');
fclose(fileID);


environment = struct();
GRAPE_output = [];
travel_times = [];
RemovedVehicle = 0;

for Iteration = 1:Simulation.Setting.Datasets

    %for participantsMode = ["Default", "Ahead"]
    for participantsMode = "Ahead"
        close all;
        Simulation.Setting.NumberOfParticipants = char(participantsMode);
        %rng(46)
        %random_seed = 59724;
        %rng(random_seed);
        random_seed = 2 + Iteration;
        rng(random_seed)
    
    
        Parameter = GetParameters(Simulation.Setting);
        GetWindow(Parameter.Map,Simulation.Setting)
        Parameter.Trajectory = GetTrajectory(Parameter.Map,Simulation.Setting);
    
        if Simulation.Setting.Record == 1
            timestamp = datestr(now, 'HH-MM');
        
            videoFilename = fullfile('C:\Users\user\Desktop\250203_0211\Simulations\', ...
            [ 'Label1_' num2str(random_seed) '_' Simulation.Setting.Util_type '_'  timestamp '.mp4']);
        
            videoWriter = VideoWriter(videoFilename, 'MPEG-4');
            videoWriter.FrameRate = 30; 
            open(videoWriter);
        end
        
        if Simulation.Setting.Mode == 1
            rng(randi(100000))
        elseif Simulation.Setting.Mode == 3 || Simulation.Setting.Mode == 4
            disp('Highway Simulation')
            fprintf('%d th Iteration', Iteration)
        else
            rng(Simulation.Setting.Seed)
        end
        SetMap(Parameter.Map, Simulation.Setting);
    

        fileID = fopen(Simulation.Setting.LogFile, 'a', 'n', 'utf-8');  % append 모드로 파일 열기
        fprintf(fileID, '\n=====   Random Seed  %d  ||  %s   ===== %s \n', ...
                random_seed, Simulation.Setting.NumberOfParticipants, datestr(now, 'HH시 MM분 SS초'));
        
        % fprintf(fileID, '\n| Lanes:  %d  | Vehicles: %d  | Exits: %d  |\n', ...
        %     Parameter.Map.Lane, TotalVehicles, length(Parameter.Map.Exit));
        fclose(fileID);


        NextArrivalTime = zeros(Parameter.Map.Lane, 1); % 5차선 기준 [0;0;0;0;0]
        %NextArrivalTime = (3600 / Parameter.Flow) * rand(Parameter.Map.Lane, 1);
        disp(NextArrivalTime);
        TotalVehicles = 0;
        firstCount = 0;
        SpawnVehicle = [];
    
        for Time = 0:Parameter.Physics:Parameter.Sim.Time
            GRAPE_done = 0;
            title(sprintf('Time: %0.2f s', Time));

            

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

            % Update Vehicle Data
            List.Vehicle.Data = UpdateData(List.Vehicle.Object,Parameter.Sim.Data);
            List.Vehicle.Active = List.Vehicle.Data(List.Vehicle.Data(:,2)>0,:);
            List.Vehicle.Object = GetAcceleration(List.Vehicle.Object, List.Vehicle.Data, Parameter.Veh);
    
            % Call GRAPE_instance every cycle_GRAPE seconds.
            if mod(Time, cycle_GRAPE) == cycle_GRAPE-1 && size(List.Vehicle.Active,1)>0  %&& Time > 2000
                disp("calling Grape Instance. . . | "+ Time);
    
                environment = GRAPE_main(List,Parameter,Simulation.Setting,Iteration);
    
                %GRAPE_output = GRAPE_instance(environment);
                %lane_alloc = GRAPE_output.Alloc;
                %GRAPE_done = 1;
                
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
                %pause(0.01) %pause(0.01)
                
                %if Simulation.Setting.Record == 1 && mod(int32(Time/Parameter.Physics), 2) == 0
                if Simulation.Setting.Record == 1 
                    frame = getframe(gcf); 
                    % frame.cdata = imresize(frame.cdata, 0.5);
                    writeVideo(videoWriter, frame); 
                end
            end
    
        end

        if Simulation.Setting.ExcelRecord

            % Iteration 종료 후 데이터 정리
            if ~isempty(travel_times)
                mean_travel_time = mean(travel_times);
            else
                mean_travel_time = NaN;
            end
            
            % 엑셀 저장을 위한 데이터 구성
            excel_filename = fullfile('C:\Users\user\Desktop\250116_0203\Simulations\', [Simulation.Setting.Util_type '.xlsx']);
            sheet_name = 'Data';
            
            % 기존 데이터 불러오기
            if exist(excel_filename, 'file')
                existing_data = readmatrix(excel_filename, 'Sheet', sheet_name);
            else
                existing_data = [];
            end

            % 데이터 추가
            new_data = [random_seed, mean_travel_time];
            updated_data = [existing_data; new_data];

            % 엑셀 파일로 저장
            writematrix(updated_data, excel_filename, 'Sheet', sheet_name);

        end
    
        disp("Iteration: " + Iteration)
        if Time > Parameter.Sim.Time - Parameter.Physics
            ;
        end
    
        %  for i = 1:size(Data{Iteration},1)
        %     for ii = 1:size(List.Vehicle.Object,1)
        %         if isempty(Data{Iteration}{i,ii})
        %             if ii < Simulation.Setting.Iterations(2,Iteration) + 1
        %                 Data{Iteration}{i,ii} = zeros(1,7);        
        %             else
        %                 Data{Iteration}{i,ii} = zeros(1,5);
        %             end
        %         end
        %     end
        % end
    
        if Simulation.Setting.Record == 1
            close(videoWriter);
            disp(['Simulation video saved to: ', videoFilename]);
        end
    end
    clear Parameter List Seed environment GRAPE_output;
end

