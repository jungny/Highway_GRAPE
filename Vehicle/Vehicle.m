classdef Vehicle < handle

    properties % Unique Data
        ID
        Lane
        Agent
        Exit
    end

    properties % Data
        Index
        EntryTime
        ExitTime
        Delay
        Data
        %Reward
        Destination
        TargetLane
        LaneChangeFlag
        PolitenessFactor
        IsChangingLane
    end

    properties(Hidden = false) % Properties
        Size
        Object
        Patch
        Parameter
        ParameterMap
        TimeStep   
        DistanceStep
        MaxVel
        Margin
        ColorCount
        trajectory_plot
    end

    properties(Hidden = false) % Dynamics
        Trajectory
        State
        Location
        Velocity
        Acceleration
        ExitState
        TempGreedyWait
    end

    properties(Hidden = true) % Control
        EnterControl
        ExitControl
        Text
    end

    properties(Hidden = true) % Reservation
        Ghost
        GhostLocation
        GhostVelocity
        GhostAcceleration
        Slots
        TimeSlot
        Horizon
    end
    
    methods
        function obj = Vehicle(Seed,Time,Parameter)
            % obj.Index = [1 2 3 4 1 2 3 4];
            obj.ID = Seed(1);
            %obj.Agent = Seed(5);
            obj.EntryTime = Time;
            obj.Lane = Seed(2);
            obj.Exit = Seed(3);
            obj.ExitState = -1;
            obj.PolitenessFactor = Seed(4);
            SpawnPosition = Seed(5);
            %obj.SpawnTime = Seed(6);
            obj.ColorCount = 0;
            obj.IsChangingLane = false;
            obj.trajectory_plot = [];

            % 고속도로에서는 방향(Destination) 관련 로직 불필요
            % 경로(Trajectory) 설정: 출발점(Source) → 도착점(Sink)
            % obj.Trajectory = [Parameter.Trajectory.Source{obj.Lane}, ...
            % Parameter.Trajectory.Sink{obj.Lane}];

            
            obj.Trajectory = [Parameter.Trajectory.Source{obj.Lane}];
            % fullTrajectory = Parameter.Trajectory.Source{obj.Lane};
            % [~, closestIdx] = min(abs(fullTrajectory(1,:) - SpawnPosition)); % SpawnPosition과 가장 가까운 인덱스 찾기
            % obj.Trajectory = fullTrajectory(:, closestIdx:end); % SpawnPosition부터 경로 시작

            % obj.ExitControl = size(obj.Trajectory,2) - size(Parameter.Trajectory.Sink{obj.Lane},2);

            obj.Object = hgtransform;
            obj.Size(1,:) = [Parameter.Veh.Size(1)-Parameter.Veh.Size(3) -Parameter.Veh.Size(3) -Parameter.Veh.Size(3) Parameter.Veh.Size(1)-Parameter.Veh.Size(3)];
            obj.Size(2,:) = [Parameter.Veh.Size(2)/2 Parameter.Veh.Size(2)/2 -Parameter.Veh.Size(2)/2 -Parameter.Veh.Size(2)/2];
            obj.Size(3,:) = obj.Size(1,:) + [Parameter.Veh.Buffer(1) -Parameter.Veh.Buffer(1) -Parameter.Veh.Buffer(1) Parameter.Veh.Buffer(1)];
            obj.Size(4,:) = obj.Size(2,:) + [Parameter.Veh.Buffer(2) Parameter.Veh.Buffer(2) -Parameter.Veh.Buffer(2) -Parameter.Veh.Buffer(2)];
            obj.Patch = patch('XData',obj.Size(1,:),'YData',obj.Size(2,:),'FaceColor','white','Parent',obj.Object);
            % if obj.Agent == 1
            %     obj.Patch = patch('XData',obj.Size(1,:),'YData',obj.Size(2,:),'FaceColor','white','Parent',obj.Object);
            % else
            %     obj.Patch = patch('XData',obj.Size(1,:),'YData',obj.Size(2,:),'FaceColor','white','Parent',obj.Object); % #cfcdc0
            % end

            x_center = mean(obj.Size(1,:));
            y_center = mean(obj.Size(2,:));
            exit_index = find(Parameter.Map.Exit == obj.Exit, 1);
            if isempty(exit_index)
                exit_index = -1;
            end

            if Parameter.Label
                obj.Text = text(x_center+3, y_center+0.1, sprintf('%d   %d', obj.ID, exit_index), ...
                    'HorizontalAlignment', 'center', ...
                    'VerticalAlignment', 'middle', ...
                    'Parent', obj.Object, ...
                    'FontSize', 9, 'Color', 'black');
            else
                 obj.Text = text(x_center, y_center+0.1, sprintf('        %d', exit_index), ...
                    'HorizontalAlignment', 'center', ...
                    'VerticalAlignment', 'middle', ...
                    'Parent', obj.Object, ...
                    'FontSize', 9, 'Color', 'black');
            end


            obj.ParameterMap = Parameter.Map;
            obj.Parameter = Parameter.Veh;
            obj.TimeStep = Parameter.Physics;
            obj.DistanceStep = 1/Parameter.Map.Scale;

            obj.State = Parameter.Veh.State.Rejected;
            % obj.Location = 1;
            obj.Location = uint32(SpawnPosition * obj.DistanceStep);
            obj.Velocity = obj.Parameter.MaxVel;
            obj.Object.Matrix(1:2,4) = obj.Trajectory(:,obj.Location);
            obj.Object.Matrix(1:2,1:2) = GetRotation(obj);
            obj.MaxVel = obj.Parameter.MaxVel;
            obj.Margin = Parameter.Map.Margin*obj.DistanceStep;

            obj.Data = GetObservation(obj);
        end
        

        function MoveVehicle(obj,Time,Parameter,List)
            if ~obj.IsChangingLane
                if obj.Location * Parameter.Map.Scale >= 20 
                    set(obj.Patch, 'FaceColor', 'white');
                else
                    set(obj.Patch, 'FaceColor', '#a9a9a9');
                end
            end

            % if obj.ColorCount > 0
            %     set(obj.Patch, 'FaceColor', '#f589e6');
            %     obj.ColorCount = obj.ColorCount-1;
            % end



            % if obj.TempGreedyWait > 0
            %     obj.TempGreedyWait = obj.TempGreedyWait - Parameter.Physics;
            % end


            if ~isempty(obj.LaneChangeFlag) && obj.LaneChangeFlag == 1 && ~obj.IsChangingLane
                % obj.ColorCount = 10;

                targetLane = obj.TargetLane;
                obj.IsChangingLane = true;
                % change lane to obj.TargetLane
                new_y = (Parameter.Map.Lane-targetLane+0.5)*Parameter.Map.Tile;
                
                change_steps = 3000; 
                start_idx = obj.Location; 
                end_idx = min(obj.Location + change_steps - 1, size(obj.Trajectory, 2)); 
                
                % change_steps 동안 new_y에 도달
                obj.Trajectory(2, start_idx:end_idx) = linspace(obj.Trajectory(2, start_idx), new_y, end_idx - start_idx + 1);

                % 예상 궤적의 x, y 좌표 계산
                x_traj = obj.Trajectory(1, start_idx:end_idx);
                y_traj = linspace(obj.Trajectory(2, start_idx), new_y, end_idx - start_idx + 1);
                
                % 현재 차선과 목표 차선에 따라 색상 결정
                current_lane = obj.Lane;
                target_lane = obj.TargetLane;
                
                if current_lane < target_lane  % 오른쪽 방향 (파란 계열)
                    if current_lane == 1  % 1→2: 연한 파란색
                        trajectory_color = '#91c4ed';
                    else  % 2→3: 진한 파란색
                        trajectory_color = '#5a9bd4';
                    end
                else  % 왼쪽 방향 (핑크 계열)
                    if current_lane == 3  % 3→2: 연한 핑크색
                        trajectory_color = '#ed91ae';
                    else  % 2→1: 진한 핑크색
                        trajectory_color = '#d46b8c';
                    end
                end
                
                % 궤적 그리기와 핸들 저장
                % if obj.Exit > 600
                    obj.trajectory_plot = plot(x_traj, y_traj, '-', 'Color', trajectory_color, 'LineWidth', 2);
                % end
                
                % 차량 색상도 궤적 색상과 동일하게 설정
                set(obj.Patch, 'FaceColor', trajectory_color);

                % 이후 구간 고정
                if end_idx < size(obj.Trajectory, 2)
                    obj.Trajectory(2, end_idx+1:end) = new_y;
                end

            end
                

            

            [nextVelocity,nextLocation] = GetDynamics(obj);
            obj.Data = GetObservation(obj);
            if nextLocation > size(obj.Trajectory,2)
                obj.State = 0;
                obj.Object.Matrix(1:2,4) = obj.Trajectory(:,end);
                obj.ExitTime = Time + (size(obj.Trajectory,2) - obj.Location)/obj.DistanceStep/obj.MaxVel;
                obj.Delay = round((obj.ExitTime-obj.EntryTime),-log10(obj.TimeStep)+1);
                if obj.Delay < 0
                    obj.Delay = 0;
                end
            else
                obj.Location = nextLocation;

                % SaveFolder = 'C:\Users\user\Desktop\250326_0409';
                % logFileName = fullfile(SaveFolder, ...
                %        ['log_2.txt']);
                % fileID = fopen(logFileName, 'a', 'n', 'utf-8');  % append 모드로 파일 열기
                % fprintf(fileID, '\n nextLocation  %d \n', nextLocation);
                % fclose(fileID);
                obj.Velocity = nextVelocity;
                obj.Object.Matrix(1:2,4) = obj.Trajectory(:,obj.Location);
                obj.Object.Matrix(1:2,1:2) = GetRotation(obj);
            end

            x_center = mean(obj.Size(1,:));
            y_center = mean(obj.Size(2,:));

            if Parameter.Label
                set(obj.Text, 'Position', [x_center+3, y_center+0.1]);
            end

            % 차선 변경 완료 시
            if obj.IsChangingLane
                target_y = (Parameter.Map.Lane - obj.TargetLane + 0.5) * Parameter.Map.Tile;
                current_y = obj.Object.Matrix(2, 4);
                if abs(current_y - target_y) < 1e-2
                    obj.Lane = obj.TargetLane;
                    obj.TargetLane = [];
                    obj.LaneChangeFlag = [];
                    obj.IsChangingLane = false;
                    set(obj.Patch, 'FaceColor', 'white');
                    
                    % NoRemoveTraj 설정에 따라 궤적 삭제 여부 결정
                    if Parameter.RemoveTraj
                        if ~isempty(obj.trajectory_plot) && ishandle(obj.trajectory_plot)
                            delete(obj.trajectory_plot);
                            obj.trajectory_plot = [];
                        end
                    end
                end
            end

            if ~isempty(obj.ExitState)
                if obj.ExitState == 1
                    set(obj.Patch, 'FaceColor', 'green');
                elseif obj.ExitState == 0
                    set(obj.Patch, 'FaceColor', 'black');
                end
            end


        end


        function PlanReservation(obj,Time)
            obj.Horizon = 100;
            obj.Slots = zeros(187,167,obj.Horizon/obj.TimeStep);
            obj.Ghost = hgtransform;
            patch('XData',obj.Size(1,:),'YData',obj.Size(2,:),'FaceColor','black','FaceAlpha',0.5,'Parent',obj.Ghost)
            obj.GhostLocation = obj.Location;
            obj.GhostVelocity = obj.Velocity;
            obj.GhostAcceleration = obj.Parameter.Accel(1);
            obj.Ghost.Matrix(1:2,4) = obj.Trajectory(:,obj.GhostLocation);
            obj.Ghost.Matrix(1:2,1:2) = GetRotation(obj.Ghost,obj.GhostLocation,obj.Trajectory);
            
            GhostTime = Time;
            while obj.GhostLocation + obj.Size(1,1)*obj.DistanceStep < obj.EnterControl
                GhostTime = GhostTime + obj.TimeStep;
                [nextVelocity,nextLocation] = GetDynamics(obj,obj.GhostAcceleration,obj.GhostVelocity,obj.GhostLocation);
                obj.GhostLocation = nextLocation;
                obj.GhostVelocity = nextVelocity;
                obj.Ghost.Matrix(1:2,4) = obj.Trajectory(:,obj.GhostLocation);
                obj.Ghost.Matrix(1:2,1:2) = GetRotation(obj.Ghost,obj.GhostLocation,obj.Trajectory);
            end
            obj.TimeSlot(1) = GhostTime;
            LayerNumber = 1;
            obj.Slots(:,:,LayerNumber) = GetReservation(obj.Ghost,obj.Size);
            while true
                GhostTime = GhostTime + obj.TimeStep;
                [nextVelocity,nextLocation] = GetDynamics(obj,obj.GhostAcceleration,obj.GhostVelocity,obj.GhostLocation);
                obj.GhostLocation = nextLocation;
                obj.GhostVelocity = nextVelocity;
                obj.Ghost.Matrix(1:2,4) = obj.Trajectory(:,obj.GhostLocation);
                obj.Ghost.Matrix(1:2,1:2) = GetRotation(obj.Ghost,obj.GhostLocation,obj.Trajectory);

                LayerNumber = LayerNumber + 1;
                obj.Slots(:,:,LayerNumber) = GetReservation(obj.Ghost,obj.Size);

                if obj.GhostLocation > obj.ExitControl
                    obj.TimeSlot(2) = GhostTime;
                    break
                end
            end
            delete(obj.Ghost)
        end

        function RemoveVehicle(obj)
            delete(obj.Object)
            delete(obj)
        end


    end
end

