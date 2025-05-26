classdef Vehicle < handle

    properties % Unique Data
        ID
        Lane
        Agent
        Exit
        SpawnTime
    end

    properties % Data
        Index
        EntryTime
        ExitTime
        Delay
        Data
        %Reward
        Destination
        LaneAlloc
        TargetLane
        LaneIfFullyInside
        LaneChangeFlag
        PolitenessFactor
        IsChangingLane
        AllocLaneDuringGRAPE
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
        CachedRotationMatrix  % Add cached rotation matrix
    end

    properties(Hidden = false) % Dynamics
        Trajectory
        State
        Location
        Velocity
        Acceleration
        ExitState
        TempGreedyWait
        DistanceToExit
        LaneChangeStartTime    % 차선변경 시작 시간
        LaneChangeDuration     % 차선변경 소요 시간 (기본값 4초)
        LaneChangeStartY       % 차선변경 시작 y좌표
        LaneChangeTargetY      % 차선변경 목표 y좌표
        LaneChangeProgress     % 차선변경 진행도 (0~1)
        LaneChangeVelocity     % 차선변경 속도
        LaneChangeAcceleration % 차선변경 가속도
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
            obj.SpawnTime = Seed(6);
            obj.ColorCount = 0;
            obj.IsChangingLane = false;
            obj.trajectory_plot = [];
            obj.AllocLaneDuringGRAPE = [];
            obj.DistanceToExit = [];
            obj.LaneIfFullyInside = [];

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
            % exit_index = find(Parameter.Map.Exit == obj.Exit, 1);
            % if isempty(exit_index)
            %     exit_index = -1;
            % end

            % if exit_index == 1
            %     exit_index = 'Ex';
            % elseif exit_index == 2
            %     exit_index = 'Th';
            % end

            obj.DistanceToExit = obj.Exit - ...
                               obj.Location * Parameter.Map.Scale;  % Exit까지 거리
            
            if obj.DistanceToExit <= 200+200
                exit_index = 'Ex';
                % disp(distance_to_exit);
            else
                exit_index = 'Th';
            end

            if Parameter.Label
                obj.Text = text(x_center+6, y_center+0.1, sprintf('       %d   %s', obj.ID, exit_index), ...
                    'HorizontalAlignment', 'center', ...
                    'VerticalAlignment', 'middle', ...
                    'Parent', obj.Object, ...
                    'FontSize', 8, 'Color', 'black');
            else
                 obj.Text = text(x_center, y_center+0.1, sprintf('        %s', exit_index), ...
                    'HorizontalAlignment', 'center', ...
                    'VerticalAlignment', 'middle', ...
                    'Parent', obj.Object, ...
                    'FontSize', 8, 'Color', 'black');
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

            % 차선변경 관련 속성 초기화
            obj.LaneChangeStartTime = [];
            obj.LaneChangeDuration = 4; % 기본값 설정
            obj.LaneChangeStartY = [];
            obj.LaneChangeTargetY = [];
            obj.LaneChangeProgress = [];
            obj.LaneChangeVelocity = [];
            obj.LaneChangeAcceleration = [];

            obj.Data = GetObservation(obj);
        end
        

        function MoveVehicle(obj,Time,Parameter,Setting)
            Draw = Setting.Draw;
            if ~obj.IsChangingLane && Draw
                % FaceColor가 이미 white가 아니면만 변경
                if ~isequal(get(obj.Patch, 'FaceColor'), [1 1 1])
                    set(obj.Patch, 'FaceColor', 'white');
                end
            end

            if ~isempty(obj.LaneChangeFlag) && obj.LaneChangeFlag == 1 && ~obj.IsChangingLane
                obj.IsChangingLane = true;
                obj.LaneChangeStartTime = Time;
                obj.LaneChangeDuration = 4; % 파라미터로 설정 가능
                
                % 현재 차선과 목표 차선의 y좌표 계산
                obj.LaneChangeStartY = obj.Trajectory(2, obj.Location);
                obj.LaneChangeTargetY = (Parameter.Map.Lane - obj.TargetLane + 0.5) * Parameter.Map.Tile;
                
                % 차선변경 초기화
                obj.LaneChangeProgress = 0;
                obj.LaneChangeVelocity = 0;
                obj.LaneChangeAcceleration = 4 / (obj.LaneChangeDuration * obj.LaneChangeDuration); % [lanes/s^2]
                
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
                
                % 차량 색상 변경
                if Draw
                    set(obj.Patch, 'FaceColor', trajectory_color);
                end
                
                % 궤적 시각화
                if Parameter.ShowTraj
                    x_traj = obj.Trajectory(1, obj.Location:min(obj.Location + 100, size(obj.Trajectory, 2)));
                    y_traj = linspace(obj.LaneChangeStartY, obj.LaneChangeTargetY, length(x_traj));
                    obj.trajectory_plot = plot(x_traj, y_traj, '-', 'Color', trajectory_color, 'LineWidth', 2);
                end
            end

            % 기본 동역학 계산
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
                obj.Velocity = nextVelocity;
                
                % 차선변경 중인 경우 횡방향 이동 적용
                if obj.IsChangingLane
                    % 차선변경 진행도 계산 (0~1)
                    elapsedTime = Time - obj.LaneChangeStartTime;
                    progress = min(elapsedTime / obj.LaneChangeDuration, 1);
                    
                    % 차선변경 동역학 업데이트
                    dt = obj.TimeStep;  % 시간 간격
                    if progress < 0.5
                        % 첫 번째 구간: 가속
                        obj.LaneChangeVelocity = obj.LaneChangeVelocity + obj.LaneChangeAcceleration * dt;
                    else
                        % 두 번째 구간: 감속
                        obj.LaneChangeVelocity = obj.LaneChangeVelocity - obj.LaneChangeAcceleration * dt;
                    end
                    
                    % 횡방향 이동량 계산 (velocity 기반)
                    lateralOffset = obj.LaneChangeVelocity * dt * (obj.LaneChangeTargetY - obj.LaneChangeStartY);
                    
                    % Trajectory 업데이트
                    currentY = obj.Trajectory(2, obj.Location);
                    newY = currentY + lateralOffset;
                    obj.Trajectory(2, obj.Location:end) = newY;
                    
                    % 위치 업데이트
                    obj.Object.Matrix(1:2,4) = obj.Trajectory(:,obj.Location);
                    
                    % 방향이 변경된 경우에만 회전 행렬 업데이트
                    if isempty(obj.CachedRotationMatrix) || ~isequal(obj.Object.Matrix(1:2,1:2), obj.CachedRotationMatrix)
                        obj.CachedRotationMatrix = GetRotation(obj);
                        obj.Object.Matrix(1:2,1:2) = obj.CachedRotationMatrix;
                    end
                    
                    % 차선변경 완료 체크
                    if progress >= 1
                        % 상태 초기화
                        obj.Lane = obj.TargetLane;
                        obj.TargetLane = [];
                        obj.LaneChangeFlag = [];
                        obj.IsChangingLane = false;
                        obj.LaneChangeStartTime = [];
                        obj.LaneChangeStartY = [];
                        obj.LaneChangeTargetY = [];
                        obj.LaneChangeProgress = [];
                        obj.LaneChangeVelocity = [];
                        obj.LaneChangeAcceleration = [];
                        
                        % 차량 색상 복원
                        if Draw && ~isequal(get(obj.Patch, 'FaceColor'), [1 1 1])
                            set(obj.Patch, 'FaceColor', 'white');
                        end
                        
                        % 궤적 제거
                        if Parameter.RemoveTraj
                            if ~isempty(obj.trajectory_plot) && ishandle(obj.trajectory_plot)
                                delete(obj.trajectory_plot);
                                obj.trajectory_plot = [];
                            end
                        end
                    end
                else
                    % 차선변경 중이 아닌 경우 기본 위치만 업데이트
                    newPosition = obj.Trajectory(:,obj.Location);
                    rotation = GetRotation(obj);
                    
                    % 4x4 동차 변환 행렬 생성 (2D 회전 및 이동)
                    transformMatrix = [rotation(1,1), rotation(1,2), 0, newPosition(1) ;
                                       rotation(2,1), rotation(2,2), 0, newPosition(2) ;
                                       0,             0,             1, 0              ;
                                       0,             0,             0, 1              ];
                                   
                    obj.Object.Matrix = transformMatrix;
                end
            end

            x_center = mean(obj.Size(1,:));
            y_center = mean(obj.Size(2,:));

            % 1. 차량 중심 위치
            location = obj.Trajectory(:, obj.Location);  % [x; y]

            % 2. 회전 행렬
            rotation = obj.Object.Matrix(1:2,1:2);

            % 3. 로컬 좌표계 상 꼭짓점 (4x2 행렬)
            localCorners = obj.Size(1:2,:);  % 2x4

            % 4. 글로벌 좌표계로 변환
            globalCorners = rotation * localCorners + location;  % 2x4
            obj.LaneIfFullyInside = [];

            for j = 1:Parameter.Map.Lane
                y_min = (Parameter.Map.Lane - j) * Parameter.Map.Tile;
                y_max = (Parameter.Map.Lane + 1 - j) * Parameter.Map.Tile;
                
                if all(globalCorners(2,:) >= y_min) && all(globalCorners(2,:) < y_max)
                    obj.LaneIfFullyInside = j;
                    break;
                end
            end
           
            

            % exit_index = find(obj.ParameterMap.Exit == obj.Exit, 1);
            % if isempty(exit_index)
            %     exit_index = -1;
            % end

            % if exit_index == 1
            %     exit_index = 'Ex';
            % elseif exit_index == 2
            %     exit_index = 'Th';
            % end
            obj.DistanceToExit = obj.Exit - ...
                                obj.Location * Parameter.Map.Scale;  % Exit까지 거리

            if obj.DistanceToExit <= 200+200
                exit_index = 'Ex';
            else
                exit_index = 'Th';
            end

            if Draw
                if Parameter.Label
                    % String이 실제로 바뀔 때만 set
                    if ~isempty(obj.LaneAlloc)
                        new_str = sprintf('%d     %d   %s', obj.LaneAlloc, obj.ID, exit_index);
                    else
                        new_str = sprintf('%d     %d   %s', obj.Lane, obj.ID, exit_index);
                    end
                    if ~strcmp(get(obj.Text, 'String'), new_str)
                        set(obj.Text, 'String', new_str);
                    end
                    set(obj.Text, 'Position', [x_center, y_center+0.1]);
                else
                    new_str = [];

                    if ~strcmp(get(obj.Text, 'String'), new_str)
                        set(obj.Text, 'String', new_str);
                    end
                end
            end

            if Draw && ~isempty(obj.ExitState)
                if obj.ExitState == 1
                    if ~isequal(get(obj.Patch, 'FaceColor'), [0 1 0])
                        set(obj.Patch, 'FaceColor', 'green');
                    end
                elseif obj.ExitState == 0
                    if ~isequal(get(obj.Patch, 'FaceColor'), [0 0 0])
                        set(obj.Patch, 'FaceColor', 'black');
                    end
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

