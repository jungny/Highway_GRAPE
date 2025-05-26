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
    
    methods
        function obj = Vehicle(Seed,Time,Parameter)
            obj.ID = Seed(1);
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

            obj.Trajectory = [Parameter.Trajectory.Source{obj.Lane}];

            if Parameter.Draw
                obj.Object = hgtransform;
                obj.Size(1,:) = [Parameter.Veh.Size(1)-Parameter.Veh.Size(3) -Parameter.Veh.Size(3) -Parameter.Veh.Size(3) Parameter.Veh.Size(1)-Parameter.Veh.Size(3)];
                obj.Size(2,:) = [Parameter.Veh.Size(2)/2 Parameter.Veh.Size(2)/2 -Parameter.Veh.Size(2)/2 -Parameter.Veh.Size(2)/2];
                obj.Size(3,:) = obj.Size(1,:) + [Parameter.Veh.Buffer(1) -Parameter.Veh.Buffer(1) -Parameter.Veh.Buffer(1) Parameter.Veh.Buffer(1)];
                obj.Size(4,:) = obj.Size(2,:) + [Parameter.Veh.Buffer(2) Parameter.Veh.Buffer(2) -Parameter.Veh.Buffer(2) -Parameter.Veh.Buffer(2)];
                obj.Patch = patch('XData',obj.Size(1,:),'YData',obj.Size(2,:),'FaceColor','white','Parent',obj.Object);

                x_center = mean(obj.Size(1,:));
                y_center = mean(obj.Size(2,:));

                obj.DistanceToExit = obj.Exit - obj.Location * Parameter.Map.Scale;

                if obj.DistanceToExit <= 200+200
                    exit_index = 'Ex';
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
            end

            obj.ParameterMap = Parameter.Map;
            obj.Parameter = Parameter.Veh;
            obj.TimeStep = Parameter.Physics;
            obj.DistanceStep = 1/Parameter.Map.Scale;

            obj.State = Parameter.Veh.State.Rejected;
            obj.Location = uint32(SpawnPosition * obj.DistanceStep);
            obj.Velocity = obj.Parameter.MaxVel;
            
            if Parameter.Draw
                obj.Object.Matrix(1:2,4) = obj.Trajectory(:,obj.Location);
                obj.Object.Matrix(1:2,1:2) = GetRotation(obj);
            end
            
            obj.MaxVel = obj.Parameter.MaxVel;
            obj.Margin = Parameter.Map.Margin*obj.DistanceStep;

            obj.LaneChangeStartTime = [];    % 차선변경 시작 시간
            obj.LaneChangeDuration = 4;      % 차선변경 소요 시간 (기본값 4초)
            obj.LaneChangeStartY = [];       % 차선변경 시작 y좌표
            obj.LaneChangeTargetY = [];      % 차선변경 목표 y좌표
            obj.LaneChangeProgress = [];     % 차선변경 진행도 (0~1)
            obj.LaneChangeVelocity = [];     % 차선변경 속도
            obj.LaneChangeAcceleration = []; % 차선변경 가속도

            obj.Data = GetObservation(obj);
        end
        

        function MoveVehicle(obj,Time,Parameter,Setting)
            Draw = Setting.Draw;
            
            if Draw
                if ~obj.IsChangingLane
                    if ~isequal(get(obj.Patch, 'FaceColor'), [1 1 1])
                        set(obj.Patch, 'FaceColor', 'white');
                    end
                end
            end

            if ~isempty(obj.LaneChangeFlag) && obj.LaneChangeFlag == 1 && ~obj.IsChangingLane
                obj.IsChangingLane = true;
                obj.LaneChangeStartTime = Time;
                obj.LaneChangeDuration = 4;  % 파라미터로 설정 가능
                
                % 현재 차선과 목표 차선의 y좌표 계산
                obj.LaneChangeStartY = obj.Trajectory(2, obj.Location);
                obj.LaneChangeTargetY = (Parameter.Map.Lane - obj.TargetLane + 0.5) * Parameter.Map.Tile;
                
                % 차선변경 초기화
                obj.LaneChangeProgress = 0;
                obj.LaneChangeVelocity = 0;
                obj.LaneChangeAcceleration = 4 / (obj.LaneChangeDuration * obj.LaneChangeDuration); % [lanes/s^2]
                
                if Draw
                    current_lane = obj.Lane;
                    target_lane = obj.TargetLane;
                    
                    if current_lane < target_lane
                        if current_lane == 1
                            trajectory_color = '#91c4ed';
                        else
                            trajectory_color = '#5a9bd4';
                        end
                    else
                        if current_lane == 3
                            trajectory_color = '#ed91ae';
                        else
                            trajectory_color = '#d46b8c';
                        end
                    end
                    
                    set(obj.Patch, 'FaceColor', trajectory_color);
                    
                    if Parameter.ShowTraj
                        x_traj = obj.Trajectory(1, obj.Location:min(obj.Location + 100, size(obj.Trajectory, 2)));
                        y_traj = linspace(obj.LaneChangeStartY, obj.LaneChangeTargetY, length(x_traj));
                        obj.trajectory_plot = plot(x_traj, y_traj, '-', 'Color', trajectory_color, 'LineWidth', 2);
                    end
                end
            end

            [nextVelocity,nextLocation] = GetDynamics(obj);
            obj.Data = GetObservation(obj);
            
            if nextLocation > size(obj.Trajectory,2)
                obj.State = 0;
                if Draw
                    obj.Object.Matrix(1:2,4) = obj.Trajectory(:,end);
                end
                obj.ExitTime = Time + (size(obj.Trajectory,2) - obj.Location)/obj.DistanceStep/obj.MaxVel;
                obj.Delay = round((obj.ExitTime-obj.EntryTime),-log10(obj.TimeStep)+1);
                if obj.Delay < 0
                    obj.Delay = 0;
                end
            else
                obj.Location = nextLocation;
                obj.Velocity = nextVelocity;
                
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
                    
                    if Draw
                        obj.Object.Matrix(1:2,4) = obj.Trajectory(:,obj.Location);
                        obj.Object.Matrix(1:2,1:2) = GetRotation(obj);
                    end
                    
                    if progress >= 1
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
                        
                        if Draw
                            if ~isequal(get(obj.Patch, 'FaceColor'), [1 1 1])
                                set(obj.Patch, 'FaceColor', 'white');
                            end
                            
                            if Parameter.RemoveTraj
                                if ~isempty(obj.trajectory_plot) && ishandle(obj.trajectory_plot)
                                    delete(obj.trajectory_plot);
                                    obj.trajectory_plot = [];
                                end
                            end
                        end
                    end
                else
                    if Draw
                        obj.Object.Matrix(1:2,4) = obj.Trajectory(:,obj.Location);
                        obj.Object.Matrix(1:2,1:2) = GetRotation(obj);
                    end
                end
            end

            if Draw
                x_center = mean(obj.Size(1,:));
                y_center = mean(obj.Size(2,:));

                location = obj.Trajectory(:, obj.Location);
                rotation = obj.Object.Matrix(1:2,1:2);
                localCorners = obj.Size(1:2,:);
                globalCorners = rotation * localCorners + location;
                obj.LaneIfFullyInside = [];

                for j = 1:Parameter.Map.Lane
                    y_min = (Parameter.Map.Lane - j) * Parameter.Map.Tile;
                    y_max = (Parameter.Map.Lane + 1 - j) * Parameter.Map.Tile;
                    
                    if all(globalCorners(2,:) >= y_min) && all(globalCorners(2,:) < y_max)
                        obj.LaneIfFullyInside = j;
                        break;
                    end
                end

                obj.DistanceToExit = obj.Exit - obj.Location * Parameter.Map.Scale;

                if obj.DistanceToExit <= 200+200
                    exit_index = 'Ex';
                else
                    exit_index = 'Th';
                end

                if Parameter.Label
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

                if ~isempty(obj.ExitState)
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
        end

        function RemoveVehicle(obj, Draw)
            if Draw
                delete(obj.Object)
            end
            delete(obj)
        end
    end
end

