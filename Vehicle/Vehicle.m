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
        TargetLaneList
    end

    properties(Hidden = false) % Properties
        Size
        Object
        Patch
        Parameter
        TimeStep   
        DistanceStep
        MaxVel
        Margin
        ColorCount
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
            %obj.MandatoryLaneList = [Parameter.Map.Lane-1, Parameter.Map.Lane, -1];
            %obj.SpawnTime = Seed(6);
            obj.ColorCount = 0;

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
            if obj.Agent == 1
                obj.Patch = patch('XData',obj.Size(1,:),'YData',obj.Size(2,:),'FaceColor','white','Parent',obj.Object);
            else
                obj.Patch = patch('XData',obj.Size(1,:),'YData',obj.Size(2,:),'FaceColor','#cfcdc0','Parent',obj.Object);
            end

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
            end


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
            if obj.Location * Parameter.Map.Scale >= Parameter.Map.GrapeThreshold
                set(obj.Patch, 'FaceColor', '#FFF38C');
            end

            if obj.ColorCount > 0
                set(obj.Patch, 'FaceColor', '#f589e6');
                obj.ColorCount = obj.ColorCount-1;
            end

            if obj.TempGreedyWait > 0
                obj.TempGreedyWait = obj.TempGreedyWait - Parameter.Physics;
            end


            if obj.LaneChangeFlag == 1
                obj.ColorCount = 10;
                set(obj.Patch, 'FaceColor', '#f589e6');

                targetLane = obj.TargetLane;
                % change lane to obj.TargetLane
                new_y = (Parameter.Map.Lane-targetLane+0.5)*Parameter.Map.Tile;
                
                change_steps = round(obj.Velocity * Parameter.LaneChangeTime / Parameter.Map.Scale); 
                start_idx = obj.Location; 
                end_idx = min(obj.Location + change_steps - 1, size(obj.Trajectory, 2)); 
                
                % change_steps 동안 new_y에 도달
                obj.Trajectory(2, start_idx:end_idx) = linspace(obj.Trajectory(2, start_idx), new_y, end_idx - start_idx + 1);
                
                % 이후 구간 고정
                if end_idx < size(obj.Trajectory, 2)
                    obj.Trajectory(2, end_idx+1:end) = new_y;
                end

                obj.LaneChangeFlag = [];
                obj.Lane = targetLane;
                obj.TargetLane = [];
            end
                

            if ~isempty(obj.ExitState)
                if obj.ExitState == 1
                    set(obj.Patch, 'FaceColor', 'green');
                elseif obj.ExitState == 0
                    set(obj.Patch, 'FaceColor', '#6b6b6b');
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
                obj.Velocity = nextVelocity;
                obj.Object.Matrix(1:2,4) = obj.Trajectory(:,obj.Location);
                obj.Object.Matrix(1:2,1:2) = GetRotation(obj);
            end

            x_center = mean(obj.Size(1,:));
            y_center = mean(obj.Size(2,:));

            if Parameter.Label
                set(obj.Text, 'Position', [x_center+3, y_center+0.1]);
            end

        end

        function RemoveVehicle(obj)
            delete(obj.Object)
            delete(obj)
        end


    end
end

