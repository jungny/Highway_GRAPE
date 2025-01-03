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
    end

    properties(Hidden = false) % Dynamics
        Trajectory
        State
        Location
        Velocity
        Acceleration
        ExitState
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
            obj.Agent = Seed(5);
            obj.EntryTime = Time;
            obj.Lane = Seed(3);
            obj.Exit = Seed(6);
            obj.ExitState = -1;

            % 고속도로에서는 방향(Destination) 관련 로직 불필요
            % 경로(Trajectory) 설정: 출발점(Source) → 도착점(Sink)
            % obj.Trajectory = [Parameter.Trajectory.Source{obj.Lane}, ...
            % Parameter.Trajectory.Sink{obj.Lane}];

            obj.Trajectory = [Parameter.Trajectory.Source{obj.Lane}];

            % if Seed(4) == 1 % 직진 - 여기만 실행
            %     obj.Destination = obj.Index(obj.Lane+2);
            %     obj.Trajectory = [Parameter.Trajectory.Source{obj.Lane} Parameter.Trajectory.Through{obj.Lane} Parameter.Trajectory.Sink{obj.Lane}];
            % elseif Seed(4) == 2 % 좌회전 - 안쓰임
            %     obj.Destination = obj.Index(obj.Lane+3);
            %     obj.Trajectory = [Parameter.Trajectory.Source{obj.Lane} Parameter.Trajectory.Left{obj.Lane} Parameter.Trajectory.Sink{obj.Index(obj.Lane+1)}];
            % elseif Seed(4) == 3 % 우회전 - 안쓰임
            %     obj.Destination = obj.Index(obj.Lane+1);
            %     obj.Trajectory = [Parameter.Trajectory.Source{obj.Lane} Parameter.Trajectory.Right{obj.Lane} Parameter.Trajectory.Sink{obj.Index(obj.Lane+3)}];
            % end
            % obj.EnterControl = size(Parameter.Trajectory.Source{obj.Lane},2);
            % obj.ExitControl = size(obj.Trajectory,2) - size(Parameter.Trajectory.Sink{obj.Lane},2);

            obj.Object = hgtransform;
            obj.Size(1,:) = [Parameter.Veh.Size(1)-Parameter.Veh.Size(3) -Parameter.Veh.Size(3) -Parameter.Veh.Size(3) Parameter.Veh.Size(1)-Parameter.Veh.Size(3)];
            obj.Size(2,:) = [Parameter.Veh.Size(2)/2 Parameter.Veh.Size(2)/2 -Parameter.Veh.Size(2)/2 -Parameter.Veh.Size(2)/2];
            obj.Size(3,:) = obj.Size(1,:) + [Parameter.Veh.Buffer(1) -Parameter.Veh.Buffer(1) -Parameter.Veh.Buffer(1) Parameter.Veh.Buffer(1)];
            obj.Size(4,:) = obj.Size(2,:) + [Parameter.Veh.Buffer(2) Parameter.Veh.Buffer(2) -Parameter.Veh.Buffer(2) -Parameter.Veh.Buffer(2)];
            if obj.Agent == 1
                obj.Patch = patch('XData',obj.Size(1,:),'YData',obj.Size(2,:),'FaceColor','white','Parent',obj.Object);
            else
                obj.Patch = patch('XData',obj.Size(1,:),'YData',obj.Size(2,:),'FaceColor','#FFF38C','Parent',obj.Object);
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
            obj.Location = 1;
            obj.Velocity = obj.Parameter.MaxVel;
            obj.Object.Matrix(1:2,4) = obj.Trajectory(:,obj.Location);
            obj.Object.Matrix(1:2,1:2) = GetRotation(obj);
            obj.MaxVel = obj.Parameter.MaxVel;
            obj.Margin = Parameter.Map.Margin*obj.DistanceStep;

            obj.Data = GetObservation(obj);
        end

        function feasible = CheckLaneChangeFeasibility(obj, targetLane, List, Parameter)
            % 현재 차량 위치와 목표 차선의 선행/후행 차량 간 거리 계산
            feasible = false;  % 기본값: 변경 불가
            current_x = obj.Location * Parameter.Map.Scale;  % 현재 차량의 x 좌표
            lane_vehicles = List.Vehicle.Active(List.Vehicle.Active(:,3) == targetLane, :);  % 목표 차선의 차량
        
            % 목표 차선의 선행/후행 차량 찾기
            distances = lane_vehicles(:,4)*Parameter.Map.Scale - current_x;
            front_distances = distances(distances > 0);
            rear_distances = distances(distances < 0);
        
            % 안전 거리 조건
            safe_distance = Parameter.Veh.SafeDistance;
            if isempty(front_distances) || min(front_distances) > safe_distance
                if isempty(rear_distances) || abs(max(rear_distances)) > safe_distance
                    feasible = true;  % 선행/후행 모두 안전 거리 만족
                end
            end
        end
        
        function [front_vehicle, front_distance] = GetFrontVehicle(obj, targetLane, List, Parameter)
            % 현재 차선의 선행 차량 찾기
            current_x = obj.Location * Parameter.Map.Scale;
        
            % 목표 차선의 차량 필터링
            lane_vehicles = List.Vehicle.Active(List.Vehicle.Active(:,3) == targetLane, :);
        
            % 모든 차량의 거리 계산
            distances = lane_vehicles(:,4) * Parameter.Map.Scale - current_x;
        
            % 선행 차량 거리 필터링
            front_distances = distances(distances > 0);
        
            % 초기화
            front_vehicle = [];
            front_distance = inf;
        
            if ~isempty(front_distances)
                % 가장 가까운 선행 차량 거리와 인덱스 찾기
                [front_distance, ~] = min(front_distances);
        
                % front_distances 값이 distances에서의 원래 인덱스 찾기
                tolerance = 1e-6; % 부동소수점 오차 허용
                original_idx = find(abs(distances - front_distance) < tolerance, 1, 'first');
        
                % 해당 인덱스의 차량 정보 추출
                front_vehicle = lane_vehicles(original_idx, :);
            end
        end

        
        function [rear_vehicle, rear_distance] = GetRearVehicle(obj, targetLane, List, Parameter)
            % 현재 차선의 후행 차량 찾기
            current_x = obj.Location * Parameter.Map.Scale;
        
            % 목표 차선 차량 필터링
            lane_vehicles = List.Vehicle.Active(List.Vehicle.Active(:,3) == targetLane, :);
        
            % 모든 차량의 거리 계산
            distances = lane_vehicles(:,4) * Parameter.Map.Scale - current_x;
        
            % 후행 차량 거리 필터링
            rear_distances = distances(distances < 0);
        
            % 초기화
            rear_vehicle = [];
            rear_distance = inf;
        
            if ~isempty(rear_distances)
                % rear_distances 값이 distances에서의 원래 인덱스 찾기
                [rear_distance, ~] = max(rear_distances);
                tolerance = 1e-6;
                original_idx = find(abs(distances - rear_distance) < tolerance, 1, 'first'); % 동일 거리의 원래 인덱스
        
                % 해당 인덱스의 차량 정보 추출
                rear_vehicle = lane_vehicles(original_idx, :);
            end
        end

        function [AccelFlag, DecelFlag, QuitFlag, objVelocity]= LaneChangeWhenNoFeasible(obj,targetLane, Parameter,List)
            AccelFlag = 0;
            DecelFlag = 0;
            QuitFlag = 0;

            % 안전 거리를 만족하지 못하면 감속/가속
            [front_vehicle, front_distance] = GetFrontVehicle(obj, targetLane, List, Parameter);
            [rear_vehicle, rear_distance] = GetRearVehicle(obj, targetLane, List, Parameter);

            if isempty(front_vehicle) || front_distance > Parameter.Veh.SafeDistance
                objVelocity = min(obj.Velocity + obj.Parameter.Accel(1), obj.Parameter.MaxVel);  % 가속해서 합류하기
                AccelFlag = 1;
            
            elseif isempty(rear_vehicle) || abs(rear_distance) > Parameter.Veh.SafeDistance
                objVelocity = max(obj.Velocity - obj.Parameter.Accel(1), obj.Parameter.MinVel);  % 감속해서 합류하기
                DecelFlag = 1;

            else
                objVelocity = obj.Velocity;
                QuitFlag = 1;
                disp("error case!");
            
                % obj.CheckLaneChangeFeasibility(targetLane, List, Parameter);
            end
        end

        

        function MoveVehicle(obj,Time,Parameter,List)
            if obj.LaneChangeFlag == 1
                if obj.Lane > obj.TargetLane
                    % LaneChangeLeft = 1;
                    targetLane = obj.Lane - 1;
                elseif obj.Lane < obj.TargetLane
                    % LaneChangeLeft = 0;
                    targetLane = obj.Lane + 1;
                end

                if  ~obj.CheckLaneChangeFeasibility(targetLane, List, Parameter)
                    [~, ~, QuitFlag, objVelocity] = LaneChangeWhenNoFeasible(obj,targetLane,Parameter,List);
                    if QuitFlag
                        disp('this never happens but added for just in case');
                    else
                        obj.Velocity = objVelocity;
                    end
                    
                else
                    
                end
                % change lane to obj.TargetLane
                new_y = (Parameter.Map.Lane-targetLane+0.5)*Parameter.Map.Tile;
                
                change_steps = 3000; 
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

