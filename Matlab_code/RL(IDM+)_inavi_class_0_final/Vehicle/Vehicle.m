classdef Vehicle < handle

    properties % Unique Data
        ID
        Lane
        Agent
    end

    properties % Data
        Index
        EntryTime
        ExitTime
        Delay
        Data
        %Reward
        Destination
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
    end

    properties(Hidden = true) % Control
        EnterControl
        ExitControl
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
            obj.Index = [1 2 3 4 1 2 3 4];
            obj.ID = Seed(1);
            obj.Agent = Seed(5);
            obj.EntryTime = Time;
            obj.Lane = Seed(3);
            if Seed(4) == 1
                obj.Destination = obj.Index(obj.Lane+2);
                obj.Trajectory = [Parameter.Trajectory.Source{obj.Lane} Parameter.Trajectory.Through{obj.Lane} Parameter.Trajectory.Sink{obj.Lane}];
            elseif Seed(4) == 2
                obj.Destination = obj.Index(obj.Lane+3);
                obj.Trajectory = [Parameter.Trajectory.Source{obj.Lane} Parameter.Trajectory.Left{obj.Lane} Parameter.Trajectory.Sink{obj.Index(obj.Lane+1)}];
            elseif Seed(4) == 3
                obj.Destination = obj.Index(obj.Lane+1);
                obj.Trajectory = [Parameter.Trajectory.Source{obj.Lane} Parameter.Trajectory.Right{obj.Lane} Parameter.Trajectory.Sink{obj.Index(obj.Lane+3)}];
            end
            obj.EnterControl = size(Parameter.Trajectory.Source{obj.Lane},2);
            obj.ExitControl = size(obj.Trajectory,2) - size(Parameter.Trajectory.Sink{obj.Lane},2);

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

        function MoveVehicle(obj,Time)
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
