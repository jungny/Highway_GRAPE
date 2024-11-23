classdef Signal < handle
    
    properties % Unique Data
        ID
        Lane
    end

    properties(Hidden = true) % Properties
        Size
        Object
        Patch
        Parameter
    end

    properties(Hidden = true) % Dynamics
        State
        Trajectory
        Location
        Velocity
        Acceleration
    end
    
    methods
        function obj = Signal(Lane,Parameter)
            obj.ID = 0;
            obj.Lane = Lane;
            obj.Trajectory = Parameter.Trajectory.Source{obj.Lane};
           
            obj.Parameter = Parameter.Sig;

            obj.State = 3;
            obj.Location = size(obj.Trajectory,2);
            obj.Velocity = 0;
            obj.Object.Matrix(1:2,4) = obj.Trajectory(:,obj.Location);
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

