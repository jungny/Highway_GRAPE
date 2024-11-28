%msg type
classdef V2VMsg < handle
    properties 
        VIN
        x
        y
        vel
        enterTime
        priority
        waitingTime
    end 
    
    methods
        function obj = V2VMsg(list)
            if nargin > 0
                obj.VIN = list.ID;
                obj.x = list.Trajectory(1,list.Location);
                obj.y = list.Trajectory(2,list.Location);
                obj.vel = list.Velocity;
                obj.enterTime = list.EntryTime;
            else
                obj.VIN = 0;
            end
        end

        function obj = CancelMsg(list)
            if nargin > 0
                obj.VIN = list.ID;
                obj.LaneNumber = list.Lane;
                obj.enterTime = list.EntryTime;
            else
                obj.VIN = 0;
            end
        end
    end
end
