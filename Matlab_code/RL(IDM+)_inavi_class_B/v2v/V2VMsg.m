%msg type
classdef V2VMsg < handle
    properties 
        VIN
        x
        y
        vel
        enterTime
        arriveTime
        exitTime
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
                if list.Trajectory(1,list.Location) <= 9000
                    obj.arriveTime = sqrt(((list.Trajectory(1,9000) - list.Trajectory(1,list.Location))^2+(list.Trajectory(2,9000) - list.Trajectory(2,list.Location))^2))/list.Velocity;
                else
                    obj.arriveTime = inf;
                end

                exit = size(list.Trajectory,2) - 7600;
                obj.exitTime = sqrt((list.Trajectory(1,exit) - list.Trajectory(1,list.Location))^2+(list.Trajectory(2,exit) - list.Trajectory(2,list.Location))^2)/list.Velocity;
                obj.priority = 0;
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
