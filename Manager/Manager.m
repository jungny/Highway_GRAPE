classdef Manager < handle
    
    properties % Basic
        TimeStep
        Slot
        TimeSlots
    end

    properties (Hidden = false) % AIM
        Parameter
    end
    
    methods
        function obj = Manager(Parameter)
            obj.TimeStep = Parameter.Physics;
            obj.Slot = zeros(187,167) + 2;
            obj.TimeSlots = repmat(obj.Slot,1,1,10*Parameter.Sim.Time/Parameter.Physics);
            obj.Parameter = Parameter.Veh;
        end

        function Object = RequestReservation(obj,Object)
            ManagerTimeSlot = obj.TimeSlots(:,:,int32(Object.TimeSlot(1)/obj.TimeStep):int32(Object.TimeSlot(2)/obj.TimeStep));
            ReservationCheck = ManagerTimeSlot == Object.Slots(:,:,1:size(ManagerTimeSlot,3));
            if ismember(1,ReservationCheck)
                Object.MaxVel = Object.MaxVel - obj.Parameter.DecVel;
                if Object.MaxVel < obj.Parameter.MinVel
                    Object.MaxVel = obj.Parameter.MaxVel;
                end
            else
                Object.State = obj.Parameter.State.Reserved;
                delete(Object.Patch)
                if Object.Agent == 1
                    Object.Patch = patch('XData',Object.Size(1,:),'YData',Object.Size(2,:),'FaceColor','#83D7EC','Parent',Object.Object);
                else
                    Object.Patch = patch('XData',Object.Size(1,:),'YData',Object.Size(2,:),'FaceColor','#34FFA0','Parent',Object.Object);
                end
                ManagerTimeSlot(Object.Slots == 1) = 1;
                obj.TimeSlots(:,:,int32(Object.TimeSlot(1)/obj.TimeStep):int32(Object.TimeSlot(2)/obj.TimeStep)) = ManagerTimeSlot;
            end
        end

        function ResetReservation(obj,Object)
            ManagerTimeSlot = obj.TimeSlots(:,:,int32(Object.TimeSlot(1)/obj.TimeStep):int32(Object.TimeSlot(2)/obj.TimeStep));
            ManagerTimeSlot(Object.Slots == 1) = 2;
            obj.TimeSlots(:,:,int32(Object.TimeSlot(1)/obj.TimeStep):int32(Object.TimeSlot(2)/obj.TimeStep)) = ManagerTimeSlot;
        end
    end
end

