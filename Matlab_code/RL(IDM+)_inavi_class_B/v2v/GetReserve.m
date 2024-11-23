function Obj = GetReserve(Obj,Object,list)
    
    dist_other = Inf;
    for i=1 : size(Object,1)
        dist_other_tmp = Object(i,6);
        if dist_other_tmp < dist_other
            dist_other = dist_other_tmp;
        end
    end


    if isempty(Object)
        Obj.priority = 1;
    end

    if ~isempty(Object) && ~isempty(Obj)
       arrive_tmp = [Obj.arriveTime; Object(:,6);];
       [sorted_values, indices] = sort(arrive_tmp);
       input_index = find(sorted_values == Obj.arriveTime);
       if size(input_index,1) > 1
        Obj.priority = input_index(1);
       else
           Obj.priority = input_index;
       end
       if Obj.priority ~= 1
            object_index = find(Object(:,8) == Obj.priority - 1);
            dist = sqrt((list.Trajectory(1,9000) - list.Trajectory(1,list.Location))^2+(list.Trajectory(2,9000) - list.Trajectory(2,list.Location))^2);
            %list.Acceleration = 2*dist / Object(object_index, 7).^2;%Obj.vel -0.15;
            if isempty(object_index)
                Obj.vel = Obj.vel - 0.3;
            else
                Obj.vel = dist / Object(object_index, 7);
                if Obj.vel > 10
                    Obj.vel = 10;
                end
            end
            list.Velocity = Obj.vel;
       end
    end


    
    %if Obj.arriveTime < dist_other
    %    Obj.priority = 1;
    %end

end