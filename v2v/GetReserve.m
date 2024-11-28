function Obj = GetReserve(Obj,Object)
    
    dist = sqrt(abs(Obj.x^2 + Obj.y^2));
    dist_other = Inf;
    for i=1 : size(Object,1)
        dist_other_tmp = sqrt(abs(Object(i,2)^2 + Object(i,3)^2));
        if dist_other_tmp < dist_other
            dist_other = dist_other_tmp
        end
    end

    if dist < dist_other
        Obj.priority = 1;
    end

end