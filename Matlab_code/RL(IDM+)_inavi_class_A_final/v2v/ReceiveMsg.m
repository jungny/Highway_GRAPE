function Obj = ReceiveMsg(obj)

    Obj(1, :) = obj.VIN;
    Obj(2, :) = obj.x;
    Obj(3, :) = obj.y;
    Obj(4, :) = obj.vel;
    Obj(5, :) = obj.enterTime;

end 
