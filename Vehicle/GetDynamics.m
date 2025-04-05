function [nextVelocity,nextLocation] = GetDynamics(obj,ghostAcceleration,ghostVelocity,ghostLocation)

    if nargin > 1
        Velocity = ghostVelocity;
        Location = ghostLocation;
        Acceleration = ghostAcceleration;
    else
        Velocity = obj.Velocity;
        Location = obj.Location;
        Acceleration = obj.Acceleration;
    end

    if isempty(Acceleration)
        Acceleration = 0; % 빈 값이면 가속도를 0으로 설정
    end

    if obj.IsChangingLane
        correction = sqrt(1 + (obj.ParameterMap.Tile / (3000*obj.ParameterMap.Scale))^2);  % ≈ sqrt(1.010)
    else
        correction = 1;
    end
    

    if Velocity + obj.TimeStep*Acceleration > obj.Parameter.MaxVel
        nextVelocity = obj.Parameter.MaxVel;
        if Velocity > obj.Parameter.MaxVel
            if Acceleration > 0
                Acceleration = obj.Parameter.Accel(2)/3;
            end
            actualTimeTaken = abs((Velocity - obj.Parameter.MaxVel)/Acceleration);
            nextLocation = double(Location + uint32(obj.DistanceStep/correction*(obj.TimeStep*obj.Parameter.MaxVel + 0.5*(Velocity - obj.Parameter.MaxVel)*actualTimeTaken)));
        elseif Velocity < obj.Parameter.MaxVel
            actualTimeTaken = abs((obj.Parameter.MaxVel - Velocity)/obj.Parameter.Accel(1));
            nextLocation = double(Location + uint32(obj.DistanceStep/correction*(obj.TimeStep*obj.Parameter.MaxVel-(obj.Parameter.MaxVel-Velocity)*actualTimeTaken*0.5)));
        else
            nextLocation = double(Location + uint32(obj.DistanceStep/correction*(obj.TimeStep*Velocity)));
        end
    elseif Velocity + obj.TimeStep*Acceleration < 0
        nextVelocity = 0;
        actualTimeTaken = abs(Velocity/Acceleration);
        nextLocation = double(Location + uint32(obj.DistanceStep/correction*0.5*Velocity*actualTimeTaken));
    else
        nextVelocity = Velocity + obj.TimeStep*Acceleration;
        try
            nextLocation = double(Location + uint32(obj.DistanceStep/correction*(0.5*obj.TimeStep*(Velocity + Velocity + Acceleration))));
        catch ME
            disp('n ');
        end
    end
end
