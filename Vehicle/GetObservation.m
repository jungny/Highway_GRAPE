function Data = GetObservation(object)
    if object.Location > object.Margin
        if object.Agent == 0
            Data = [sign(object.State) object.Trajectory(1,object.Location) object.Trajectory(2,object.Location) object.Velocity object.Destination];
        else
            Data = [sign(object.State) object.Trajectory(1,object.Location) object.Trajectory(2,object.Location) object.Velocity object.Acceleration];
        end
    else
        if object.Agent == 0
            Data = zeros(1,5);        
        else
            Data = zeros(1,7);
        end
    end
end

