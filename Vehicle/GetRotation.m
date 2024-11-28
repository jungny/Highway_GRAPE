function Rotation = GetRotation(obj,ghostlocation,trajectory)

    if nargin == 3
        Location = ghostlocation;
        Trajectory = trajectory;
    else
        Location = obj.Location;
        Trajectory = obj.Trajectory;
    end

    if Location < size(Trajectory,2)
        Vector = Trajectory(:,Location+1)-Trajectory(:,Location);
    else
        Vector = Trajectory(:,end)-Trajectory(:,end-1);
    end
    Angle = atan2(Vector(2),Vector(1));
    Rotation = [cos(Angle) -sin(Angle);sin(Angle) cos(Angle)];
    
end

