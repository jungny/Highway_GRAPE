function Rotation = GetRotation(obj,ghostlocation,trajectory)
    if nargin == 3
        Location = ghostlocation;
        Trajectory = trajectory;
    else
        Location = obj.Location;
        Trajectory = obj.Trajectory;
    end

    % 벡터 계산 최적화
    if Location < size(Trajectory,2)
        Vector = Trajectory(:,Location+1)-Trajectory(:,Location);
    else
        Vector = Trajectory(:,end)-Trajectory(:,end-1);
    end
    
    % atan2 계산 (이 부분이 가장 계산량이 많음)
    Angle = atan2(Vector(2), Vector(1));
    
    % 회전 행렬 생성 (cos, sin 계산을 한 번만 수행)
    c = cos(Angle);
    s = sin(Angle);
    Rotation = [c -s; s c];
end

