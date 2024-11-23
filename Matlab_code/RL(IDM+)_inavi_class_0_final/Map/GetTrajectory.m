function Trajectory = GetTrajectory(Map,Setting)

    Center = Map.Center;

    for i = 1:4
        Length = Map.Tile*Map.Lane + Map.Road+Map.Margin+Map.Stop;

        Trajectory.Source{i,1}(1,:) = Map.Tile*Map.Lane+Map.Stop:Map.Scale:Length;
        Trajectory.Source{i,1}(2,:) = zeros(1,size(Trajectory.Source{i,1}(1,:),2))+Map.Tile/2;
        Trajectory.Source{i,1} = flip([0 -1;1 0]^i*Trajectory.Source{i,1} + Center,2);
        Trajectory.Source{i,1} = Trajectory.Source{i,1}(:,1:end-1);
      
        Trajectory.Sink{i,1}(1,:) = -Map.Tile*Map.Lane-Map.Stop:-Map.Scale:-Length+Map.Margin/2;
        Trajectory.Sink{i,1}(2,:) = zeros(1,size(Trajectory.Sink{i,1}(1,:),2))+Map.Tile/2;
        Trajectory.Sink{i,1} = [0 -1;1 0]^i*Trajectory.Sink{i,1} + Center;
        Trajectory.Sink{i,1} = Trajectory.Sink{i,1}(:,1:end-1);
    end
    for i = 1:4
        Trajectory.Through{i,1}(1,:) = Map.Tile*Map.Lane+Map.Stop:-Map.Scale:-Map.Tile*Map.Lane-Map.Stop;
        Trajectory.Through{i,1}(2,:) = zeros(1,size(Trajectory.Through{i,1}(1,:),2))+Map.Tile/2;
        Trajectory.Through{i,1} = [0 -1;1 0]^i*Trajectory.Through{i,1} + Center;
        Trajectory.Through{i,1} = Trajectory.Through{i,1}(:,1:end-1);

        RY = Map.Tile*Map.Lane+Map.Tile/2+Map.Stop;
        RX = Map.Tile*Map.Lane+Map.Tile/2+Map.Stop;
        h = ((RY-RX)^2)/((RY+RX)^2);
        C = pi*(RY+RX)*(1+(3*h/(10+sqrt(4-3*h))))/4;
        Distance = linspace(0,pi/2,C/Map.Scale);
        Trajectory.Left{i,1}(1,:) = Map.Tile*Map.Lane+Map.Stop-RX*cos(Distance);
        Trajectory.Left{i,1}(2,:) = -Map.Tile*Map.Lane-Map.Stop+RY*sin(Distance);
        Trajectory.Left{i,1} = flip([0 -1;1 0]^i*Trajectory.Left{i,1} + Center,2);
        Trajectory.Left{i,1} = Trajectory.Left{i,1}(:,1:end-1);

        RY = Map.Tile/2+Map.Stop;
        RX = Map.Tile/2+Map.Stop;
        h = ((RY-RX)^2)/((RY+RX)^2);
        C = pi*(RY+RX)*(1+(3*h/(10+sqrt(4-3*h))))/4;
        Distance = linspace(0,pi/2,C/Map.Scale);
        Trajectory.Right{i,1}(1,:) = Map.Tile*Map.Lane+Map.Stop-RX*sin(Distance);
        Trajectory.Right{i,1}(2,:) = -Map.Tile*Map.Lane-Map.Stop+RY*cos(Distance);
        Trajectory.Right{i,1} = flip([0 -1;1 0]^(i+1)*Trajectory.Right{i,1} + Center,2);
        Trajectory.Right{i,1} = Trajectory.Right{i,1}(:,1:end-1);

    end

    if Setting.Debug == 1
        for i = 1:4

            plot(Trajectory.Source{i,1}(1,:),Trajectory.Source{i,1}(2,:))
            plot(Trajectory.Sink{i,1}(1,:),Trajectory.Sink{i,1}(2,:))
            plot(Trajectory.Through{i,1}(1,:),Trajectory.Through{i,1}(2,:))

            plot(Trajectory.Left{i,1}(1,:),Trajectory.Left{i,1}(2,:))

            plot(Trajectory.Right{i,1}(1,:),Trajectory.Right{i,1}(2,:))

        end
    end

        
end

