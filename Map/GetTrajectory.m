function Trajectory = GetTrajectory(Map,~)

    Center = Map.Center;

    % 출발점(Source) 경로 설정
    for i = 1:Map.Lane
        % Source 경로: 출발점 → 도로 끝
        Trajectory.Source{i,1}(1,:) = 0:Map.Scale:(Map.Road+50);  % X 좌표
        Trajectory.Source{i,1}(2,:) = Map.Lane*Map.Tile -zeros(1, size(Trajectory.Source{i,1}(1,:), 2)) - (i - 0.5) * Map.Tile; % Y 좌표 (각 차선 중심)
        % 도로 중심 보정
        Trajectory.Source{i,1} = Trajectory.Source{i,1} + Center ;
    end

      
end

