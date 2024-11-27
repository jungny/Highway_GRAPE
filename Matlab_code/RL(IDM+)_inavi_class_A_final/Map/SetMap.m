function SetMap(Map, Setting)
    % Grass 영역 계산 (고속도로 크기 + 마진)
    Margin = Map.Margin;
    GrassX = [ - Margin, Map.Road + Margin, ...
               Map.Road + Margin, - Margin];
    GrassY = [ - Margin,  - Margin, ...
               Map.Tile*Map.Lane + Margin, Map.Tile*Map.Lane + Margin];

    % Grass 그리기
    patch('XData', GrassX, 'YData', GrassY, ...
          'FaceColor', Map.Color.Grass, 'EdgeColor', 'none');
    
    
    Length = Map.Road; % 도로의 길이
    Center = [0; 0]; % 고속도로 중심 좌표
    Width = Map.Tile * Map.Lane; % 도로 전체 폭

    % 도로 영역 그리기
    Road(1,:) = [0, Length, Length, 0];
    Road(2,:) = [0, 0, Width, Width];
    patch('XData', Road(1,:) + Center(1), 'YData', Road(2,:) + Center(2), ...
          'FaceColor', Map.Color.Road, 'EdgeColor', 'none');

    % Exit 도로 추가
    for i = 1:length(Setting.Exit)
        ExitStart = Setting.Exit(i); % Exit 시작 위치
        ExitLaneStart = 0; % Exit은 제일 아래 차선에서 시작
        LaneWidth = Width / Map.Lane; % 차선 너비
        RectWidth = 20; % Exit 직사각형의 너비
        RectHeight = LaneWidth; % Exit 직사각형의 높이
        RotationAngle = -18; % 회전 각도 (시계 방향)

        % 1. 직사각형 좌표 생성
        RectX = [0, RectWidth, RectWidth, 0];
        RectY = [0, 0, -RectHeight, -RectHeight];

        % 2. 좌표 회전
        RotationMatrix = [cosd(RotationAngle), -sind(RotationAngle); ...
                          sind(RotationAngle), cosd(RotationAngle)];
        RotatedCoords = RotationMatrix * [RectX; RectY+4];

        % 3. 기준점 이동
        RotatedX = RotatedCoords(1, :) + ExitStart;
        RotatedY = RotatedCoords(2, :) + ExitLaneStart;

        % 4. Exit 도로 렌더링
        patch('XData', RotatedX, 'YData', RotatedY, ...
              'FaceColor', Map.Color.Road, 'EdgeColor', 'none');
        
        % 5. Exit 위치에 동그라미 추가
        CircleX = ExitStart; % 동그라미의 X 좌표 (Exit 시작점)
        CircleY = LaneWidth/2;
        CircleSize = 3; % 동그라미 크기
        plot(CircleX, CircleY, 'o', 'MarkerSize', CircleSize, ...
             'MarkerFaceColor', [0.6, 0.3, 0.1], 'MarkerEdgeColor', 'none'); % 갈색 동그라미

    end

    % 차선 그리기
    for j = 1:(Map.Lane - 1)
        LineY = -Width*0 + Map.Tile * j;
        plot([0, Length], [LineY, LineY], ...
             'LineStyle', '--', 'Color', 'white', 'LineWidth', 1);
    end
end
