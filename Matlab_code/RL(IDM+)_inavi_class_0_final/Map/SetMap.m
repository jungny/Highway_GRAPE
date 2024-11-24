function SetMap(Map)
    % Grass 영역 계산 (고속도로 크기 + 마진)
    Margin = Map.Margin;
    GrassX = [-Map.Road/2 - Margin, Map.Road/2 + Margin, ...
               Map.Road/2 + Margin, -Map.Road/2 - Margin];
    GrassY = [-Map.Tile*Map.Lane/2 - Margin, -Map.Tile*Map.Lane/2 - Margin, ...
               Map.Tile*Map.Lane/2 + Margin, Map.Tile*Map.Lane/2 + Margin];

    % Grass 그리기
    patch('XData', GrassX, 'YData', GrassY, ...
          'FaceColor', Map.Color.Grass, 'EdgeColor', 'none');

    % 고속도로 중심 좌표
    Center = [0; 0];
    Length = Map.Road; % 도로의 길이
    Width = Map.Tile * Map.Lane; % 도로 전체 폭

    % 도로 영역 그리기
    Road(1,:) = [-Length/2, Length/2, Length/2, -Length/2];
    Road(2,:) = [-Width/2, -Width/2, Width/2, Width/2];
    patch('XData', Road(1,:) + Center(1), 'YData', Road(2,:) + Center(2), ...
          'FaceColor', Map.Color.Road, 'EdgeColor', 'none');

    % 차선 그리기
    for j = 1:(Map.Lane - 1)
        LineY = -Width/2 + Map.Tile * j;
        plot([-Length/2, Length/2], [LineY, LineY], ...
             'LineStyle', '--', 'Color', 'white', 'LineWidth', 1);
    end
end
