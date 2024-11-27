function GetWindow(Map,~)

    % if Setting.Debug == 1
    %     Setting.Zoom = Map.Margin;
    % else
    %     Setting.Zoom = 0;
    % end
    % 고속도로 크기와 마진 계산
    Margin = Map.Margin;
    MapWidth = Map.Road + Margin * 2; % 고속도로 가로 크기
    MapHeight = Map.Tile * Map.Lane + Margin * 2; % 고속도로 세로 크기

    % 사용 가능한 화면 크기
    ScreenSize = get(0, 'ScreenSize'); % [X, Y, Width, Height]
    ScreenWidth = ScreenSize(3); % 화면 너비
    ScreenHeight = ScreenSize(4); % 화면 높이

    % 창 너비를 화면 너비에 맞춤
    WindowWidth = ScreenWidth;
    AspectRatio = MapWidth / MapHeight; % 가로-세로 비율
    WindowHeight = WindowWidth / AspectRatio; % 비율에 맞는 창 높이

    % 창이 화면 맨 위에 붙도록 위치 설정
    WindowX = 0; % 화면 왼쪽에 붙이기
    WindowY = ScreenHeight - WindowHeight -90; % 화면 맨 위에 붙이기

    % 창 크기와 위치 설정
    set(gcf, 'Position', [WindowX, WindowY, WindowWidth, WindowHeight]);

    % 축 비율 유지
    axis equal;
    xlim([- Margin, Map.Road + Margin]); % 가로 범위
    ylim([-Map.Tile*Map.Lane/2 - Margin, Map.Tile*Map.Lane/2 + Margin]); % 세로 범위

    % 플롯 영역을 창에 꽉 채우도록 조정
    ax = gca; % 현재 축
    ax.Units = 'normalized';
    ax.Position = [0 0 1 1]; % 여백 없이 창을 꽉 채우기
    hold on;
end
