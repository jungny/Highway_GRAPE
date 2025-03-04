function GetWindow(Map, ~)
    %viewType = 'Default';
    viewType = 'FullHighway';
    %viewType = 'ScrollableHighway';
    % viewType에 따라 창을 생성
    switch viewType
        case 'Default'
            CreateDefaultWindow(Map);
        case 'FullHighway'
            CreateFullHighwayView(Map);
        case 'ScrollableHighway'
            CreateScrollableHighwayView(Map);
        otherwise
            error('Invalid viewType. Choose from "Default", "FullHighway", or "ScrollableHighway".');
    end
end

function CreateDefaultWindow(Map)
    % 기본 창 설정
    Margin = Map.Margin;
    MapWidth = Map.Road + Margin * 2;
    MapHeight = Map.Tile * Map.Lane + Margin * 2;

    ScreenSize = get(0, 'ScreenSize');
    ScreenWidth = ScreenSize(3);
    ScreenHeight = ScreenSize(4);

    WindowWidth = ScreenWidth; % 창 너비를 스크린 너비로 설정
    AspectRatio = MapWidth / MapHeight;
    WindowHeight = WindowWidth / AspectRatio;

    WindowX = 0;
    WindowY = ScreenHeight - WindowHeight - 90; % 통일된 Y 위치

    figure('Name', 'Default Highway View');
    set(gcf, 'Position', [WindowX, WindowY, WindowWidth, WindowHeight]);
    axis equal;
    xlim([-Margin, Map.Road + Margin]);
    ylim([-Margin, Map.Tile * Map.Lane + Margin]);

    ax = gca;
    ax.Units = 'normalized';
    ax.Position = [0 0 1 1];
    hold on;

    % 맵 표시 (필요시 활성화)
    % plotMap(Map);
end

function CreateFullHighwayView(Map)
    % 축소된 전체 고속도로 뷰 창
    Margin = Map.Margin;
    ScreenSize = get(0, 'ScreenSize');
    ScreenWidth = ScreenSize(3);
    ScreenHeight = ScreenSize(4);

    WindowWidth = ScreenWidth+2000; % 창 너비를 스크린 너비로 설정
    AspectRatio = Map.Road / (Map.Tile * Map.Lane);
    WindowHeight = 120;

    WindowX = 0; % X 위치 통일
    WindowY = ScreenHeight - WindowHeight - 90; % 통일된 Y 위치

    figure('Name', 'Full Highway View');
    set(gcf, 'Position', [WindowX, WindowY, WindowWidth, WindowHeight]);
    axis equal;
    xlim([-Margin, Map.Road + Margin]);
    ylim([-Margin, Map.Tile * Map.Lane + Margin]);

    ax = gca;
    ax.Units = 'normalized';
    ax.Position = [0 0 1 1];
    hold on;

    % 맵 표시 (필요시 활성화)
    % plotMap(Map);
end

function CreateScrollableHighwayView(Map)
    % 스크롤 가능한 창
    Margin = Map.Margin;
    ScreenSize = get(0, 'ScreenSize');
    ScreenWidth = ScreenSize(3);
    ScreenHeight = ScreenSize(4);

    scroll_position = 0; % 초기 스크롤 위치
    window_length = 400; % 스크롤 창 초기 길이

    WindowWidth = ScreenWidth; % 창 너비를 스크린 너비로 설정
    AspectRatio = window_length / (Map.Tile * Map.Lane);
    WindowHeight = WindowWidth / AspectRatio;

    WindowX = 0; % X 위치 통일
    WindowY = ScreenHeight - WindowHeight - 90; % 통일된 Y 위치

    

    f = figure('Name', 'Scrollable Highway View');
    set(f, 'Position', [WindowX, WindowY, WindowWidth, WindowHeight]);

    function updateView()
        clf(f);
        axis equal;
        xlim([scroll_position, scroll_position + window_length]);
        ylim([-Margin, Map.Tile * Map.Lane + Margin]);
        hold on;

        % 맵 표시 (필요시 활성화)
        % plotMap(Map);
    end

    updateView();

    % 스크롤 버튼 추가
    uicontrol(f, 'Style', 'pushbutton', 'String', 'Left', 'Position', [10, 10, 50, 20], ...
              'Callback', @(~, ~) moveLeft());
    uicontrol(f, 'Style', 'pushbutton', 'String', 'Right', 'Position', [70, 10, 50, 20], ...
              'Callback', @(~, ~) moveRight());

    function moveLeft()
        scroll_position = max(0, scroll_position - window_length / 2);
        updateView();
    end

    function moveRight()
        scroll_position = min(Map.Road - window_length, scroll_position + window_length / 2);
        updateView();
    end
end

