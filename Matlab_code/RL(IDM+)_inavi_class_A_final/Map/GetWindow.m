function GetWindow(Map,Setting)

    if Setting.Debug == 1
        Setting.Zoom = Map.Margin;
    else
        Setting.Zoom = 0;
    end
    xlim([-100 100])%-Map.Size/2-Setting.Zoom Map.Size/2+Setting.Zoom])
    ylim([-100 100])%-Map.Size/2-Setting.Zoom Map.Size/2+Setting.Zoom])
    set(gcf, 'Position',  [0, 0, Setting.Window, Setting.Window])
%     axis off 
    Width = 0.88;
    set(gca, 'Position', [(1-Width)/2 (1-Width)/2-0.005 Width Width])
    hold on

end

