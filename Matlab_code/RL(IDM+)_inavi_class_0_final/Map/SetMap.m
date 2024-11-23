function SetMap(Map)
    
    Center = Map.Center;

    Field(1,:) = [Map.Size/2 -Map.Size/2 -Map.Size/2 Map.Size/2];
    Field(2,:) = [Map.Size/2 Map.Size/2 -Map.Size/2 -Map.Size/2];
    
    patch('XData',Field(1,:),'YData',Field(2,:),'FaceColor',Map.Color.Grass,'EdgeColor','none')
    % Roads
    for i = 1:4
        LengthR = Map.Tile*Map.Lane + Map.Road + Map.Stop;
        LengthL = Map.Tile*Map.Lane + Map.Road + Map.Stop;
        for j = 1:Map.Lane
            Road(1,:) = [LengthR -LengthL -LengthL LengthR];
            Road(2,:) = [Map.Tile Map.Tile 0 0] + Map.Tile*(j-1);
            Road = [0 -1;1 0]^i*Road + Center;
            Line = [0 -1;1 0]^i*[-LengthL LengthR;Map.Tile*(j-1) Map.Tile*(j-1)] + Center;
            patch('XData',Road(1,:),'YData',Road(2,:),'FaceColor',Map.Color.Road,'EdgeColor','none')
            plot(Line(1,:),Line(2,:),'LineStyle','--','Color','white')
        end
    end
    % Stop Line
    for i = 1:4
        StopLoc = Map.Tile*Map.Lane+Map.Stop;
        StopLine(1,:) = StopLoc + [0.5 -0.5 -0.5 0.5];
        StopLine(2,:) = [Map.Tile*Map.Lane Map.Tile*Map.Lane 0 0];
        StopLine = [0 -1;1 0]^i*StopLine + Center;
        patch('XData',StopLine(1,:),'YData',StopLine(2,:),'FaceColor','white','EdgeColor','none')
    end
    % Right Turn
    t = linspace(0,pi/2,50);
    for i = 1:4
        Curve = [];
        Curve(1,:) = Map.Tile*Map.Lane+Map.Stop - Map.Stop*sin(t);
        Curve(2,:) = Map.Tile*Map.Lane+Map.Stop - Map.Stop*cos(t);
        Curve = [[Map.Tile*Map.Lane;Map.Tile*Map.Lane] Curve [Map.Tile*Map.Lane;Map.Tile*Map.Lane]];
        Curve = [0 -1;1 0]^i*Curve + Center;
        patch('XData',Curve(1,:),'YData',Curve(2,:),'FaceColor',Map.Color.Road,'EdgeColor','none')
    end

    % Black Line
    for i = 1:4
        LengthR = Map.Tile*Map.Lane + Map.Road + Map.Stop;
        LengthL = Map.Tile*Map.Lane + Map.Road + Map.Stop;
        Line = [0 -1;1 0]^i*[0 LengthR; 0 0] + Center;
        plot(Line(1,:),Line(2,:),'LineStyle','-','Color','#FFE438','LineWidth',2)
    end
    % Fill Patch
    StopHorz(1,:) = Center(1) + [Map.Tile*Map.Lane+Map.Stop -Map.Tile*Map.Lane-Map.Stop -Map.Tile*Map.Lane-Map.Stop Map.Tile*Map.Lane+Map.Stop];
    StopHorz(2,:) = Center(2) + [Map.Tile*Map.Lane Map.Tile*Map.Lane -Map.Tile*Map.Lane -Map.Tile*Map.Lane];
    patch('XData',StopHorz(1,:),'YData',StopHorz(2,:),'FaceColor',Map.Color.Road,'EdgeColor','none')
    StopVert(1,:) = Center(1) + [Map.Tile*Map.Lane -Map.Tile*Map.Lane -Map.Tile*Map.Lane Map.Tile*Map.Lane];
    StopVert(2,:) = Center(2) + [Map.Tile*Map.Lane+Map.Stop Map.Tile*Map.Lane+Map.Stop -Map.Tile*Map.Lane-Map.Stop -Map.Tile*Map.Lane-Map.Stop];
    patch('XData',StopVert(1,:),'YData',StopVert(2,:),'FaceColor',Map.Color.Road,'EdgeColor','none')
    Intersection(1,:) = Center(1) + [Map.Tile*Map.Lane -Map.Tile*Map.Lane -Map.Tile*Map.Lane Map.Tile*Map.Lane] + Center(1);
    Intersection(2,:) = Center(2) + [Map.Tile*Map.Lane Map.Tile*Map.Lane -Map.Tile*Map.Lane -Map.Tile*Map.Lane] + Center(2);
    patch('XData',Intersection(1,:),'YData',Intersection(2,:),'FaceColor',Map.Color.Road,'EdgeColor','none')



end

