function P = GetParameters(Setting)
    
    P.ExitThreshold = 50;
    P.Physics = 0.1;
    P.Control = 0.1;
    
    % Sim
    P.Sim.Time = Setting.Time;
    P.Sim.Data = 5;
        % ID
        % State
        % Lane
        % Location
        % Velocity

    % Map
    P.Map.Color.Road = '#CDCDCD';
    P.Map.Color.Grass = '#B5E3AB';

    P.Map.Scale = 0.01;
    P.Map.Lane = 3;
    P.Map.Tile = 8; %4
    P.Map.Road = 2000;
    P.Map.Margin = 10;
    P.Map.Stop = 6;
    P.Map.Center = [0;0];
    P.Map.Exit = [1040, 1970];

    % Vehicle
    P.Veh.MaxVel = 33; % Original: 10
    P.Veh.DecVel = 1;
    P.Veh.MinVel = 8; % 8
    P.Veh.Accel = [-3 3]; % [1.5 3]
    P.Veh.Size = [4.5 1.9 1.2];
    P.Veh.Buffer = [2.5 0.5];
        P.Veh.State.Out = 0;
        P.Veh.State.Rejected = 1;
        P.Veh.State.Reserved = 2;
        P.Veh.State.Signal = 3;
    P.Veh.Safety = 5; % Original: 2
    P.Veh.Headway = 1.6; % Original: 1.6
    P.Veh.Exp = 4; %4
    P.Veh.SafeDistance = 9;

    % Signal
    P.Sig = 0;

    P.Map.Size = (P.Map.Stop + P.Map.Lane*P.Map.Tile + P.Map.Road)*2;
    
end

