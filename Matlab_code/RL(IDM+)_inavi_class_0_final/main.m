close all
clear
clc
addpath('Map\','Vehicle\','Signal\','Manager\','v2v')

Simulation.Setting.Window = 1000;
Simulation.Setting.Draw = 1;
Simulation.Setting.Debug = 0;
Simulation.Setting.Mode = 1;

    % 1: Dataset Generation
    % 2: Evaluation
Simulation.Setting.Vehicles = 3;
Simulation.Setting.Time = 50;
Simulation.Setting.Datasets = 1;
Simulation.Setting.Agents = 3;
Simulation.Setting.Turns = 1;
Simulation.Setting.Iterations(1,:) = 1:Simulation.Setting.Datasets;
Simulation.Setting.Iterations(2,:) = Simulation.Setting.Agents*ones(1,Simulation.Setting.Datasets);
Simulation.Setting.Iterations(3,:) = [ones(1,1)]; % 2*ones(1,300) 3*ones(1,300) 4*ones(1,300) 5*ones(1,300) 6*ones(1,300) 7*ones(1,300) 8*ones(1,300) 9*ones(1,300) 10*ones(1,300) 11*ones(1,300) 12*ones(1,300) 13*ones(1,300) 14*ones(1,300) 15*ones(1,300) 16*ones(1,300) 17*ones(1,300) 18*ones(1,300) 19*ones(1,300) 20*ones(1,300)];
Simulation.Setting.Iterations(4,:) = randperm(1000000,Simulation.Setting.Datasets);

%% Set Simulation Parameters

Parameter = GetParameters(Simulation.Setting);
GetWindow(Parameter.Map,Simulation.Setting)
Parameter.Trajectory = GetTrajectory(Parameter.Map,Simulation.Setting);
Data = cell(Simulation.Setting.Datasets,2);

%% Run Simulation
priority_order = []
in_intersection = false;
for Iteration = 1:Simulation.Setting.Datasets
    Data{Iteration} = cell(int32(Parameter.Sim.Time/Parameter.Physics +1),Simulation.Setting.Vehicles);
    if Simulation.Setting.Mode == 1
        rng(randi(100000))
    else
        rng(Simulation.Setting.Seed)
    end
    SetMap(Parameter.Map);

    Seed.Vehicle = GetSeed(Simulation.Setting,Parameter,Iteration);
    Data{1,2} = Seed.Vehicle;
    List.Vehicle.Object = cell(size(Seed.Vehicle,2),1);
    List.Signal.Object = cell(4,1);
    for i = 1:4
        List.Signal.Object{i} = Signal(i,Parameter);
    end
    List.Signal.Data = UpdateData(List.Signal.Object,Parameter.Sim.Data);
    Intersection = Manager(Parameter);

    for Time = 0:Parameter.Physics:Parameter.Sim.Time
        title(sprintf('Time: %0.2f s', Time));
        
        % Generate Vehicles
        if ~isempty(Seed.Vehicle)
            if int32(Time/Parameter.Physics) == int32(Seed.Vehicle(2,1)/Parameter.Physics)
                List.Vehicle.Object{Seed.Vehicle(1,1)} = Vehicle(Seed.Vehicle(:,1),Time,Parameter);
                Seed.Vehicle = Seed.Vehicle(:,2:end);
            end
        end
    
        % Update Vehicle Data
        List.Vehicle.Data = UpdateData(List.Vehicle.Object,Parameter.Sim.Data);
        List.Vehicle.Active = List.Vehicle.Data(List.Vehicle.Data(:,2)>0,:);
        List.Vehicle.Object = GetAcceleration(List.Vehicle.Object,List.Vehicle.Data,List.Signal.Data,Parameter.Veh);
    
        % Vehicle stop & random 
        if mod(int32(Time/Parameter.Physics),int32(Parameter.Control/Parameter.Physics)) == 0
            if size(List.Vehicle.Active,1) == 3
                if round(List.Vehicle.Object{1,1}.Velocity,1) == 0.0 &&round(List.Vehicle.Object{2,1}.Velocity,1) == 0 && round(List.Vehicle.Object{3,1}.Velocity,1) == 0 && List.Vehicle.Object{1}.State == 1 &&List.Vehicle.Object{2}.State == 1 &&List.Vehicle.Object{3}.State == 1 
                    priority_order = randperm(3,3);
                end
            end
        end
        
        if ~isempty(priority_order) 
            for i = 1 : size(List.Vehicle.Active,1)
                Trajectory_len = size(List.Vehicle.Object{List.Vehicle.Active(i,1)}.Trajectory);
                if priority_order(List.Vehicle.Active(i,1)) == 1 && in_intersection == false  
                    List.Vehicle.Object{List.Vehicle.Active(i,1)}.State = 2;
                    List.Vehicle.Object{List.Vehicle.Active(i,1)}.Patch = patch('XData',List.Vehicle.Object{List.Vehicle.Active(i,1)}.Size(1,:),'YData',List.Vehicle.Object{List.Vehicle.Active(i,1)}.Size(2,:),'FaceColor','#83D7EC','Parent',List.Vehicle.Object{List.Vehicle.Active(i,1)}.Object);
                    priority_order = priority_order - 1;
                    in_intersection = true;
                elseif priority_order(List.Vehicle.Active(i,1)) == 0 && List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location > Trajectory_len(2) - 7300
                    in_intersection = false;
                end
            end
        end
    
        % Move Vehicle
        for i = 1:size(List.Vehicle.Active,1)
            MoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)},Time)
        end
    
        % Remove Processed Vehicles
        for i = 1:size(List.Vehicle.Active,1)
            if List.Vehicle.Object{List.Vehicle.Active(i,1)}.State == 0
                RemoveVehicle(List.Vehicle.Object{List.Vehicle.Active(i,1)})
                List.Vehicle.Object{List.Vehicle.Active(i,1)} = [];
            end
        end
        % Process Data
        for i = 1:size(List.Vehicle.Object,1)
            if ~isempty(List.Vehicle.Object{i})
                Data{Iteration}{int32(Time/Parameter.Physics+1),i} = List.Vehicle.Object{i}.Data;
            end
        end
    
        % Finalize Time Step
        if Simulation.Setting.Draw == 1
            drawnow();
            pause(0.01)
        end
        if isempty(Seed.Vehicle)
            if isempty(List.Vehicle.Active)
                break
            end
        end

    end

    disp("Iteration: " + Iteration)
    if Time > Parameter.Sim.Time - Parameter.Physics
        ;
    end

     for i = 1:size(Data{Iteration},1)
        for ii = 1:size(List.Vehicle.Object,1)
            if isempty(Data{Iteration}{i,ii})
                if ii < Simulation.Setting.Iterations(2,Iteration) + 1
                    Data{Iteration}{i,ii} = zeros(1,7);        
                else
                    Data{Iteration}{i,ii} = zeros(1,5);
                end
            end
        end
    end


end
