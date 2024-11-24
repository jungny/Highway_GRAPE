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
Simulation.Setting.Exit = [40,100];

%% Set Simulation Parameters

Parameter = GetParameters(Simulation.Setting);
GetWindow(Parameter.Map,Simulation.Setting)
Parameter.Trajectory = GetTrajectory(Parameter.Map,Simulation.Setting);
Data = cell(Simulation.Setting.Datasets,2);

%% Run Simulation
Receive_V2V_check = [false;false;false];
V2V_cancel = [false;false;false];

for Iteration = 1:Simulation.Setting.Datasets
    Data{Iteration} = cell(int32(Parameter.Sim.Time/Parameter.Physics +1),Simulation.Setting.Vehicles);
    if Simulation.Setting.Mode == 1
        rng(randi(100000))
    else
        rng(Simulation.Setting.Seed)
    end
    SetMap(Parameter.Map, Simulation.Setting);

    Seed.Vehicle = GetSeed(Simulation.Setting,Parameter,Iteration);
    Data{1,2} = Seed.Vehicle;
    List.Vehicle.Object = cell(size(Seed.Vehicle,2),1);
    List.Signal.Object = cell(4,1);
    for i = 1:4
        List.Signal.Object{i} = Signal(i,Parameter);
    end
    List.Signal.Data = UpdateData(List.Signal.Object,Parameter.Sim.Data);
    Intersection = Manager(Parameter);
    V2V.Object = cell(3,1);

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
    
        % Msg generate & receive
        if mod(int32(Time/Parameter.Physics),int32(Parameter.Control/Parameter.Physics)) == 0
            for i = 1:size(List.Vehicle.Active,1)
                if V2V_cancel(List.Vehicle.Active(i,1)) == false
                    %generative msg
                    Trajectory_len = size(List.Vehicle.Object{List.Vehicle.Active(i,1)}.Trajectory);
                    if ~isempty(List.Vehicle.Object{List.Vehicle.Active(i,1)}) &&  List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location>1000 && List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location < Trajectory_len(2) - 7300
                        V2V.Object{List.Vehicle.Active(i,1)} = V2VMsg(List.Vehicle.Object{List.Vehicle.Active(i,1)});
                    elseif ~isempty(List.Vehicle.Object{List.Vehicle.Active(i,1)}) &&  List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location > Trajectory_len(2) - 7300
                        V2V.Object{List.Vehicle.Active(i,1)} = {}; %예약 취소
                        V2V_cancel(List.Vehicle.Active(i,1)) = true;
                    end
                    Receive_V2V{i}=[];
                    for j = 1 : size(List.Vehicle.Active,1)
                        if List.Vehicle.Active(j,1) ~= List.Vehicle.Active(i,1) && List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location > 1000
                            if ~isempty(V2V.Object{List.Vehicle.Active(j,1)})
                                Receive_V2V{i}(end+1, :) = ReceiveMsg(V2V.Object{List.Vehicle.Active(j,1)});
                                Receive_V2V_check(i) = true;
                            end
                        end
                    end
                elseif V2V_cancel(List.Vehicle.Active(i,1)) == true
                    List.Vehicle.Object{List.Vehicle.Active(i,1)}.State = 1;
                    List.Vehicle.Object{List.Vehicle.Active(i,1)}.Patch = patch('XData',List.Vehicle.Object{List.Vehicle.Active(i,1)}.Size(1,:),'YData',List.Vehicle.Object{List.Vehicle.Active(i,1)}.Size(2,:),'FaceColor','#34FFA0','Parent',List.Vehicle.Object{List.Vehicle.Active(i,1)}.Object);
                end

            %priority eval
                if ~isempty(List.Vehicle.Object{List.Vehicle.Active(i,1)}) &&  List.Vehicle.Object{List.Vehicle.Active(i,1)}.Location>1000 && V2V_cancel(i) == false
                    V2V.Object{List.Vehicle.Active(i,1)} = GetReserve(V2V.Object{List.Vehicle.Active(i,1)},Receive_V2V{List.Vehicle.Active(i,1)});
                    if V2V.Object{List.Vehicle.Active(i,1)}.priority == 1
                        List.Vehicle.Object{List.Vehicle.Active(i,1)}.State = 2;
                        List.Vehicle.Object{List.Vehicle.Active(i,1)}.Patch = patch('XData',List.Vehicle.Object{List.Vehicle.Active(i,1)}.Size(1,:),'YData',List.Vehicle.Object{List.Vehicle.Active(i,1)}.Size(2,:),'FaceColor','#83D7EC','Parent',List.Vehicle.Object{List.Vehicle.Active(i,1)}.Object);
                    else
                        List.Vehicle.Object{List.Vehicle.Active(i,1)}.Patch = patch('XData',List.Vehicle.Object{List.Vehicle.Active(i,1)}.Size(1,:),'YData',List.Vehicle.Object{List.Vehicle.Active(i,1)}.Size(2,:),'FaceColor','white','Parent',List.Vehicle.Object{List.Vehicle.Active(i,1)}.Object);
                    end
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
