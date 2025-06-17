clc; clear;

T = readtable('param_combinations.csv');
T = T(ismember(T.ID, 157:256), :);  
%T = T(T.ID == 112, :);
%T = T(ismember(T.ID, [3,109,83]), :);

for i = 1:height(T)
    ID = T.ID(i);
    strategy = string(T.Strategy{i});
    radius = T.BubbleRadius(i);
    k_val = T.k(i);
    k_Mode = T.k_Mode(i);
    exitRate = T.ExitRate(i);

    for mode = [0, 2] % [0, 2]. 0: GRAPE, 2: CycleGreedy
        try
            tic; % Start timing
            config.ID = ID;
            config.Strategy = strategy;
            config.BubbleRadius = radius;
            config.k = k_val;
            config.k_Mode = k_Mode;
            config.ExitRate = exitRate;
            config.GRAPEmode = mode;
            config.RecordExcel = 1;
            config.RecordVideo = 0; % Excel 기록 시 0(false)로 설정
            config.Iterations = 5; % Excel 기록 시 5로 설정
            config.InitialRandomSeed = 1; % Debug 시에만 1에서 변경

            run_single_simulation(config);
            close all;
            
            % Check if execution time exceeds 40 minutes (2400 seconds)
            if toc > 2400
                warning('Simulation for ID %d, mode %d exceeded time limit of 40 minutes. Skipping to next iteration.', ID, mode);
                continue;
            end
        catch ME
            warning('Error occurred during simulation for ID %d, mode %d: %s', ID, mode, ME.message);
            continue;
        end
    end
    clearvars -except T i
end
