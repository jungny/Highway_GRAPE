clc; clear;

T = readtable('param_combinations.csv');
%T = T(ismember(T.ID, 107:108), :);  
T = T(T.ID == 107, :);

for i = 1:height(T)
    ID = T.ID(i);
    strategy = string(T.Strategy{i});
    radius = T.BubbleRadius(i);
    k_val = T.k(i);
    k_Mode = T.k_Mode(i);
    exitRate = T.ExitRate(i);

    for mode = [0, 2] % [0, 2]. 0: GRAPE, 2: CycleGreedy
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

        run_single_simulation(config);
        close all;
        %clearvars -except T i config
    end
    clearvars -except T i
end
