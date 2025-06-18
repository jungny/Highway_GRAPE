clc; clear;

T = readtable('param_combinations.csv');
%T = T(ismember(T.ID, 36:39) | ismember(T.ID, 147:152), :);
T = T(T.ID == 35, :);
%T = T(ismember(T.ID, [2,8]), :);

for i = 1:height(T)
    ID = T.ID(i);
    strategy = string(T.Strategy{i});
    radius = T.BubbleRadius(i);
    k_val = T.k(i);
    k_Mode = T.k_Mode(i);
    exitRate = T.ExitRate(i);

    for mode = 0 % [0, 2]. 0: GRAPE, 2: CycleGreedy
        config.ID = ID;
        config.Strategy = strategy;
        config.BubbleRadius = radius;
        config.k = k_val;
        config.k_Mode = k_Mode;
        config.ExitRate = exitRate;
        config.GRAPEmode = mode;
        config.tFixParam = 2; % NoFix이면 NaN
        config.RecordExcel = 1;
        config.RecordVideo = 0; % Excel 기록 시 0(false)로 설정
        config.Iterations = 1; % Excel 기록 시 5로 설정
        config.InitialRandomSeed = 5; % Debug 시에만 1에서 변경

        run_single_simulation(config);
        close all;
    end
    clearvars -except T i
end
