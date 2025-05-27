clc; clear;

T = readtable('param_combinations.csv');
T = T(ismember(T.ID, 31:61), :);  % ID가 2~61인 것만

for i = 1:height(T)
    ID = T.ID(i);
    strategy = string(T.Strategy{i});
    radius = T.BubbleRadius(i);
    k_val = T.k(i);
    exitRate = T.ExitRate(i);

    for mode = [0, 2]
        config.ID = ID;
        config.Strategy = strategy;
        config.BubbleRadius = radius;
        config.k = k_val;
        config.ExitRate = exitRate;
        config.GRAPEmode = mode;

        run_single_simulation(config);
        close all;
        clearvars -except T i config
    end
end
