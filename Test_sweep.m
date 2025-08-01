clearvars; close all; clc;

% Parameters
populationSizes = [50, 75, 100];
maxGenVals = [50, 100, 500];
mutationRates = [0.001, 0.01, 0.05];
numTrials = 5;

results = [];

% PID parameter bounds
bounds = [0 100; 0 50; 0 10];

for p = 1:length(populationSizes)
    popSize = populationSizes(p);

    % Same initial population for all trials
    initPopulation = zeros(popSize, 3);
    for i = 1:popSize
        for g = 1:3
            initPopulation(i, g) = rand * (bounds(g,2) - bounds(g,1));
        end
    end

    for i = 1:length(maxGenVals)
        for j = 1:length(mutationRates)
            maxGen = maxGenVals(i);
            mutRate = mutationRates(j);

            fprintf('\nTesting: popSize = %d, maxGen = %d, mutationRate = %.3f\n', ...
                    popSize, maxGen, mutRate);

            fitnesses = zeros(numTrials, 1);
            pids = zeros(numTrials, 3);

            for t = 1:numTrials
                % Use identical initial population
                [bestPID, bestFitness] = run_pid_ga_testing(maxGen, mutRate, initPopulation);
                fitnesses(t) = bestFitness;
                pids(t, :) = bestPID;

                fprintf('Trial %d: Fitness = %.4f | PID = [%.2f %.2f %.2f]\n', ...
                        t, bestFitness, bestPID(1), bestPID(2), bestPID(3));
            end

            % Save results
            results(end+1).populationSize = popSize;
            results(end).maxGen = maxGen;
            results(end).mutationRate = mutRate;
            results(end).avgFitness = mean(fitnesses);
            results(end).stdFitness = std(fitnesses);
            results(end).avgKp = mean(pids(:,1));
            results(end).avgKi = mean(pids(:,2));
            results(end).avgKd = mean(pids(:,3));
        end
    end
end

% Display results
T = struct2table(results);
disp(T);
T = sortrows(T, "avgFitness");
disp('Sorted by average fitness:');
disp(T);

% Extract arrays
populationList = [results.populationSize]';
maxGenList     = [results.maxGen]';
mutationList   = [results.mutationRate]';
avgFitness     = [results.avgFitness]';
stdFitness     = [results.stdFitness]';
avgKp          = [results.avgKp]';
avgKi          = [results.avgKi]';
avgKd          = [results.avgKd]';

uniquePopSizes = unique(populationList);
uniqueMaxGen   = unique(maxGenList);
uniqueMutRate  = unique(mutationList);

% Plot 1: Average Fitness 3D Scatter
figure;
scatter3(maxGenList, mutationList, avgFitness, 100, avgFitness, 'filled');
xlabel('Max Generations'); ylabel('Mutation Rate'); zlabel('Avg Fitness');
title('Average Fitness vs Parameters'); colorbar; grid on; view(135, 25);

% Plot 2: Std Deviation of Fitness
figure;
scatter3(maxGenList, mutationList, stdFitness, 100, stdFitness, 'filled');
xlabel('Max Generations'); ylabel('Mutation Rate'); zlabel('Fitness Std Dev');
title('Fitness Std Dev vs Parameters'); colorbar; grid on; view(135, 25);

% Plot 3: Avg Fitness vs MaxGen (grouped by Mutation Rate)
figure; hold on;
colors = lines(length(uniqueMutRate));
for i = 1:length(uniqueMutRate)
    idx = mutationList == uniqueMutRate(i);
    plot(maxGenList(idx), avgFitness(idx), '-o', 'LineWidth', 2, ...
        'Color', colors(i,:), 'DisplayName', sprintf('\\mu = %.3f', uniqueMutRate(i)));
end
xlabel('Max Generations'); ylabel('Average Fitness');
title('Avg Fitness vs MaxGen (Grouped by Mutation Rate)');
legend show; grid on;

% Plot 4: PID Gain Trends
figure;
subplot(3,1,1);
scatter3(maxGenList, mutationList, avgKp, 100, avgKp, 'filled');
title('Avg Kp'); xlabel('Max Gen'); ylabel('Mutation Rate'); zlabel('Kp'); grid on;

subplot(3,1,2);
scatter3(maxGenList, mutationList, avgKi, 100, avgKi, 'filled');
title('Avg Ki'); xlabel('Max Gen'); ylabel('Mutation Rate'); zlabel('Ki'); grid on;

subplot(3,1,3);
scatter3(maxGenList, mutationList, avgKd, 100, avgKd, 'filled');
title('Avg Kd'); xlabel('Max Gen'); ylabel('Mutation Rate'); zlabel('Kd'); grid on;

% Plot 5: Per Population Size 3D
for p = 1:length(uniquePopSizes)
    popSize = uniquePopSizes(p);
    idx = populationList == popSize;

    figure;
    scatter3(maxGenList(idx), mutationList(idx), avgFitness(idx), ...
             100, avgFitness(idx), 'filled');
    xlabel('Max Generations'); ylabel('Mutation Rate'); zlabel('Avg Fitness');
    title(sprintf('Population Size %d: Avg Fitness vs Parameters', popSize));
    colorbar; grid on; view(135, 25);
end



%% Plot 6: Avg Fitness vs Population Size (Grouped by Mutation Rate)
figure; hold on;
colors = lines(length(uniqueMutRate));
for i = 1:length(uniqueMutRate)
    mu = uniqueMutRate(i);
    idx = mutationList == mu;

    % Group by population size
    popSizes_mu = unique(populationList(idx));
    avgFitness_mu = arrayfun(@(ps) mean(avgFitness(idx & populationList == ps)), popSizes_mu);

    plot(popSizes_mu, avgFitness_mu, '-o', 'LineWidth', 2, ...
        'DisplayName', sprintf('\\mu = %.3f', mu), 'Color', colors(i,:));
end
xlabel('Population Size'); ylabel('Average Best Fitness');
title('Average Fitness vs Population Size (Grouped by Mutation Rate)');
legend show; grid on;

%% Plot 7: Avg Fitness vs Population Size (Grouped by Max Generation)
figure; hold on;
colors = lines(length(uniqueMaxGen));
for i = 1:length(uniqueMaxGen)
    mg = uniqueMaxGen(i);
    idx = maxGenList == mg;

    popSizes_mg = unique(populationList(idx));
    avgFitness_mg = arrayfun(@(ps) mean(avgFitness(idx & populationList == ps)), popSizes_mg);

    plot(popSizes_mg, avgFitness_mg, '-s', 'LineWidth', 2, ...
        'DisplayName', sprintf('MaxGen = %d', mg), 'Color', colors(i,:));
end
xlabel('Population Size'); ylabel('Average Best Fitness');
title('Average Fitness vs Population Size (Grouped by Max Generation)');
legend show; grid on;


%% Plot 8: Avg Fitness vs MaxGen for each (Population, Mutation Rate) pair
figure; hold on;

% Use unique color for each line
colors = lines(length(uniquePopSizes) * length(uniqueMutRate));
lineCount = 1;
legendEntries = {};

for i = 1:length(uniquePopSizes)
    for j = 1:length(uniqueMutRate)
        pop = uniquePopSizes(i);
        mut = uniqueMutRate(j);

        % Filter results
        idx = populationList == pop & mutationList == mut;
        gens = maxGenList(idx);
        fitness = avgFitness(idx);

        % Sort by maxGen to get proper line plot
        [gens_sorted, sortIdx] = sort(gens);
        fitness_sorted = fitness(sortIdx);

        % Plot
        plot(gens_sorted, fitness_sorted, '-o', ...
            'Color', colors(lineCount,:), ...
            'LineWidth', 2);

        % Label
        legendEntries{end+1} = sprintf('Pop %d, \\mu = %.3f', pop, mut);
        lineCount = lineCount + 1;
    end
end

xlabel('Max Generations');
ylabel('Average Best Fitness');
title('Avg Fitness vs MaxGen (All Pop/Mutation Pairs)');
legend(legendEntries, 'Location', 'bestoutside');
grid on;
