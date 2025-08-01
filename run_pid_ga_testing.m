function [bestPID, bestFitness] = run_pid_ga_testing(maxGenerations, mutationRate, population)
% RUN_PID_GA_STEADY - Genetic Algorithm for PID Tuning
% Inputs:
%   maxGenerations - number of generations
%   mutationRate   - mutation rate
%   population     - initial population matrix (N x 3)
% Outputs:
%   bestPID        - best [Kp, Ki, Kd]
%   bestFitness    - fitness of best PID

    warning('off', 'all'); % suppress warnings

    % Transfer function: Adjust as needed
    num = 1;
    den = [1 -2 10];
    plant = tf(num, den);

    % Desired performance specs: [rise, settling, overshoot, steady_state_error]
    desired_specs = [0, 0, 0, 0];
    weights = [0.05, 0.7, 0.05, 0.2];

    % PID parameter bounds
    bounds = [0 100; 0 50; 0 10];

    populationSize = size(population, 1);
    globalBestFitness = Inf;
    globalBestIndividual = [0 0 0];

    genBestFitnessHistory = zeros(maxGenerations, 1);
    globalFitnessHistory = zeros(maxGenerations, 1);

    % Main GA loop
    for gen = 1:maxGenerations
        % Evaluate fitness
        fitnessVals = zeros(populationSize, 1);
        for i = 1:populationSize
            fitnessVals(i) = pid_fitness(population(i,:), desired_specs, weights, plant);
        end

        [genBestFitness, bestIdx] = min(fitnessVals);
        genBestFitnessHistory(gen) = genBestFitness;
        globalFitnessHistory(gen) = globalBestFitness;

        % Save first gen response for comparison
        if gen == 1
            bestIndGen1 = population(bestIdx, :);
            pidGen1 = pid(bestIndGen1(1), bestIndGen1(2), bestIndGen1(3));
            sysGen1 = feedback(pidGen1 * plant, 1);
        end

        if genBestFitness < globalBestFitness
            globalBestFitness = genBestFitness;
            globalBestIndividual = population(bestIdx, :);
        end

        if globalBestFitness < 1e-5
            break;
        end

        % --- Reproduction ---
        newPopulation = zeros(size(population));
        invFitVals = 1 ./ fitnessVals;
        sumInvFit = sum(invFitVals);
        normalizedFitnessVals = invFitVals / sumInvFit;

        for i = 1:2:populationSize
            idx1 = roulette_wheel(normalizedFitnessVals);
            idx2 = roulette_wheel(normalizedFitnessVals);
            while idx1 == idx2
                idx2 = roulette_wheel(normalizedFitnessVals);
            end

            p1 = population(idx1, :);
            p2 = population(idx2, :);

            [child1, child2] = crossover(p1, p2, bounds);
            child1 = GA_Mutation(child1, mutationRate);
            child2 = GA_Mutation(child2, mutationRate);

            newPopulation(i,:) = child1;
            newPopulation(i+1,:) = child2;
        end

        population = newPopulation;
    end

    % Final result
    bestPID = globalBestIndividual;
    bestFitness = globalBestFitness;
end
