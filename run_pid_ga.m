% run_pid_ga_steady.m
% Genetic Algorithm: Parents produce 2 children, replace themselves


clearvars; close all; clc;  %clean start
warning('off', 'all'); % to avoid warnings about unstable systems

% --- PARAMETERS ---
populationSize = 80;
maxGenerations = 100;
fitnessThreshold = 0.00001;
mutationRate = 0.00000001;
globalBestFitness = 5;  %will be updated each generation
globalBestindividual = [0;0;0]; %will be updated each generation

% Define a sample motor transfer function (adjust as needed)
% Example: Second-order plant G(s) = 1 / (s^2 - 2s + 10)
num = 1;
den = [1 -2 10];
plant = tf(num, den);


%--- PREALLOCATE PLOTTING PARAMETERS---
genBestFitnessHistory = zeros(maxGenerations, 1);  
globalFitnessHistory = zeros(maxGenerations, 1);


% --- SPEC ---
%desired_specs = [0.5, 2.0, 5];   % [rise, settling, overshoot, steady_state_error]
desired_specs = [0, 0, 0, 0];

weights = [0.3, 0.3, 0.2, .2];


% --- BOUNDS for [Kp, Ki, Kd] ---
bounds = [0 100; 0 50; 0 10];

% --- INIT POPULATION ---
population = zeros(populationSize, 3);
for i = 1:populationSize
    for g = 1:3
        population(i,g) = rand * (bounds(g,2) - bounds(g,1));
    end
end

% --- MAIN LOOP ---
for gen = 1:maxGenerations
   
    % Evaluate fitness
    fitnessVals = zeros(populationSize, 1);
    for i = 1:populationSize
        fitnessVals(i) = pid_fitness(population(i,:), desired_specs, weights, plant);
    end
    
    %find the best fitness for the generation and print the result
    [genBestFitness, bestIdx] = min(fitnessVals);
    fprintf('Gen %d | Best Fitness: %.4f\n', gen, genBestFitness);
    genBestFitnessHistory(gen) = genBestFitness; %for plotting

    % --- Plot Step Response for Best of First Generation ---
    if gen == 1
        bestIndGen1 = population(bestIdx, :);
        Kp1 = bestIndGen1(1);
        Ki1 = bestIndGen1(2);
        Kd1 = bestIndGen1(3);

        pidGen1 = pid(Kp1, Ki1, Kd1);
        sysGen1 = feedback(pidGen1 * plant, 1);
    end
    
    %keep track of the globalBest for plotting
    globalFitnessHistory(gen) = globalBestFitness;

    
    %Update the global best if applicable
    if genBestFitness < globalBestFitness
        globalBestFitness = genBestFitness;
        globalBestIndividual = population(bestIdx, :);
    end
    
    %if goal has been reached then end
    if globalBestFitness < fitnessThreshold
        disp('✅ Target fitness reached.');
        break;
    end

    % --- New Generation: Pairwise crossover & mutation ---
    newPopulation = zeros(size(population));
    for i = 1:2:populationSize
        % Select 2 parents randomly
        invFitVals=fitnessVals.^(-1);
        sumInvFit=sum(invFitVals);

        normalizedFitnessVals = zeros(populationSize, 1);
        for n=1:populationSize
            normalizedFitnessVals(n)=invFitVals(n)/sumInvFit;
        end

        idx(1)=roulette_wheel(normalizedFitnessVals);
        idx(2)=roulette_wheel(normalizedFitnessVals);

        
        while(idx(1)==idx(2))
            idx(2)=roulette_wheel(normalizedFitnessVals);
        end

        p1 = population(idx(1), :);
        p2 = population(idx(2), :);
       
        % Crossover
        [child1, child2] = crossover(p1, p2, bounds);

        % Mutation
        child1 = GA_Mutation(child1, mutationRate, bounds);
        child2 = GA_Mutation(child2, mutationRate, bounds);

        % Add children to new population
        newPopulation(i,:)   = child1;
        newPopulation(i+1,:) = child2;
    end
    % Replace entire population
    population = newPopulation;
end

% --- Final Result ---
fprintf('\nBest PID: Kp=%.2f, Ki=%.2f, Kd=%.2f | Fitness: %.4f\n', ...
        globalBestIndividual(1), globalBestIndividual(2), globalBestIndividual(3), globalBestFitness);

%plot the convergence over time
figure(1);
plot(1:maxGenerations, globalFitnessHistory);
xlabel('Generation'); ylabel('Best Fitness');
title('GA Convergence');
grid on;

figure(2);
plot(1:gen, genBestFitnessHistory(1:gen), 'b-', 'LineWidth', 2);
xlabel('Generation');
ylabel('Best Fitness');
title('Best Fitness per Generation');
grid on;

% --- Step Response of Best PID ---
Kp = globalBestIndividual(1);
Ki = globalBestIndividual(2);
Kd = globalBestIndividual(3);

% Rebuild best PID controller
bestPID = pid(Kp, Ki, Kd);

% Create closed-loop system using the same plant
best_sys_cl = feedback(bestPID * plant, 1);

% Step response info
info = stepinfo(best_sys_cl);

% Display final performance metrics
fprintf('\nFinal Step Response Characteristics:\n');
fprintf('Rise Time     : %.4f s\n', info.RiseTime);
fprintf('Settling Time : %.4f s\n', info.SettlingTime);
fprintf('Overshoot     : %.2f %%\n', info.Overshoot);

% Plot the step response
figure(3);
step(best_sys_cl); % First system
hold on;
step(sysGen1); % Second system
hold off;

%title(sprintf('Step Response of Best PID: Kp=%.2f, Ki=%.2f, Kd=%.2f', Kp, Ki, Kd));
title(sprintf('PID Step Response'));
legend('Best PID', 'Initial PID'); % Update labels as needed
grid on;
