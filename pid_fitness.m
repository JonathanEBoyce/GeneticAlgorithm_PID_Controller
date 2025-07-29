function fitness = pid_fitness(individual, desired_specs, weights, plant)
    % Extract PID gains from the individual
    Kp = individual(1);
    Ki = individual(2);
    Kd = individual(3);

    % Desired specs [rise_time, settling_time, overshoot]
    desired_rise_time = desired_specs(1);
    desired_settling_time = desired_specs(2);
    desired_overshoot = desired_specs(3);
    desired_steady_state_error = desired_specs(4);

    % Weights [w1, w2, w3]
    w1 = weights(1);
    w2 = weights(2);
    w3 = weights(3);
    w4 = weights(4);

    % Define a sample motor transfer function (adjust as needed)
    % Example: Second-order plant G(s) = 1 / (s^2 + 10s + 20)
    %num = 1;
    %den = [1 10 20];
    %plant = tf(num, den);

    % Create PID controller
    controller = pid(Kp, Ki, Kd);

    % Closed-loop system
    sys_cl = feedback(controller * plant, 1);

    % Step response info
    info = stepinfo(sys_cl);

    % Suppress specific warning temporarily
    warnID = 'Control:stepinfo:SimulationNotSteady';
    warning('off', warnID);

    % Defensive programming: fill with large error if simulation failed
    if isnan(info.SettlingTime) || isnan(info.RiseTime) || isnan(info.Overshoot)
        fitness = 1e9;  % Massive penalty
        return;
    end

    % Extract actual values
    actual_rise_time = info.RiseTime;
    actual_settling_time = info.SettlingTime;
    actual_overshoot = info.Overshoot;
    actual_steady_state_error = abs(1 - dcgain(sys_cl));


    % Compute absolute errors
    rise_time_error = abs(actual_rise_time - desired_rise_time);
    settling_time_error = abs(actual_settling_time - desired_settling_time);
    overshoot_error = abs(actual_overshoot - desired_overshoot);
    steady_state_error = abs(actual_steady_state_error - desired_steady_state_error)*10;
    

    % Compute fitness (lower is better)
    fitness = w1 * rise_time_error + ...
              w2 * settling_time_error + ...
              w3 * overshoot_error + ...
              w4 * steady_state_error;

    % Optional: Add penalty for unstable systems
    if ~isstable(sys_cl)
        fitness = fitness + 1e6;  % Large penalty
    end
end
