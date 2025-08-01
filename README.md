# GeneticAlgorithm_PID_Controller
This repository contains the genetic algorithm used for optimizing PID Controllers and represents our ECE 470 Final Project  

## Authors
Logan Dring (V00984802), Kevin Warkentin (V01004708), Jonathan Boyce (V00967622)

## Requirements
MATLAB R2024b or later  
Control Toolbox

## Running the PID Parameter Tuner
The following is the steps to run and test the genetic algorithm:  
1. Create a MATLAB directory containing the following scripts: *run_pid_ga.m*, *GA_mutation.m*, *crossover.m*, *pid_fitness.m*, and *roulette_wheel.m*.
2. Set desired transfer function by modifying the values of variable "num" and "den" in run_pid_ga.
3. Set desired system performance metrics by modifying the values within the 1 x 4 real row vector desired_specs in run_pid_ga with values corresponding to the [rise, settling, overshoot, steady_state_error]
4. Set desired fitness function weights, W<sub>1</sub>, W<sub>2</sub>, W<sub>3</sub>, and W<sub>4</sub>, by modifying the values within the 1 x 4 real row vector weights in run_pid_ga to priotize certain performance metrics. The weights correspond to the following fitness function:  
  Fitness = W<sub>1</sub> × E<sub>rise time</sub> + W<sub>2</sub> × E<sub>settling time</sub> + W<sub>3</sub> × E<sub>overshoot</sub> + W<sub>4</sub> × E<sub>steady-state</sub>
5. Run the script *run_pid_fitness.m*
  
