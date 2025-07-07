# Simulated Annealing for Collision-Free Path Planning 

*This was a project for Artificial Intelligence INF-295*

## File Structure
- [main](https://github.com/Rodrigo-Alfaro/SimulatedAnnealing-FC/blob/main/main.cpp): Where everything related to the execution of the simulations is. Check the headers for the parameter definitions related to the algorithm parameters and simulation parameters
- [simm_ann](https://github.com/Rodrigo-Alfaro/SimulatedAnnealing-FC/blob/main/sim_ann.cpp): Where everything related to the implementation of the algorithm is

## How to run
Navigate to the folder with the project
```
    make
```
And then to run it:
```
    make run
```
This will execute the simulations and save them in a .csv file. The default name for the file is: sim_ann_results.csv

To clear the output files:
```
    make clean
```

The output CSV has these headers:
- **instance_name**: Name of the test instance or problem scenario
- **mean_initial_cost**: Average cost of the initial solution before optimization
- **mean_best_cost**: Average cost of the best solution found by the algorithm
- **mean_cost_difference**: Average difference between initial and best costs (improvement)
- **mean_execution_time_ms**: Average execution time in milliseconds
