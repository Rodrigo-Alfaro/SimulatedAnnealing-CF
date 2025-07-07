#ifndef SIM_ANN_H
#define SIM_ANN_H

#include <vector>
#include <utility>
#include <cmath>
#include <random>
#include <limits>

struct GridInstance {
    std::vector<std::vector<int>> grid;
    std::pair<int, int> start;
    std::pair<int, int> end;
    int rows;
    int cols;
};

class SimulatedAnnealing {
public: // all public for easy access

    //Parametros
    double T;
    double cooling_rate;
    double temp_threshold; // stopping condition
    std::vector<std::pair<int, int>> Best_sol;
    std::vector<std::pair<int, int>> Current_sol;
    GridInstance grid_instance; 
    unsigned random_seed;

    //Constructor
    SimulatedAnnealing(double T, double cooling_rate, double temp_threshold, 
                       const GridInstance& grid_inst);

    //Funciones
    bool is_valid_position(const std::pair<int, int>& pos);
    bool is_valid_path(const std::vector<std::pair<int, int>>& path);
    std::vector<std::pair<int, int>> generate_initial_path();
    double evaluate_cost(const std::vector<std::pair<int, int>>& path);
    std::vector<std::pair<int, int>> generate_neighbor(const std::vector<std::pair<int, int>>& path);
    void run(bool print_progress = true);
    void set_random_seed(unsigned seed);
};

#endif // SIM_ANN_H
