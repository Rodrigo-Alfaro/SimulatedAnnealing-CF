#include <iostream>
#include "sim_ann.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <chrono>
#include <iomanip>
#include <string>
#include <random>

//Parametros para Simulated Annealing
const double T = 100.0;
const double cooling_rate = 0.95; 
const double temp_threshold = 5.0;

//Variables Globales - Parametros experimentos
const std::string instances_dir = "instancias"; 
const std::string output_data = "sim_ann_results.csv";
const int NUM_SIMULATIONS = 1000; 
const bool verbose = false; // true to print detailed output

std::vector<std::string> instance_files = {
        "prob_10_11s.prob",
        "prob_10_1n.prob",
        "prob_10_1o.prob",
        "prob_10_4ch.prob",
        "prob_10_4n.prob",
        "prob_10_7n.prob",
        "prob_20_1o.prob",
        "prob_20_4o.prob",
        "prob_40_11s.prob",
        "prob_40_1n.prob",
        "prob_40_1o.prob"
    };

// Structs
struct SimulationResult {
    double initial_cost;
    double best_cost;
    double cost_difference;
    double execution_time_ms;
};

//Declaracion Funciones
void parse_instance(const std::string& path, GridInstance& grid_instance);
void print_grid_with_path(const GridInstance& grid_instance, const std::vector<std::pair<int, int>>& path);
SimulationResult run_single_simulation(const std::string& instance_path, unsigned seed);
void run_multiple_simulations(const std::string& instance_path, int num_simulations, 
                            std::ofstream& output_file);

int main() {

    const std::string OUTPUT_FILE = output_data;
    std::ofstream output_file(OUTPUT_FILE);
    if (!output_file.is_open()) {
        std::cerr << "file error" << OUTPUT_FILE << std::endl;
        return 1;
    }
    
    output_file << "instance_name,mean_initial_cost,mean_best_cost,mean_cost_difference,mean_execution_time_ms\n";

    for (const auto& filename : instance_files) {
        std::string fullpath = instances_dir + "/" + filename;
        run_multiple_simulations(fullpath, NUM_SIMULATIONS, output_file);
    }
    output_file.close();
    std::cout << "\nAll simulations completed. Results saved to " << OUTPUT_FILE << std::endl;

    return 0;
}

//Implementacion Funciones

// Printear el grid con el camino encontrado con 9 como camino, usada cuando verbose es true
void print_grid_with_path(const GridInstance& grid_instance, const std::vector<std::pair<int, int>>& path) {
    std::vector<std::vector<int>> display_grid = grid_instance.grid;
    
    for (size_t i = 1; i < path.size() - 1; ++i) {
        int x = path[i].first;
        int y = path[i].second;
        if (x >= 0 && x < grid_instance.cols && y >= 0 && y < grid_instance.rows) {
            if (display_grid[y][x] != 2 && display_grid[y][x] != 3) { // Don't overwrite start/end
                display_grid[y][x] = 9; // 9 as the sol
            }
        }
    }
    
    std::cout << "\nGrid with best path (9 = path, 0 = free, 1 = obstacle, 2 = start, 3 = end):" << std::endl;
    for (int y = 0; y < grid_instance.rows; ++y) {
        for (int x = 0; x < grid_instance.cols; ++x) {
            std::cout << display_grid[y][x];
            if (x < grid_instance.cols - 1) std::cout << ",";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

// Parse de las instancias usando coordenadas (x, y) para el grid
void parse_instance(const std::string& path, GridInstance& grid_instance) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "file error" << path << std::endl;
        return;
    }
    
    std::vector<std::vector<int>> grid;
    std::string line;
    int row = 0;
    
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        std::vector<int> grid_row;
        std::stringstream ss(line);
        std::string cell;
        int col = 0;
        
        while (std::getline(ss, cell, ',')) {
            int value = std::stoi(cell);
            grid_row.push_back(value);
            if (value == 2) { // start
                grid_instance.start = {col, row};
            } else if (value == 3) { // end
                grid_instance.end = {col, row};
            }
            col++;
        }
        
        grid.push_back(grid_row);
        row++;
    }
    
    grid_instance.grid = grid;
    grid_instance.rows = grid.size();
    grid_instance.cols = grid.empty() ? 0 : grid[0].size();
    
    file.close();
    
    std::cout << "Parsed grid: " << grid_instance.rows << "x" << grid_instance.cols << std::endl;
    std::cout << "Start: (" << grid_instance.start.first << ", " << grid_instance.start.second << ")" << std::endl;
    std::cout << "End: (" << grid_instance.end.first << ", " << grid_instance.end.second << ")" << std::endl;
}

//Run de una sola simulacion, para no tener problemas con la aleatoriedad, tambien se mide el tiempo de ejecucion
SimulationResult run_single_simulation(const std::string& instance_path, unsigned seed) {
    SimulationResult result;
    
    GridInstance grid_instance;
    parse_instance(instance_path, grid_instance);
    
    if (grid_instance.grid.empty()) {
        std::cerr << "Failed to parse instance file: " << instance_path << std::endl;
        result.initial_cost = -1.0;
        result.best_cost = -1.0;
        result.cost_difference = 0.0;
        result.execution_time_ms = 0.0;
        return result;
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    SimulatedAnnealing sa(T, cooling_rate, temp_threshold, grid_instance);
    sa.set_random_seed(seed); // Set a specific seed for this run, to have randomness
    
    result.initial_cost = sa.evaluate_cost(sa.Current_sol);
    sa.run(verbose); // false to not print details
    result.best_cost = sa.evaluate_cost(sa.Best_sol);
    result.cost_difference = result.initial_cost - result.best_cost;

    auto end_time = std::chrono::high_resolution_clock::now();
    result.execution_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    return result;
}

// Run de multiples simulaciones para una instancia, ocupando la funcion anterior para medir
// lo necesario para los resultados
void run_multiple_simulations(const std::string& instance_path, int num_simulations, 
                            std::ofstream& output_file) {
    double total_initial_cost = 0.0;
    double total_best_cost = 0.0;
    double total_cost_difference = 0.0;
    double total_execution_time = 0.0;
    int successful_runs = 0;
                                
    std::random_device rd; //random numbers gen
    std::mt19937 gen(rd());
    
    std::cout << "Running " << num_simulations << " simulations for " << instance_path << std::endl;
    
    for (int i = 0; i < num_simulations; ++i) {
        unsigned seed = gen();// new seed for each simulation
        
        SimulationResult result = run_single_simulation(instance_path, seed);
        
        if (result.initial_cost >= 0.0) {
            total_initial_cost += result.initial_cost;
            total_best_cost += result.best_cost;
            total_cost_difference += result.cost_difference;
            total_execution_time += result.execution_time_ms;
            successful_runs++;
        }
        if (verbose){
            if ((i + 1) % 10 == 0 || i + 1 == num_simulations) {
            std::cout << "Completed " << (i + 1) << " of " << num_simulations << " simulations" << std::endl;
        }
        }
    }
    
    if (successful_runs > 0) {
        double mean_initial_cost = total_initial_cost / successful_runs;
        double mean_best_cost = total_best_cost / successful_runs;
        double mean_cost_difference = total_cost_difference / successful_runs;
        double mean_execution_time = total_execution_time / successful_runs;
        
        std::string instance_name = instance_path.substr(instance_path.find_last_of("/\\") + 1);
        output_file << instance_name << ","
                   << std::fixed << std::setprecision(4) << mean_initial_cost << ","
                   << std::fixed << std::setprecision(4) << mean_best_cost << ","
                   << std::fixed << std::setprecision(4) << mean_cost_difference << ","
                   << std::fixed << std::setprecision(2) << mean_execution_time << "\n";
        
        std::cout << "Results for " << instance_name << ":" << std::endl;
        std::cout << "  Mean Initial Cost: " << mean_initial_cost << std::endl;
        std::cout << "  Mean Best Cost: " << mean_best_cost << std::endl;
        std::cout << "  Mean Cost Difference: " << mean_cost_difference << std::endl;
        std::cout << "  Mean Execution Time: " << mean_execution_time << " ms" << std::endl;
    } else {
        std::cerr << "error with the simulations" << instance_path << std::endl;
    }
}