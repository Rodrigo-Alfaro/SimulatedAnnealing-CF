#include "sim_ann.h"
#include <cstdlib>
#include <ctime>
#include <iostream>

//Constructor con los parametros basicos para SA
SimulatedAnnealing::SimulatedAnnealing(double T, double cooling_rate, double temp_threshold, 
                                       const GridInstance& grid_inst) {
    this->T = T;
    this->cooling_rate = cooling_rate;
    this->temp_threshold = temp_threshold;
    this->grid_instance = grid_inst;
    this->random_seed = std::time(0);
    this->Current_sol = generate_initial_path();
    this->Best_sol = this->Current_sol;
    
    std::srand(this->random_seed); //seed for randomness
}

void SimulatedAnnealing::set_random_seed(unsigned seed) {
    this->random_seed = seed;
    std::srand(seed);
}

//Funcion de evaluacion del costo de la solucion, 
// en este caso la distancia euclidiana entre los puntos del camino
// con penalizacion por obstaculos
double SimulatedAnnealing::evaluate_cost(const std::vector<std::pair<int, int>>& path) {
    double cost = 0.0;
    
    if (!is_valid_path(path)) { //penalize invalid paths
        return std::numeric_limits<double>::max();
    }
    
    for (size_t i = 0; i < path.size() - 1; ++i) {
        double dx = path[i+1].first - path[i].first;
        double dy = path[i+1].second - path[i].second;
        cost += std::sqrt(dx * dx + dy * dy);
    }
    return cost;
}

//Movimiento (generacion de vecino) aleatorio,
// se elige un punto aleatorio del camino y se le aplica una perturbacion aleatoria
// respetando las restricciones del grid y manteniendo la continuidad del camino
std::vector<std::pair<int, int>> SimulatedAnnealing::generate_neighbor(const std::vector<std::pair<int, int>>& path) {
    std::vector<std::pair<int, int>> neighbor = path;
    
    if (path.size() <= 2) return neighbor; // Can't modify start/end only paths
    
        int i = rand() % (path.size() - 2) + 1; // start and end static
        
        std::pair<int, int> prev = path[i-1];
        std::pair<int, int> next = path[i+1];
        
        for (int attempts = 0; attempts < 20; ++attempts) {
            int dx = (rand() % 3) - 1; // -1, 0, 1
            int dy = (rand() % 3) - 1;
            
            std::pair<int, int> new_pos = {path[i].first + dx, path[i].second + dy};
            bool valid_connection = false;
            
            int dist_to_prev = std::max(std::abs(new_pos.first - prev.first), 
                                      std::abs(new_pos.second - prev.second));
            int dist_to_next = std::max(std::abs(new_pos.first - next.first), 
                                      std::abs(new_pos.second - next.second));
            
            valid_connection = (dist_to_prev <= 1 && dist_to_next <= 1);
            if (is_valid_position(new_pos) && valid_connection) {
                neighbor[i] = new_pos;
                break;
            }
        }
    return neighbor;
}

//Ver si la posicion esta dentro del grid y no chocando con un obstaculo
// Retorna true si es una posicion valida, false si es un obstaculo o fuera
bool SimulatedAnnealing::is_valid_position(const std::pair<int, int>& pos) {
    int x = pos.first;
    int y = pos.second;
    
    if (x < 0 || x >= grid_instance.cols || y < 0 || y >= grid_instance.rows) {
        return false;
    }
    
    return grid_instance.grid[y][x] != 1; // 1 = obstacle
}

//Verifica si el camino es valido, curva suave, todos los puntos conectados
// y que el camino comienza en el punto de inicio y termina en el punto final
bool SimulatedAnnealing::is_valid_path(const std::vector<std::pair<int, int>>& path) {
    if (path.empty()) return false;
    
    for (const auto& pos : path) {
        if (!is_valid_position(pos)) {
            return false;
        }
    }
    for (size_t i = 0; i < path.size() - 1; ++i) {
        int dx = std::abs(path[i+1].first - path[i].first);
        int dy = std::abs(path[i+1].second - path[i].second);
        if (dx > 1 || dy > 1) {
            return false;
        }
    }
    if (path.front().first != grid_instance.start.first || 
        path.front().second != grid_instance.start.second ||
        path.back().first != grid_instance.end.first || 
        path.back().second != grid_instance.end.second) {
        return false;
    }
    return true;
}

//Solucion inicial generada con DFS, si no se encuentra un camino valido,
// se intenta con BFS, y si no se encuentra, se crea un camino directo
std::vector<std::pair<int, int>> SimulatedAnnealing::generate_initial_path() {
    std::vector<std::pair<int, int>> path;
    
    std::vector<std::vector<bool>> visited(grid_instance.rows, std::vector<bool>(grid_instance.cols, false));
    std::vector<std::vector<std::pair<int, int>>> parent(grid_instance.rows, 
                                                       std::vector<std::pair<int, int>>(grid_instance.cols, 
                                                                                     {-1, -1}));
    std::vector<std::pair<int, int>> stack;
    
    stack.push_back(grid_instance.start);
    visited[grid_instance.start.second][grid_instance.start.first] = true;
    
    bool found = false;
    
    std::vector<std::pair<int, int>> moves = {// cardinal and intercardinal 
        {0, -1}, {1, 0}, {0, 1}, {-1, 0}, {1, -1}, {1, 1}, {-1, 1}, {-1, -1}
    };
    
    while (!stack.empty() && !found) {
        std::pair<int, int> current = stack.back();
        stack.pop_back();
        
        if (current.first == grid_instance.end.first && current.second == grid_instance.end.second) {
            found = true;
            break;
        }
        for (const auto& move : moves) {
            int nx = current.first + move.first;
            int ny = current.second + move.second;
            std::pair<int, int> next_pos = {nx, ny};
            
            if (is_valid_position(next_pos) && !visited[ny][nx]) {
                visited[ny][nx] = true;
                parent[ny][nx] = current;
                stack.push_back(next_pos);
            }
        }
    }
    
    if (found) { // reconstruction
        std::pair<int, int> current = grid_instance.end;
        std::vector<std::pair<int, int>> reverse_path;
        
        while (!(current.first == grid_instance.start.first && current.second == grid_instance.start.second)) {
            reverse_path.push_back(current);
            current = parent[current.second][current.first];
        }
        
        reverse_path.push_back(grid_instance.start);
        for (int i = reverse_path.size() - 1; i >= 0; --i) {
            path.push_back(reverse_path[i]);
        }
    } else {// BFS
        for (auto& row : visited) {
            std::fill(row.begin(), row.end(), false);
        }
        std::vector<std::pair<int, int>> queue;
        queue.push_back(grid_instance.start);
        visited[grid_instance.start.second][grid_instance.start.first] = true;
        
        while (!queue.empty() && !found) {
            std::pair<int, int> current = queue.front();
            queue.erase(queue.begin());
            
            for (const auto& move : moves) {
                int nx = current.first + move.first;
                int ny = current.second + move.second;
                std::pair<int, int> next_pos = {nx, ny};
                
                if (is_valid_position(next_pos) && !visited[ny][nx]) {
                    visited[ny][nx] = true;
                    parent[ny][nx] = current;
                    queue.push_back(next_pos);
                    
                    if (nx == grid_instance.end.first && ny == grid_instance.end.second) {
                        found = true;
                        break;
                    }
                }
            }
        }
        if (found) { // BFS reconstruction
            std::pair<int, int> current = grid_instance.end;
            std::vector<std::pair<int, int>> reverse_path;
            
            while (!(current.first == grid_instance.start.first && current.second == grid_instance.start.second)) {
                reverse_path.push_back(current);
                current = parent[current.second][current.first];
            }
            
            reverse_path.push_back(grid_instance.start);
            
            for (int i = reverse_path.size() - 1; i >= 0; --i) {
                path.push_back(reverse_path[i]);
            }
        }
    }
    std::cout << "Initial sol with " << path.size() << " points" << std::endl;
    return path;
}

//Run del algoritmo SA con restricciones de camino valido 
void SimulatedAnnealing::run(bool print_progress) {
    double current_cost = evaluate_cost(Current_sol);
    double best_cost = evaluate_cost(Best_sol);
    int iterations = 0;// just as a meassure, the stopping condition is the temp_threshold
    
    if (!is_valid_path(Current_sol)) {
        if (print_progress) {
            std::cout << "Initial sol not valid" << std::endl;
        }
        Current_sol = generate_initial_path();
        Best_sol = Current_sol;
        current_cost = evaluate_cost(Current_sol);
        best_cost = current_cost;
    }

    while (T > temp_threshold){ //end when temperature is low enough
        iterations++;
        std::vector<std::pair<int, int>> neighbor = generate_neighbor(Current_sol);
        if (!is_valid_path(neighbor)) { // valid neighbor
            continue;
        }
        
        double neighbor_cost = evaluate_cost(neighbor);
        double delta = neighbor_cost - current_cost; // minimize
        bool accepted = false;

        if (delta < 0 || (double) rand() / RAND_MAX < std::exp(-delta/T)) { //better sol or SA method
            Current_sol = neighbor;
            current_cost = neighbor_cost;
            T = delta < 0 ? T : T * cooling_rate; //only cool if we accepted a worse solution
            accepted = true;
        } 
        if (current_cost < best_cost) { //update if the sol is better - AM
            Best_sol = Current_sol;
            best_cost = current_cost;
            
            if (print_progress && (iterations % 100 == 0 || best_cost < current_cost)) { //verbose
                std::cout << "Iteration " << iterations 
                          << ", Path Length: " << Best_sol.size() 
                          << ", Best cost: " << best_cost 
                          << ", Temperature: " << T << std::endl;
            }
        }
    }
    
    if (print_progress) {
        std::cout << "\nSimulated Annealing completed with " << iterations << " iterations." << std::endl;
        std::cout << "Final path length: " << Best_sol.size() << " points" << std::endl;
        std::cout << "Final temperature: " << T << std::endl;
        
        if (!is_valid_path(Best_sol)) {
            std::cout << "Final sol not valid" << std::endl;
        } 
    }
}


