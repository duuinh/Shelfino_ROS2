#include "orienteering_solver.hpp"

std::vector<int> find_optimal_path(double max_distance, std::vector<std::vector<double>> distance_matrix, std::vector<double> rewards) {
    int n_nodes_ = distance_matrix.size();
    std::vector<int> nodes(n_nodes_ - 2);
    
    for (int i = 0; i < n_nodes_-2; ++i) {
        nodes[i] = i+1;
    }
    std::vector<std::vector<int>> combinations;
    
    do { 
        combinations.push_back(nodes);
    } while ( std::next_permutation(nodes.begin(),nodes.end()) );

    std::vector<int> best_path;
    double best_path_reward = -1;
    double best_path_distance = 0;
    
    for (int i = 0; i < combinations.size(); i++) {
        double total_distance = 0;
        double total_reward = 0;
        int current_node = 0;
        std::vector<int> path = {0};
        for (int j = 0; j < combinations[i].size(); j++) {
            double temp = total_distance + distance_matrix[current_node][combinations[i][j]] + distance_matrix[combinations[i][j]][n_nodes_ -1];
            if (temp > max_distance) {
               break; 
            }
            total_distance += distance_matrix[current_node][combinations[i][j]] ;
            current_node = combinations[i][j];
            path.push_back(current_node);
    
            total_reward += rewards[current_node];
        }
        path.push_back(n_nodes_-1);
        total_distance += distance_matrix[current_node][n_nodes_-1];
        
        if (total_reward >= best_path_reward) {
            if (total_reward == best_path_reward && total_distance < best_path_distance) {
                best_path = path;
                best_path_reward = total_reward;
                best_path_distance = total_distance;
            } else {
                best_path = path;
                best_path_reward = total_reward;
                best_path_distance = total_distance;
            }
        }
    }
    
    std::cout << "best_path_reward: "<< best_path_reward << std::endl;
    std::cout << "best_path_distance: "<< best_path_distance << std::endl;
    return best_path;
}