#pragma once 

#include <vector>
#include <iostream>
#include <algorithm>  

struct PathValue {
    std::vector<int> path;
    double total_distance = 0;
    double total_reward = 0;

    PathValue();
    PathValue(std::vector<int> path, double total_distance, double total_reward) : path(path), total_distance(total_distance), total_reward(total_reward) {};
};

std::vector<int> find_optimal_path(double max_distance, std::vector<std::vector<double>> distance_matrix, std::vector<double> rewards);