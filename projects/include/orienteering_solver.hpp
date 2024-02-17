#pragma once 

#include <vector>
#include <iostream>
#include <algorithm>  

std::vector<int> find_optimal_path(double max_distance, std::vector<std::vector<double>> distance_matrix, std::vector<double> rewards);