#pragma once 

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include "rclcpp/rclcpp.hpp"
#include "simplex.hpp"
#include "math_utils.hpp"

class ILP_Solver {
private:
    double dist_max_;                                   // maximum distance
    std::vector<double> rewards_;                       // reward of each node
    std::vector<std::vector<double>> dist_matrix_;      // distance matrix
    int n_nodes_;                                       // number of nodes

    // inputs for simplex algorithm
    std::vector<std::vector<double>> a_mat_;            // coeff. matrix of constraints
    std::vector<double> b_vec_;                         // column vector on the right hand side of constraints
    std::vector<double> c_vec_;                         // coeff. vector of cost function

    bool use_big_M_ = false;                      
    int M_ = -9999;                                     // big M value (-M for Max. problem)

    void initialize_simplex_tableau();
    void add_slack_variables_simplex();                 // to make the initial feasible solution

    void combinations_recursive(std::vector<int>& options, int k, std::vector<int>& combination, int start, std::vector<std::vector<int>>& result);
    std::vector<std::vector<int>> generate_combinations();
    bool is_integer_solution(const std::vector<double>& solution);
    std::vector<int> reconstruct_path(const std::vector<double>& solution);
    double compute_value_gained(const std::vector<double>& solution);
    std::pair<std::vector<double>, std::vector<double>> modify_constraint(int i, int new_value, const std::vector<double>& b_vec, const std::vector<double>& c_vec);

public:
    ILP_Solver(const std::vector<double>& rewards, std::vector<std::vector<double>>& dist_matrix, const double& dist_max);
    std::vector <int> find_optimal_path_BnB();          // use branch and bound method
};
