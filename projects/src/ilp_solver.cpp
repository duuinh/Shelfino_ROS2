#include "ilp_solver.hpp"

std::vector<int> ILP_Solver::find_optimal_path_BnB() {
    std::vector<int> optimal_path;
    std::stack<BnB_Node> q;
    std::vector<double> best_solution;
    double best_solution_value = std::numeric_limits<double>::min();
    // add initial problem to queue
    q.push({b_vec_, c_vec_});
    while (!q.empty()) {
        BnB_Node node = q.top();
        q.pop();
        // Solve linear relaxation
        Simplex simplex(a_mat_, node.b, node.c);
        std::vector<double> solution = simplex.solve();
        if (solution.empty()) {
            std::cout << "Simplex solver couldn't find feasible solution"
                      << std::endl;
            continue;
        }
        double solution_value = compute_value_gained(solution);
        std::cout << "Current value: " << solution_value
                  << ", Best value: " << best_solution_value << std::endl;
        if (solution_value <= best_solution_value) {
            std::cout << "Solution is worse than best solution" << std::endl;
            continue;
        }

        if (is_integer_solution(solution)) {
            std::cout << "Better integer solution found" << std::endl;
            best_solution = solution;
            best_solution_value = solution_value;
        } else {
            for (int i = 0; i < solution.size(); ++i) {
                if (!is_integer(solution[i])) {
                    std::cout << "x [" << i << "] = " << solution[i]
                              << " is not integer" << std::endl;
                    // branching step: new value of x can be 0 or 1
                    BnB_Node new_node = modify_constraint(i, 0, node.b, node.c);
                    q.push(new_node);
                    new_node = modify_constraint(i, 1, node.b, node.c);
                    q.push(new_node);
                    break;
                }
            }
        }
    }
    if (best_solution.empty()) {
        std::cout << "No solution found" << std::endl;
    } else {
        std::cout << "Reward gained: " << compute_value_gained(best_solution)
                  << std::endl;
        optimal_path = reconstruct_path(best_solution);
    }
    return optimal_path;
}

void ILP_Solver::initialize_simplex_tableau() {
    
    // cost function: maximize rewards gained
    // create c vector with size of n^2
    c_vec_.resize(n_nodes_ * n_nodes_);
    for (int i = 0; i < n_nodes_; ++i) {
        for (int j = 0; j < n_nodes_; ++j) {
            if (i != j) {
                // travel from i to j => get reward of node i
                c_vec_[i * n_nodes_ + j] = rewards_[i];
            }
        }
    }

    // Constraints for orienteering problem:
    // A - left hand side
    // b - right hand side

    std::vector<double> a_row(n_nodes_ * n_nodes_);
    int end_idx = n_nodes_ - 1;
    
    // binary optimization problem: x can be 0 (not selected) or 1 (selected): x<=1
    for (int i = 0; i < n_nodes_ * n_nodes_; ++i) {
        a_row.assign(a_row.size(), 0);
        a_row[i] = 1;
        a_mat_.push_back(a_row);
        b_vec_.push_back(1);
    }

    // start node have one outgoing edge; sum(x_0j) = 1
    a_row.assign(a_row.size(), 0);
    for (int i = 1; i < n_nodes_; ++i) {
        a_row[i] = 1;
    }
    a_mat_.push_back(a_row);
    b_vec_.push_back(1);
    
    // end node have one incoming edge; sum(x_iN) = 1
    a_row.assign(a_row.size(), 0);
    for (int i = 0; i < n_nodes_ - 1; ++i) {
        a_row[i * n_nodes_ + end_idx] = 1;
    }
    a_mat_.push_back(a_row);
    b_vec_.push_back(1);
    
    // start node can't be visited: sum(x_0j) = 0
    a_row.assign(a_row.size(), 0);
    for (int i = 0; i < n_nodes_; ++i) {
        a_row[i * n_nodes_] = 1;
    }
    a_mat_.push_back(a_row);
    b_vec_.push_back(0);   

    
    //// end node can't have outgoing edge: sum(x_Nj) = 0
    // a_row.assign (a_row.size (), 0); for (int i = 0; i < n_nodes_; ++i)
    //{
    // a_row[end_idx * n_nodes_ + i] = 1;}
    // a_mat_.push_back (a_row); b_vec_.push_back (0);
    
    //  total distance should not exceed the budget: total_dist <= dist_max
    a_row.assign(a_row.size(), 0);
    for (int i = 0; i < n_nodes_ -1 ; ++i) {
           for (int j = 1; j < n_nodes_; ++j) {
            a_row[i * n_nodes_ + j] = dist_matrix_[i][j];
        }
    }
    a_mat_.push_back(a_row);
    b_vec_.push_back(dist_max_);
    
    // // each node can be visited only once: sum(x_kj) <= 1
    // for (int j = 1; j < n_nodes_; ++j) {
    //     a_row.assign(a_row.size(), 0);
    //     for (int i = 1; i < n_nodes_- 1; ++i) {
    //         a_row[i * n_nodes_ + j] = 1;
    //     }
    //     a_mat_.push_back(a_row);
    //     b_vec_.push_back(1);
    // }
    // // each node can have only one outgoing edge: sum(x_ik) <= 1
    // for (int i = 0; i < n_nodes_-1; ++i) {
    //     a_row.assign(a_row.size(), 0);
    //     for (int j = 1; j < n_nodes_-1; ++j) {
    //         a_row[i * n_nodes_ + j] = 1;
    //     }
    //     a_mat_.push_back(a_row);
    //     b_vec_.push_back(1);
    // }
    
    // path must be connected: sum(x_ik) = sum(x_kj)
    for (int k = 1; k < n_nodes_-1; ++k) {
        a_row.assign(a_row.size(), 0);
        for (int i = 0; i < n_nodes_ -1 ; ++i) {
            a_row[i * n_nodes_ + k] = -1;   // Coeffs for x_ik terms
        }
        for (int j = 1; j < n_nodes_; ++j) {
            a_row[k * n_nodes_ + j] = 2;    // Coeffs for x_kj terms
        }
        a_mat_.push_back(a_row);
        b_vec_.push_back(1);
    }
    
    // avoid subtours: sum(x_ij) <= |S| - 1, where S is subset of nodes 
    std::vector<std::vector<int>> combinations = generate_combinations();
    for (const auto& combination : combinations) {
        a_row.assign(a_row.size(), 0);
        for (const auto& i : combination) {
            for (int j = 1; j < n_nodes_- 1; ++j) {
                if (i != j) {
                    a_row[i * n_nodes_ + j] = 1; 
                }
            }
        }
        a_mat_.push_back(a_row);
        b_vec_.push_back(combination.size()-1);
    }
    
    add_slack_variables_simplex();
}

void ILP_Solver::add_slack_variables_simplex() {
    // add slack vaiables
    int a_size = a_mat_.size();
    for (int i = 0; i < a_size; i++) {
        for (int j = 0; j < a_size; j++) {
            if (i == j) {
                a_mat_[i].push_back(1);
            } else {
                a_mat_[i].push_back(0);
            }
        }
    }
    // add zero vector to c
    std::vector<double> zero_vec(a_mat_[0].size() - c_vec_.size(), 0);
    c_vec_.insert(c_vec_.end(), zero_vec.begin(), zero_vec.end());
    // add big M to the constraints identified as =
    if (use_big_M_) {
        c_vec_[(n_nodes_ * n_nodes_) + (n_nodes_ * n_nodes_)] = M_VALUE;
        // c_vec_[(n_nodes_ * n_nodes_) + (n_nodes_ * n_nodes_) +1] = M_VALUE;
    }
}

void ILP_Solver::combinations_recursive(std::vector<int> &options, int k,
                                        std::vector<int> &combination,
                                        int start,
                                        std::vector<std::vector<int>> &result) {
    if (k == 0) {
        result.push_back(combination);
        return;
    }

    for (int i = start; i <= options.size() - k; ++i) {
        combination.push_back(options[i]);
        combinations_recursive(options, k - 1, combination, i + 1, result);
        combination.pop_back();
    }
}

std::vector<std::vector<int>> ILP_Solver::generate_combinations() {
    // generate all possible node combinations
    // 2 <= |S| <= n - 2
    std::vector<std::vector<int>> combinations;
    // nodes included in combinations
    std::vector<int> nodes(n_nodes_ - 2);
    for (int i = 0; i < n_nodes_-2; ++i) {
        nodes[i] = i+1;
    }
    // generate combinations with sizes from 2 to n - 2
    for (int i = 2; i < n_nodes_-1; ++i) {
        std::vector<std::vector<int>> result;
        std::vector<int> combination;
        combinations_recursive(nodes, i, combination, 0, result);
        combinations.insert(combinations.end(), result.begin(), result.end());
    }
    return combinations;
}

double ILP_Solver::compute_value_gained(
    const std::vector<double> &best_solution) {
    double gain = 0.0;
    for (int i = 0; i < best_solution.size(); ++i) {
        gain += c_vec_[i] * best_solution[i];
    }
    return gain;
}

bool ILP_Solver::is_integer_solution(const std::vector<double> &solution) {
    bool is_integer_solution = true;
    for (int i = 0; i < solution.size(); ++i) {
        if (!is_integer(solution[i])) {
            is_integer_solution = false;
            break;
        }
    }
    return is_integer_solution;
}

void copy_vector(const std::vector<double> &input, std::vector<double> &copy) {
    copy.clear();
    for (size_t i = 0; i < input.size(); i++) {
        copy.push_back(input[i]);
    }
}

BnB_Node ILP_Solver::modify_constraint(int i, int new_value,
                                       const std::vector<double> &b_vec,
                                       const std::vector<double> &c_vec) {
    BnB_Node new_node;
    copy_vector(b_vec, new_node.b);
    copy_vector(c_vec, new_node.c);
    // modify constraint for x_i
    new_node.b[i] = new_value;
    // add big M
    if (use_big_M_) {
        new_node.c[n_nodes_ * n_nodes_ + i] = M_VALUE;
    }
    return new_node;
}

std::vector<int> ILP_Solver::reconstruct_path(
    const std::vector<double> &solution) {
    std::vector<int> optimal_path = {0};
    std::vector<std::vector<double>> solution_matrix =
        reshape(solution, n_nodes_, n_nodes_);
    int current_node = 0;
    int end_idx = n_nodes_ - 1;
    while (current_node != end_idx) {
        for (int j = 0; j < n_nodes_; j++) {
            if (solution_matrix[current_node][j] == 1) {
                optimal_path.push_back(j);
                current_node = j;
                break;
            }
        }
    }
    if (optimal_path.empty()) {
      optimal_path = {0, n_nodes_-1};
    }
    return optimal_path;
}

ILP_Solver::ILP_Solver(const std::vector<double> &rewards,
                       std::vector<std::vector<double>> &dist_matrix,
                       const double &dist_max) {
    dist_max_ = dist_max;
    rewards_ = rewards;
    dist_matrix_ = dist_matrix;
    n_nodes_ = rewards.size();
    initialize_simplex_tableau();
}
