#include "simplex.hpp"

Simplex::Simplex(std::vector<std::vector<double>> a_mat, std::vector<double> b_vec, std::vector<double> c_vec) {
        a_mat_ = a_mat;
        b_vec_ = b_vec;
        c_vec_ = c_vec;
}

std::vector<double> Simplex::solve() {
    std::vector<double> solution;
    cB_.resize(a_mat_.size(), 0);   
    basis_.resize(a_mat_.size(), 0); 
    std::vector<double> z(a_mat_[0].size(), 0);     // reduced cost vector
    int pivot_col, pivot_row;
    bool optimal_reached = false;

    // initialize basis
    for (int i = 0; i < c_vec_.size(); ++i) {
        basis_[i] = c_vec_.size() - 1 - i;
    }

    while(true) {
        // calculate reduced cost vector
        for (int j = 0; j < a_mat_[0].size(); ++j) {
            z[j] = 0;
            for (int i = 0; i < a_mat_.size(); ++i) {
                z[j] -= cB_[i] * a_mat_[i][j];
            }
            z[j] += c_vec_[j];  // relative profit
        }

        if (is_optimal(z)) {
            std::cout << "Simplex: Optimal Solution Found" << std::endl;
            optimal_reached = true;
            break;
        }

        pivot_col = find_pivot_column(z);        
        pivot_row = find_pivot_row(pivot_col);

        if (pivot_row == -1) {
            std::cout << "Simplex: Unbounded Solution" << std::endl;
            break;
        }
        
        update_tableau(pivot_row, pivot_col);
    }
    if (optimal_reached) {
        solution = extract_solution();
    }
    return solution;
}

bool Simplex::is_optimal(const std::vector<double>& v) {
    // check if all elements are non-positive (for MAX problem)
    for (double x : v) {
        if (x > 0) {
        return false;
        }
    }
    return true;    
}

int Simplex::find_pivot_column(const std::vector<double>& v) {
    // find the column corresponding to max relative profit
    int max_index = 0;
    double max_value = v[0];
    for (int i = 1; i < v.size(); ++i) {
        if (v[i] > max_value) {
        max_index = i;
        max_value = v[i];
        }
    }
    return max_index;
}

int Simplex::find_pivot_row(const int& col) {
    // find minimum ratio index
    int min_index = -1;
    double min_value = std::numeric_limits<double>::max();
    for (int i = 0; i < a_mat_.size(); ++i) {
        if (b_vec_[i] > 0 && a_mat_[i][col] > 0) {
            double ratio  = b_vec_[i]/a_mat_[i][col];
            if (ratio < min_value) {
                min_index = i;
                min_value = ratio;
            }
        }
    }
    return min_index;
}

void Simplex::update_tableau(const int& pivot_row, const int& pivot_col) { 
    double pivot_element = a_mat_[pivot_row][pivot_col];
    for (int j = 0; j < a_mat_[0].size(); ++j) {
        a_mat_[pivot_row][j] /= pivot_element;  // = 1
    }
    b_vec_[pivot_row] /= pivot_element;

    for (int i = 0; i < a_mat_.size(); ++i) {
        if (i == pivot_row) continue; 
        double coeff = -a_mat_[i][pivot_col];
        for (int j = 0; j < a_mat_[0].size(); ++j) {
            a_mat_[i][j] += a_mat_[pivot_row][j] * coeff;  
        }
        b_vec_[i] += b_vec_[pivot_row] * coeff;
    }
    cB_[pivot_row] = c_vec_[pivot_col];
    basis_[pivot_row] = pivot_col;
}

std::vector<double> Simplex::extract_solution() {
    int n = c_vec_.size()-a_mat_.size();
    std::vector<double> solution(n, 0);
    for (int i = 0; i < basis_.size(); ++i) {
        if (basis_[i] < n) {
            // basic variables' values correspond to the solution
            solution[basis_[i]] = b_vec_[i];
        }
    }
    return solution;
}

