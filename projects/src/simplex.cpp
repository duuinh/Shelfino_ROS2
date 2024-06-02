#include "simplex.hpp"

Simplex::Simplex(std::vector<std::vector<double>> a_mat,
                 std::vector<double> b_vec, std::vector<double> c_vec) {
    a_mat_ = a_mat;
    b_vec_ = b_vec;
    c_vec_ = c_vec;
}

std::vector<double> Simplex::solve() {
    std::vector<double> solution;
    cB_.resize(a_mat_.size(), 0);
    basis_.resize(a_mat_.size(), 0);
    z_.resize(a_mat_[0].size(), 0);
    int pivot_col, pivot_row;
    bool optimal_reached = false;
    int n_squared = c_vec_.size() - a_mat_.size();
    int counter = 0;

    // initialize basis and cB
    for (int i = 0; i < basis_.size(); ++i) {
        int c_idx = n_squared + i;
        basis_[i] = c_idx;
        cB_[i] = c_vec_[c_idx];
        if (c_vec_[c_idx] ==  M_VALUE) {
            big_M_index.push_back(c_idx);
        }
    }

    z_ = c_vec_;
    for (int j = 0; j < z_.size(); ++j) {
        z_[j] *= -1;
        if (j < n_squared) {
          for (int i = 0; i < cB_.size(); ++i) {
            z_[j] += cB_[i] * a_mat_[i][j];    // panalize objective function
          }
        } else {
          z_[j] = 0;
        }
    }

    std::cout << "Starting simplex solver" << std::endl;
    while (counter < MAX_ITERATE) {
        if (is_optimal()) {
            if (is_big_M_eliminated()) {
                std::cout << "Simplex: Optimal Solution Found" << std::endl;
            } else {
                std::cout << "Simplex: Infeasible Solution" << std::endl;
            }
            optimal_reached = true;
            break;
        }

        pivot_col = find_pivot_column();
        pivot_row = find_pivot_row(pivot_col);
        if (pivot_row == -1) {
            std::cout << "Simplex: Unbounded Solution" << std::endl;
            break;
        }

        update_tableau(pivot_row, pivot_col);
        counter++;
    }
    if (optimal_reached) {
        solution = extract_solution();
    }
    return solution;
}

bool Simplex::is_big_M_eliminated() {
    int n_squared = c_vec_.size() - a_mat_.size();
    for (int i = 0; i < basis_.size(); ++i) {
        if (basis_[i] >= n_squared && b_vec_[i] > 0) {
            // check if there's artificial vector in the optimal basis
            for (int j = 0; j < basis_.size(); ++j) {
                if (basis_[i] == big_M_index[j]) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool Simplex::is_optimal() {
    for (double x : z_) {
        double x_rounded = std::round(x);
        x = std::abs(x - x_rounded) < 0.01 ? x_rounded : x;
        if (x < 0) {
            return false;
        }
    }
    return true;
}

int Simplex::find_pivot_column() {
    // find the column corresponding to min relative profit
    int min_index = -1;
    double min_value = std::numeric_limits<double>::max();
    for (int i = c_vec_.size() - a_mat_.size() -1; i >= 0; --i) {
        if (z_[i] < min_value) {
            min_index = i;
            min_value = z_[i];
        }
    }
    return min_index;
}

int Simplex::find_pivot_row(const int &col) {
    // find minimum ratio index
    int min_index = -1;
    double min_value = std::numeric_limits<double>::max();
    for (int i = a_mat_.size() -1; i >= 0; --i) {
        if (a_mat_[i][col] > 0) {
            double ratio = b_vec_[i] / a_mat_[i][col];
            if (ratio < min_value) {
                min_index = i;
                min_value = ratio;
            }
        }
    }
    return min_index;
}

void Simplex::update_tableau(const int &pivot_row, const int &pivot_col) {
    double pivot_element = a_mat_[pivot_row][pivot_col];
    for (int j = 0; j < a_mat_[0].size(); ++j) {
        a_mat_[pivot_row][j] /= pivot_element;  // = 1
    }
    b_vec_[pivot_row] /= pivot_element;
    for (int i = 0; i < a_mat_.size(); ++i) {
        if (i == pivot_row) continue;
        double m_i = a_mat_[i][pivot_col] / a_mat_[pivot_row][pivot_col];
        for (int j = 0; j < a_mat_[0].size(); ++j) {
            a_mat_[i][j] -= m_i * a_mat_[pivot_row][j];
        }
        b_vec_[i] -= m_i * b_vec_[pivot_row];
    }

    cB_[pivot_row] = c_vec_[pivot_col];
    basis_[pivot_row] = pivot_col;

    // calculate reduced cost vector
    for (int j = 0; j < a_mat_[0].size(); ++j) {
        z_[j] = 0;
        for (int i = 0; i < a_mat_.size(); ++i) {
            z_[j] += cB_[i] * a_mat_[i][j];
        }
        z_[j] -= c_vec_[j];  // -c
    }
}

std::vector<double> Simplex::extract_solution() {
    int n = c_vec_.size() - a_mat_.size();
    std::vector<double> solution(n, 0);
    for (int i = 0; i < basis_.size(); ++i) {
        if (basis_[i] < n) {
            // basic variables' values correspond to the solution
            double x = b_vec_[i];
            double x_rounded = std::round(b_vec_[i]);
            solution[basis_[i]] =
                std::abs(x - x_rounded) < 0.01 ? x_rounded : x;
        }
    }
    std::cout << "Simplex: solution = [ ";
    for (int i = 0; i < solution.size(); ++i) {
        std::cout << solution[i] << ", ";
    }
    std::cout << "]" << std::endl;
    return solution;
}
