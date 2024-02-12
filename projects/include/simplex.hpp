#include <vector>
#include <limits>
#include <iostream>

class Simplex {
private:
    std::vector<std::vector<double>> a_mat_;    // coeff. matrix of constraints
    std::vector<double> b_vec_;                 // column vector on the right hand side of constraints
    std::vector<double> c_vec_;                 // coeff. vector of cost function

    std::vector<double> cB_;                     // coeff. of the basis
    std::vector<int> basis_;

    bool is_optimal(const std::vector<double>& vector);
    int find_pivot_column(const std::vector<double>& v);
    int find_pivot_row(const int& pivot_col);
    void update_tableau(const int& pivot_row, const int& pivot_col);
    std::vector<double> extract_solution();

public:
    std::vector<double> solve();

    Simplex(std::vector<std::vector<double>> a_mat, std::vector<double> b_vec, std::vector<double> c_vec);
};