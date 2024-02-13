#include <vector>
#include <limits>
#include <iostream>
#include <cmath>

#define M               -10000                  // big M value (-M for Max. problem)
#define MAX_ITERATE     100

class Simplex {
   private:
    std::vector<std::vector<double>> a_mat_;    // coeff. matrix of constraints
    std::vector<double>b_vec_;                  // column vector on the right hand side of constraints
    std::vector<double> c_vec_;                 // coeff. vector of cost function
    std::vector<double> cB_;                    // coeff. of the basis
    std::vector<int> basis_;
    std::vector<double> z_;                     // reduced cost vector

    std::vector<int> big_M_index;

    bool is_optimal();
    int find_pivot_column();
    int find_pivot_row(const int &pivot_col);
    void update_tableau(const int &pivot_row, const int &pivot_col);
    std::vector<double> extract_solution();
    bool is_big_M_eliminated();

   public:
    std::vector<double> solve();

    Simplex(std::vector<std::vector<double>> a_mat, std::vector<double> b_vec,
            std::vector<double> c_vec);
};