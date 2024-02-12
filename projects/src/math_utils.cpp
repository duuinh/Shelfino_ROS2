#include "math_utils.hpp"

std::vector<double> flatten(const std::vector<std::vector<double>>& input) {
    std::vector<double> flattened;
    for (const auto& row : input) {
        for (double value : row) {
            flattened.push_back(value);
        }
    }
    return flattened;
}
std::vector<std::vector<double>> reshape(const std::vector<double>& input, int rows, int cols) {
    std::vector<std::vector<double>> matrix(rows, std::vector<double>(cols));
    int index = 0;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            matrix[i][j] = input[index++];
        }
    }
    return matrix;
}

bool is_integer(const double& value) {
    double intpart;
    return std::modf(value, &intpart) == 0.0; // fractpart == 0.0
}