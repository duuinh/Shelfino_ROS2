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

double sinc(double x) {
    if (abs(x) < 0.002) {
        return 1-(x*x)*(1/6-(x*x)/120);
    } 
    return sin(x)/x;
}

double mod2pi(double angle)
{
    double output = angle;
    while (output < 0) {
        output += 2 * M_PI;
    }
    while (output >= 2 * M_PI) { 
        output -= 2 * M_PI;
    }
    return output;
}

double modpi(float angle)
{
    double output = angle;
    while (output <= -M_PI) {
        output += 2 * M_PI;
    }
    while (output > M_PI) { 
        output -= 2 * M_PI;
    }
    return output;
}

double distance(const Point& a, const Point& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}