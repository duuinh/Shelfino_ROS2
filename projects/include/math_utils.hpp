#pragma once 

#include <vector>
#include <cmath>

std::vector<std::vector<double>> reshape(const std::vector<double>& input, int rows, int cols);
std::vector<double> flatten(const std::vector<std::vector<double>>& input);
bool is_integer(const double& value);

double sinc(double t);
double mod2pi(double angle);