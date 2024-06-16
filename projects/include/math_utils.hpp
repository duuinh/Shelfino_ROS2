#pragma once 

#include <vector>
#include <cmath>
#include "common_struct.hpp"

std::vector<std::vector<double>> reshape(const std::vector<double>& input, int rows, int cols);
std::vector<double> flatten(const std::vector<std::vector<double>>& input);
bool is_integer(const double& value);

double sinc(double x);
double mod2pi(double angle);
double modpi(float angle);
double distance(const Point& a, const Point& b);