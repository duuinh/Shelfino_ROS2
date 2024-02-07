#pragma once

#include <string> 
#include <vector>

enum obstacle_type {
  CYLINDER,
  BOX
};

struct obstacle {
  double x, y;
  double radius;
  // obstacle_type type;
};

struct edge {
  double x, y;
  double weight;
};