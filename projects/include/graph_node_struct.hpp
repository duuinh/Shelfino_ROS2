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

struct graph_node {
  double x, y;
  double weight = 0.0;
};