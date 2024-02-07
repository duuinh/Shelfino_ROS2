#pragma once

#include <string> 
#include <vector>

enum obstacle_type {
  CYLINDER,
  BOX
};

struct obstacle {
  double radius;
  double x, y;
  double dx, dy;
  obstacle_type type;
};

struct edge {
  double x, y;
  double weight;
};