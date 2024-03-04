#pragma once

enum ObstacleType {
  CYLINDER,
  BOX
};

struct Obstacle {
  double x, y;
  double radius;
  double dx, dy;
  ObstacleType type;
};

struct OP_Node {
  double x, y;
  double reward = 0.0;
};