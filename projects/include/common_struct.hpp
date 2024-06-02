#pragma once
#include <vector>

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

struct GraphNode {
  double x, y;
  double reward = 0.0;
};

struct Point
{
    double x;
    double y;
};

struct ShortestPath
{
    std::vector<GraphNode> path;
    double length;
};
