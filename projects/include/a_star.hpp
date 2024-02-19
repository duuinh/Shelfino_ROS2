#pragma once

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>
#include "graph_node_struct.hpp"
#include "common_defines.h"

struct Motion
{
    int dx;
    int dy;
    double cost;
};

struct Point
{
    int x;
    int y;
};

struct ShortestPath
{
    std::vector<graph_node> path;
    double length;
};

class AStarNode {
public:
    int x;
    int y;
    double g; // cost from start node to current node
    int parent_idx;

    AStarNode(int x, int y) : x(x), y(y), g(0), parent_idx(-1) {
    }

    AStarNode(int x, int y, double g, int parent_idx) : x(x), y(y), g(g), parent_idx(parent_idx) {
    }

    double getF_score(const Point& goal) const { // total cost (g + h)
        int dx = abs(x - goal.x);
        int dy = abs(y- goal.y);
        double h = sqrt(dx * dx + dy * dy + 0.0); // heuristic cost from current node to goal node
        return g + h;
    }

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }

    bool operator==(const AStarNode& other) const {
        return x == other.x && y == other.y;
    }

};

class AStar {
private:
    std::vector<std::vector<bool>> obstacle_map;
    int x_width, y_width;
    double x_max = std::numeric_limits<double>::min();
    double x_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::min();
    double y_min = std::numeric_limits<double>::max();
    double resolution = 0.01;
    Motion directions[8] = {{1, 0, 1},
                        {0, 1, 1},
                        {-1, 0, 1},
                        {0, -1, 1},
                        {-1, -1, std::sqrt(2)},
                        {-1, 1, std::sqrt(2)},
                        {1, -1, std::sqrt(2)},
                        {1, 1, std::sqrt(2)}};

public:
    AStar(std::vector<graph_node> borders, std::vector<obstacle> obstacles);
    ShortestPath get_shortest_path(const graph_node& start_point, const graph_node& target_point);

private:
    int get_node_index(AStarNode node);
    int get_xy_scaled(double xy_original, double xy_min);
    double get_xy_original(int xy_scaled, double xy_min);

    // Function to get points along a line using Bresenham's algorithm
    std::vector<Point> get_points_along_line(int x1, int y1, int x2, int y2);
};
