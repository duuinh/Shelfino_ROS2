#pragma once
#include "common_struct.hpp"
#include <cmath>
#include <vector>
#include <algorithm>
#include <random>
#include "math_utils.hpp"
#include "common_defines.hpp"
#include "homog2d.hpp"
#include <unordered_map>
#include <queue>

/*
 * Probabilistic Roadmap (PRM)
 */

struct RoadMap
{
    std::vector<Point> nodes;
    std::vector<std::vector<int>> graph;
};

class PRM
{
public:
    PRM(std::vector<Obstacle> obstacles, std::vector<GraphNode> borders);
    ShortestPath find_shortest_path(const Point &start_point, const Point &target_point);

private:
    RoadMap roadmap;
    std::vector<Obstacle> obstacles_;
    int n_samples = 500;
    int k_neighbors = 100;      // number of neighbor nodes
    double clearance = 0.5;     // offset radius of the robot that should be free of obstacles [m]
    Point generate_new_point(double x_min, double x_max, double y_min, double y_max);
    bool is_in_free_space(Point &new_point, std::vector<GraphNode> borders);
    bool is_obstacle_free(Point &new_point, Point &neighbor);
    bool is_inside_map(Point point, double radius, std::vector<GraphNode> borders);
    void connect_to_neighbors(int current_node_index);
    int get_node_index(Point point);
};

class AStarNode
{
public:
    double x;
    double y;
    double g; // cost from start node to current node
    int parent_idx;
    int idx;

    AStarNode(double x, double y, int idx) : x(x), y(y), g(0), parent_idx(-1), idx(idx)
    {
    }

    AStarNode(double x, double y, int idx, int parent_idx, double g) : x(x), y(y), g(g), parent_idx(parent_idx), idx(idx)
    {
    }

    double getF_score(const Point &goal) const
    { // total cost (g + h)
        double dx = abs(x - goal.x);
        double dy = abs(y - goal.y);
        double h = sqrt(dx * dx + dy * dy + 0.0); // heuristic cost from current node to goal node
        return g + h;
    }

    double get_distance(const Point &other)
    {
        int dx = abs(x - other.x);
        int dy = abs(y - other.y);
        return sqrt(dx * dx + dy * dy + 0.0);
    }

    bool operator==(const Point &other) const
    {
        return x == other.x && y == other.y;
    }

    bool operator==(const AStarNode &other) const
    {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Point &other) const
    {
        return x != other.x && y != other.y;
    }
};
