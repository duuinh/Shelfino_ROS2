#pragma once

#include <iostream>
#include <cmath>
#include <cstdlib>
#include "math_utils.hpp"
#include "common_struct.hpp"
#include <map>
#include <limits>
#include "homog2d.hpp"

enum DubinsWord {
    LSL,
    RSR,
    LSR,
    RSL,
    RLR,
    LRL,
    UNDEFINED
};

enum MotionPrimitive {
    Straight = 0,
    LeftTurn = 1,
    RightTurn = -1
};

const std::map<DubinsWord, std::vector<MotionPrimitive>> dubins_word_to_motions = {
    {DubinsWord::LSL, {MotionPrimitive::LeftTurn, MotionPrimitive::Straight, MotionPrimitive::LeftTurn}},
    {DubinsWord::RSR, {MotionPrimitive::RightTurn, MotionPrimitive::Straight, MotionPrimitive::RightTurn}},
    {DubinsWord::LSR, {MotionPrimitive::LeftTurn, MotionPrimitive::Straight, MotionPrimitive::RightTurn}},
    {DubinsWord::RSL, {MotionPrimitive::RightTurn, MotionPrimitive::Straight, MotionPrimitive::LeftTurn}},
    {DubinsWord::RLR, {MotionPrimitive::RightTurn, MotionPrimitive::LeftTurn, MotionPrimitive::RightTurn}},
    {DubinsWord::LRL, {MotionPrimitive::LeftTurn, MotionPrimitive::RightTurn, MotionPrimitive::LeftTurn}}
};

struct PrimitiveResult {
    bool is_valid = false;
    double s1 = 0, s2 = 0, s3 = 0;
};

class WayPoint {
public:
    double x;
    double y;
    double orientation;

    WayPoint() : x{0}, y{0}, orientation{0} {}
    WayPoint(double x, double y, double orientation) : x{x}, y{y}, orientation{orientation} {}
    WayPoint(Point position, double orientation) : x{position.x}, y{position.y}, orientation{orientation} {}
};

class DubinsArc {
public:
    WayPoint start;
    double length;
    double curvature;

    DubinsArc() : start{WayPoint()}, length{0}, curvature{0} {};

    DubinsArc(WayPoint startPoint, double arcLength, double curvature) : start{startPoint}, length{arcLength}, curvature{curvature} {}

    WayPoint get_final_point() const {
        double x = this->start.x + this->length * sinc(this->curvature * this->length / 2.0) * cos(this->start.orientation + this->curvature * this->length / 2.0);
        double y = this->start.y + this->length * sinc(this->curvature * this->length / 2.0) * sin(this->start.orientation + this->curvature * this->length / 2.0);
        double orientation = mod2pi(this->start.orientation + this->curvature * this->length);
        return WayPoint(x, y, orientation);
    }
    
    std::vector<WayPoint> get_points() const;
};

class DubinsCurve {
public:
    std::vector<DubinsArc> arcs;

    DubinsCurve() : arcs{DubinsArc(), DubinsArc(), DubinsArc()} {}
    DubinsCurve(DubinsArc arc1, DubinsArc arc2, DubinsArc arc3) : arcs{arc1, arc2, arc3} {}

    double get_length() const {
        double length = 0;
        for (const auto& arc : arcs) {
            length += arc.length;
        }
        return length;
    }

    std::vector<WayPoint> get_points() const;
};

bool check_collision(DubinsCurve curve, std::vector<GraphNode> &borders, std::vector<Obstacle> &obstacles);

PrimitiveResult calculate_primitive_result(DubinsWord curveType, double scaledThetaStart, double scaledThetaEnd, double scaledCurvatureMax);
DubinsCurve find_shortest_curve(WayPoint startPoint, WayPoint endPoint, const double curvature, std::vector<GraphNode> &borders, std::vector<Obstacle> &obstacles);
std::vector<DubinsCurve> solve_multipoints_dubins (std::vector<WayPoint> &points, const double curvature, std::vector<GraphNode> &borders, std::vector<Obstacle> &obstacles);