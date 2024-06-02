#ifndef DUBINS_H
#define DUBINS_H


#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include "math_utils.hpp"
#include "shapes.hpp"
#include <map>

enum CurveType {
    LeftStraightLeft,
    RightStraightRight,
    LeftStraightRight,
    RightStraightLeft,
    RightLeftRight,
    LeftRightLeft,
    Undefined = -1
};

enum ArcDirection {
    Straight = 0,
    LeftTurn = 1,
    RightTurn = -1
};

class WayPoint {
public:
    double x;
    double y;
    double orientation;

    WayPoint() : x{0}, y{0}, orientation{0} {}
    WayPoint(double x, double y, double orientation) : x{x}, y{y}, orientation{orientation} {}
    WayPoint(Point2D position, double orientation) : x{position.x}, y{position.y}, orientation{orientation} {}

    Point2D getPosition() const { return Point2D(x, y); }

    friend bool operator==(const WayPoint& lhs, const WayPoint& rhs) {
        return lhs.x == rhs.x && lhs.y == rhs.y && lhs.orientation <= rhs.orientation * 1.03 && lhs.orientation >= rhs.orientation * 0.97;
    }
};
std::string operator+(std::string s, const WayPoint& point);

class DubinsArc {
public:
    WayPoint startPoint;
    double arcLength;
    double curvature;

    DubinsArc() : startPoint{WayPoint()}, arcLength{0}, curvature{0} {};
    DubinsArc(WayPoint startPoint, double arcLength, double curvature) : startPoint{startPoint}, arcLength{arcLength}, curvature{curvature} {}

    WayPoint getDestination() const;
    std::vector<WayPoint> toPoints(int pointCount) const;
    std::vector<WayPoint> toPointsUniform(double spacing) const;
};
std::string operator+(std::string s, const DubinsArc& arc);

class DubinsCurve {
public:
    std::vector<DubinsArc> arcs;

    DubinsCurve() : arcs{DubinsArc(), DubinsArc(), DubinsArc()} {}
    DubinsCurve(DubinsArc arc1, DubinsArc arc2, DubinsArc arc3) : arcs{arc1, arc2, arc3} {}

    double get_length() const {
        double totalLength = 0;
        for (const auto& arc : arcs) totalLength += arc.arcLength;
        return totalLength;
    }

    std::vector<WayPoint> toPoints(int pointCount) const;
    /**
     * Converts the Dubins path to a vector of uniformly spaced points.
     *
     * @param spacing The spacing between consecutive points.
     * @return A vector of WayPoint objects representing the uniformly spaced points.
     */
    std::vector<WayPoint> toPointsUniform(double spacing) const;
};
std::string operator+(std::string s, const DubinsCurve& curve);


struct PrimitiveResult {
    bool isValid;
    double firstArcLength, secondArcLength, thirdArcLength;
};

DubinsCurve calculateShortestPath(WayPoint start, WayPoint end, double curvature);
bool checkIntersection(DubinsCurve path, Polygon polygon);
bool checkIntersection(DubinsCurve path, Segment2D segment);
bool checkIntersection(DubinsCurve path, Polygon polygon, int precision);
bool checkIntersectionWithSides(DubinsCurve path, Polygon polygon);

PrimitiveResult calc_primitive_result(CurveType curveType, double scaledThetaStart, double scaledThetaEnd, double scaledCurvatureMax);
DubinsCurve find_shortest_curve(WayPoint startPoint, WayPoint endPoint, const double curvature);
bool checkDubinsArcIntersection(DubinsArc arc, Segment2D segment);

std::vector<DubinsCurve> solve_multipoints_dubins (std::vector<WayPoint> &points, const double curvature);
#endif