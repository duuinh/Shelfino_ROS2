#ifndef DUBINS_H
#define DUBINS_H


#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstdlib>
//#include <utils/utils.h>
#include <shapes/shapes.hpp>
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

class PathPoint {
public:
    double xPos;
    double yPos;
    double orientation;

    PathPoint() : xPos{0}, yPos{0}, orientation{0} {}
    PathPoint(double xPos, double yPos, double orientation) : xPos{xPos}, yPos{yPos}, orientation{orientation} {}
    PathPoint(Point2D position, double orientation) : xPos{position.x}, yPos{position.y}, orientation{orientation} {}

    Point2D getPosition() const { return Point2D(xPos, yPos); }

    friend bool operator==(const PathPoint& lhs, const PathPoint& rhs) {
        return lhs.xPos == rhs.xPos && lhs.yPos == rhs.yPos && lhs.orientation <= rhs.orientation * 1.03 && lhs.orientation >= rhs.orientation * 0.97;
    }
};
std::string operator+(std::string s, const PathPoint& point);

class PathArc {
public:
    PathPoint startPoint;
    double arcLength;
    double curvature;

    PathArc() : startPoint{PathPoint()}, arcLength{0}, curvature{0} {};
    PathArc(PathPoint startPoint, double arcLength, double curvature) : startPoint{startPoint}, arcLength{arcLength}, curvature{curvature} {}

    PathPoint getDestination() const;
    std::vector<PathPoint> toPoints(int pointCount) const;
    std::vector<PathPoint> toPointsUniform(double spacing) const;
};
std::string operator+(std::string s, const PathArc& arc);

class PathCurve {
public:
    std::vector<PathArc> arcs;

    PathCurve() : arcs{PathArc(), PathArc(), PathArc()} {}
    PathCurve(PathArc arc1, PathArc arc2, PathArc arc3) : arcs{arc1, arc2, arc3} {}

    double getLength() const {
        double totalLength = 0;
        for (const auto& arc : arcs) totalLength += arc.arcLength;
        return totalLength;
    }

    std::vector<PathPoint> toPoints(int pointCount) const;
    /**
     * Converts the Dubins path to a vector of uniformly spaced points.
     *
     * @param spacing The spacing between consecutive points.
     * @return A vector of PathPoint objects representing the uniformly spaced points.
     */
    std::vector<PathPoint> toPointsUniform(double spacing) const;
};
std::string operator+(std::string s, const PathCurve& curve);

class PathConnection {
private:
    PathPoint sourcePoint, destinationPoint;
    PathCurve path;
    bool isEmpty;
public:
    PathConnection() : isEmpty{true} {}
    PathConnection(PathPoint sourcePoint, PathPoint destinationPoint, PathCurve path) : sourcePoint{sourcePoint}, destinationPoint{destinationPoint}, path{path}, isEmpty{false} {}

    PathPoint getSource() const { return sourcePoint; }
    PathPoint getDestination() const { return destinationPoint; }
    PathCurve getPath() const { return path; }
    bool isEmptyConnection() const { return isEmpty; }
    std::string toJson(double precision) const;
};

struct PathPrimitive {
    bool isValid;
    double firstArcLength, secondArcLength, thirdArcLength;
};

PathCurve calculateShortestPath(PathPoint start, PathPoint end, double curvature);
std::vector<PathCurve> generatePathCurves(PathPoint start, PathPoint end, double curvature);
bool checkIntersection(PathCurve path, Polygon polygon);
bool checkIntersection(PathCurve path, Segment2D segment);
bool checkIntersection(PathCurve path, Polygon polygon, int precision);
bool checkIntersectionWithSides(PathCurve path, Polygon polygon);

double sinc(double t){
    if(t==0){
        return 1;
    }
    return abs(t)<0.002 ? 1-(t*t)/6*(1-(t*t)/20) : sin(t)/t;
}

double mod2pi(double angle)
{
    double out = angle;
    while(out < 0) out+= 2*M_PI;
    while(out >= 2*M_PI) out-= 2*M_PI;
    return out;
}

double modpi(double angle)
{
    double out = angle;
    while(out < -M_PI) out+= 2*M_PI;
    while(out >= M_PI) out-= 2*M_PI;
    return out;
}

#endif