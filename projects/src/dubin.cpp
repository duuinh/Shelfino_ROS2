#include "dubins.hpp"
using namespace std;

// Returns a vector containing the approximation of a PathCurve using n points.
// If a straight arc is found inside the curve, its points will compose 10% of the total points.
// Points that compose a curve arc will be proportional to the length of the arc inside the curve.
std::vector<PathPoint> PathCurve::toPoints(int n) {
    std::vector<PathPoint> points;
    const double straightCoeff = 0.1;
    double curvesLength = getLength();
    for (auto& arc : arcs) {
        if (arc.curvature == 0) {
            n -= static_cast<int>(straightCoeff * n);
            curvesLength -= arc.arcLength;
        }
    }
    for (auto& arc : arcs) {
        double coeff = arc.curvature == 0 ? straightCoeff : (arc.arcLength / curvesLength);
        int arcPointsCount = round(coeff * n);
        auto arcPoints = arc.toPoints(arcPointsCount);
        points.insert(points.end(), arcPoints.begin(), arcPoints.end());
    }

    return points;
}

std::vector<PathPoint> PathCurve::toPointsUniform(double spacing) {
    std::vector<PathPoint> points;
    if (arcs.empty()) {
        return points;
    }
    points.push_back(arcs[0].startPoint);
    for (auto& arc : arcs) {
        auto arcPoints = arc.toPointsUniform(spacing);
        if (!arcPoints.empty()) {
            points.insert(points.end(), arcPoints.begin() + 1, arcPoints.end());
        }
    }
    return points;
}

std::vector<PathPoint> PathArc::toPointsUniform(double spacing) {
    int pointsCount = round(this->arcLength / spacing) + 2;
    return toPoints(pointsCount);
}

// Returns a vector of n points that approximates the arc.
std::vector<PathPoint> PathArc::toPoints(int n) {
    std::vector<PathPoint> points;
    if (n < 2) {
        return points;
    }
    double unitSegmentLength = this->arcLength / (n - 1);
    for (int i = 0; i < n; i++) {
        double arcLengthPortion = unitSegmentLength * i;
        PathArc smallerArc(this->startPoint, arcLengthPortion, this->curvature);
        PathPoint point = smallerArc.getDestination();
        points.push_back(point);
    }
    return points;
}

PathPoint PathArc::getDestination() {
    double x = this->startPoint.xPos + this->arcLength * sinc(this->curvature * this->arcLength / 2.0) * cos(this->startPoint.orientation + this->curvature * this->arcLength / 2);
    double y = this->startPoint.yPos + this->arcLength * sinc(this->curvature * this->arcLength / 2.0) * sin(this->startPoint.orientation + this->curvature * this->arcLength / 2);
    double orientation = mod2pi(this->startPoint.orientation + this->curvature * this->arcLength);
    return PathPoint(x, y, orientation);
}

Eigen::Vector4d scaleToStandard(PathPoint startPoint, PathPoint endPoint, const double curvature) {
    double dx = endPoint.xPos - startPoint.xPos;
    double dy = endPoint.yPos - startPoint.yPos;
    double phi = atan2(dy, dx);
    double lambda = hypot(dx, dy) / 2;

    double scaledThetaStart = mod2pi(startPoint.orientation - phi);
    double scaledThetaEnd = mod2pi(endPoint.orientation - phi);
    double scaledCurvatureMax = curvature * lambda;
    return Eigen::Vector4d(scaledThetaStart, scaledThetaEnd, scaledCurvatureMax, lambda);
}

Eigen::Vector3d scaleFromStandard(double lambda, double scaledS1, double scaledS2, double scaledS3) {
    return Eigen::Vector3d(scaledS1 * lambda, scaledS2 * lambda, scaledS3 * lambda);
}

PathPrimitive calculateLSL(double scaledThetaStart, double scaledThetaEnd, double scaledCurvatureMax) {
    double const inverseK = 1 / scaledCurvatureMax;
    PathPrimitive result;

    double c = cos(scaledThetaEnd) - cos(scaledThetaStart);
    double s = 2 * scaledCurvatureMax + sin(scaledThetaStart) - sin(scaledThetaEnd);
    double temp1 = atan2(c, s);
    result.firstArcLength = inverseK * mod2pi(temp1 - scaledThetaStart);
    double temp2 = 2 + 4 * (scaledCurvatureMax * scaledCurvatureMax) - 2 * cos(scaledThetaStart - scaledThetaEnd) + 4 * scaledCurvatureMax * (sin(scaledThetaStart) - sin(scaledThetaEnd));
    if (temp2 < 0) {
        result.isValid = false;
        result.firstArcLength = 0;
        result.secondArcLength = 0;
        result.thirdArcLength = 0;
    } else {
        result.isValid = true;
        result.secondArcLength = inverseK * sqrt(temp2);
        result.thirdArcLength = inverseK * mod2pi(scaledThetaEnd - temp1);
    }
    return result;
}

PathPrimitive calculateRSR(double scaledThetaStart, double scaledThetaEnd, double scaledCurvatureMax) {
    double const inverseK = 1 / scaledCurvatureMax;
    PathPrimitive result;

    double c = cos(scaledThetaStart) - cos(scaledThetaEnd);
    double s = 2 * scaledCurvatureMax - sin(scaledThetaStart) + sin(scaledThetaEnd);
    double temp1 = atan2(c, s);
    result.firstArcLength = inverseK * mod2pi(scaledThetaStart - temp1);
    double temp2 = 2 + 4 * (scaledCurvatureMax * scaledCurvatureMax) - 2 * cos(scaledThetaStart - scaledThetaEnd) - 4 * scaledCurvatureMax * (sin(scaledThetaStart) - sin(scaledThetaEnd));
    if (temp2 < 0) {
        result.isValid = false;
        result.firstArcLength = 0;
        result.secondArcLength = 0;
        result.thirdArcLength = 0;
    } else {
        result.secondArcLength = inverseK * sqrt(temp2);
        result.thirdArcLength = inverseK * mod2pi(temp1 - scaledThetaEnd);
        result.isValid = true;
    }
    return result;
}

PathPrimitive calculateLSR(double scaledThetaStart, double scaledThetaEnd, double scaledCurvatureMax) {
    double const inverseK = 1 / scaledCurvatureMax;
    PathPrimitive result;

    double c = cos(scaledThetaStart) + cos(scaledThetaEnd);
    double s = 2 * scaledCurvatureMax + sin(scaledThetaStart) + sin(scaledThetaEnd);
    double temp1 = atan2(-c, s);
    double temp3 = 4 * (scaledCurvatureMax * scaledCurvatureMax) - 2 + 2 * cos(scaledThetaStart - scaledThetaEnd) + 4 * scaledCurvatureMax * (sin(scaledThetaStart) + sin(scaledThetaEnd));
    if (temp3 < 0) {
        result.isValid = false;
    } else {
        result.secondArcLength = inverseK * sqrt(temp3);
        double temp2 = -atan2(-2, result.secondArcLength * scaledCurvatureMax);
        result.firstArcLength = inverseK * mod2pi(temp1 + temp2 - scaledThetaStart);
        result.thirdArcLength = inverseK * mod2pi(temp1 + temp2 - scaledThetaEnd);
        result.isValid = true;
    }
    return result;
}

PathPrimitive calculateRSL(double scaledThetaStart, double scaledThetaEnd, double scaledCurvatureMax) {
    double const inverseK = 1 / scaledCurvatureMax;
    PathPrimitive result;

    double c = cos(scaledThetaStart) + cos(scaledThetaEnd);
    double s = 2 * scaledCurvatureMax - sin(scaledThetaStart) - sin(scaledThetaEnd);
    double temp1 = atan2(c, s);
    double temp3 = 4 * (scaledCurvatureMax * scaledCurvatureMax) - 2 + 2 * cos(scaledThetaStart - scaledThetaEnd) - 4 * scaledCurvatureMax * (sin(scaledThetaStart) + sin(scaledThetaEnd));
    if (temp3 < 0) {
        result.isValid = false;
    } else {
        result.secondArcLength = inverseK * sqrt(temp3);
        double temp2 = atan2(2, result.secondArcLength * scaledCurvatureMax);
        result.firstArcLength = inverseK * mod2pi(scaledThetaStart - temp1 + temp2);
        result.thirdArcLength = inverseK * mod2pi(scaledThetaEnd - temp1 + temp2);
        result.isValid = true;
    }
    return result;
}

PathPrimitive calculateRLR(double scaledThetaStart, double scaledThetaEnd, double scaledCurvatureMax) {
    double const inverseK = 1 / scaledCurvatureMax;
    PathPrimitive result;

    double c = cos(scaledThetaStart) - cos(scaledThetaEnd);
    double s = 2 * scaledCurvatureMax - sin(scaledThetaStart) + sin(scaledThetaEnd);
    double temp1 = atan2(c, s);
    double temp2 = 0.125 * (6 - 4 * (scaledCurvatureMax * scaledCurvatureMax) + 2 * cos(scaledThetaStart - scaledThetaEnd) + 4 * scaledCurvatureMax * (sin(scaledThetaStart) - sin(scaledThetaEnd)));
    if (abs(temp2) > 1) {
        result.isValid = false;
    } else {
        result.secondArcLength = inverseK * mod2pi(2 * M_PI - acos(temp2));
        result.firstArcLength = inverseK * mod2pi(scaledThetaStart - temp1 + 0.5 * result.secondArcLength * scaledCurvatureMax);
        result.thirdArcLength = inverseK * mod2pi(scaledThetaStart - scaledThetaEnd + scaledCurvatureMax * (result.secondArcLength - result.firstArcLength));
        result.isValid = true;
    }
    return result;
}

PathPrimitive calculateLRL(double scaledThetaStart, double scaledThetaEnd, double scaledCurvatureMax) {
    double const inverseK = 1 / scaledCurvatureMax;
    PathPrimitive result;

    double c = cos(scaledThetaEnd) - cos(scaledThetaStart);
    double s = 2 * scaledCurvatureMax + sin(scaledThetaStart) - sin(scaledThetaEnd);
    double temp1 = atan2(c, s);
    double temp2 = 0.125 * (6 - 4 * (scaledCurvatureMax * scaledCurvatureMax) + 2 * cos(scaledThetaStart - scaledThetaEnd) - 4 * scaledCurvatureMax * (sin(scaledThetaStart) - sin(scaledThetaEnd)));
    if (abs(temp2) > 1) {
        result.isValid = false;
    } else {
        result.secondArcLength = inverseK * mod2pi(2 * M_PI - acos(temp2));
        result.firstArcLength = inverseK * mod2pi(temp1 - scaledThetaStart + 0.5 * result.secondArcLength * scaledCurvatureMax);
        result.thirdArcLength = inverseK * mod2pi(scaledThetaEnd - scaledThetaStart + scaledCurvatureMax * (result.secondArcLength - result.firstArcLength));
        result.isValid = true;
    }
    return result;
}



PathPrimitive calculatePathPrimitive(CurveType curveType, double scaledThetaStart, double scaledThetaEnd, double scaledCurvatureMax) {
    switch (curveType) {
        case CurveType::LeftStraightLeft:
            return calculateLSL(scaledThetaStart, scaledThetaEnd, scaledCurvatureMax);
        case CurveType::RightStraightRight:
            return calculateRSR(scaledThetaStart, scaledThetaEnd, scaledCurvatureMax);
        case CurveType::LeftStraightRight:
            return calculateLSR(scaledThetaStart, scaledThetaEnd, scaledCurvatureMax);
        case CurveType::RightStraightLeft:
            return calculateRSL(scaledThetaStart, scaledThetaEnd, scaledCurvatureMax);
        case CurveType::RightLeftRight:
            return calculateRLR(scaledThetaStart, scaledThetaEnd, scaledCurvatureMax);
        case CurveType::LeftRightLeft:
            return calculateLRL(scaledThetaStart, scaledThetaEnd, scaledCurvatureMax);
        default:
            PathPrimitive result;
            result.isValid = false;
            return result;
    }
}

void sortPathCurvesByLength(std::vector<PathCurve>& curves) {
    if (curves.size() < 2) {
        return; 
    }
    bool swapped = true;
    int n = curves.size();
    while (swapped) {
        swapped = false;
        for (int i = 1; i < n; i++) {
            if (curves[i - 1].getLength() > curves[i].getLength()) {
                std::swap(curves[i - 1], curves[i]);
                swapped = true;
            }
        }
        --n; 
    }
}

std::vector<PathCurve> generatePathCurves(PathPoint startPoint, PathPoint endPoint, const double curvature) {
    std::vector<PathCurve> curves;
    constexpr int optimalCurvesCount = 6;

    const std::map<CurveType, std::vector<ArcDirection>> curveToArcTypes = {
        {CurveType::LeftStraightLeft, {ArcDirection::LeftTurn, ArcDirection::Straight, ArcDirection::LeftTurn}},
        {CurveType::RightStraightRight, {ArcDirection::RightTurn, ArcDirection::Straight, ArcDirection::RightTurn}},
        {CurveType::LeftStraightRight, {ArcDirection::LeftTurn, ArcDirection::Straight, ArcDirection::RightTurn}},
        {CurveType::RightStraightLeft, {ArcDirection::RightTurn, ArcDirection::Straight, ArcDirection::LeftTurn}},
        {CurveType::RightLeftRight, {ArcDirection::RightTurn, ArcDirection::LeftTurn, ArcDirection::RightTurn}},
        {CurveType::LeftRightLeft, {ArcDirection::LeftTurn, ArcDirection::RightTurn, ArcDirection::LeftTurn}}
    };

    Eigen::Vector4d scaled = scaleToStandard(startPoint, endPoint, curvature);

    for (int i = 0; i < optimalCurvesCount; i++) {
        CurveType ct = static_cast<CurveType>(i);
        PathPrimitive prim = calculatePathPrimitive(ct, scaled(0), scaled(1), scaled(2));
        if (prim.isValid) {
            Eigen::Vector3d curveLengths = scaleFromStandard(scaled(3), prim.firstArcLength, prim.secondArcLength, prim.thirdArcLength);
            auto arcTypes = curveToArcTypes.find(ct);
            PathArc arc0(startPoint, curveLengths(0), static_cast<double>(arcTypes->second[0]) * curvature);
            PathPoint intermediatePoint1 = arc0.getDestination();
            PathArc arc1(intermediatePoint1, curveLengths(1), static_cast<double>(arcTypes->second[1]) * curvature);
            PathPoint intermediatePoint2 = arc1.getDestination();
            PathArc arc2(intermediatePoint2, curveLengths(2), static_cast<double>(arcTypes->second[2]) * curvature);

            curves.push_back(PathCurve(arc0, arc1, arc2));
        }
    }

    sortPathCurvesByLength(curves);

    return curves;
}

PathCurve findShortestPathCurve(PathPoint startPoint, PathPoint endPoint, const double curvature) {
    auto curves = generatePathCurves(startPoint, endPoint, curvature);
    if (curves.empty()) {
        return PathCurve(); // Return an empty PathCurve if no path is found
    }
    return curves.at(0); // Assuming curves are sorted, return the shortest
}

bool checkPathArcIntersection(PathArc arc, Segment2D segment) {
    // If arc is straight, simply compute a line segment and check for intersection
    if (arc.curvature == 0) {
        Point2D source(arc.startPoint.xPos, arc.startPoint.yPos);
        PathPoint arcDest = arc.getDestination();
        Point2D dest(arcDest.xPos, arcDest.yPos);
        Segment2D arcSegment(source, dest);
        return intersect(segment, arcSegment); // Assuming intersect function exists for Segment2D types
    } else {
        // For a curved arc, build a circle to represent the arc and check intersection
        Point2D p1(arc.startPoint.xPos, arc.startPoint.yPos);

        PathArc arcMid(arc.startPoint, arc.arcLength * 0.5, arc.curvature);
        PathPoint midPoint = arcMid.getDestination();
        Point2D p2(midPoint.xPos, midPoint.yPos);

        PathPoint endPoint = arc.getDestination();
        Point2D p3(endPoint.xPos, endPoint.yPos);

        // Assuming getCircle function exists to find the circle from 3 points
        Circle circle = getCircle(p1, p2, p3);

        // Determine start and end points for intersection check based on arc curvature direction
        Point2D arcStart = (arc.curvature < 0) ? p3 : p1;
        Point2D arcEnd = (arc.curvature < 0) ? p1 : p3;

        return intersect(circle, segment, arcStart, arcEnd); // Assuming an intersect overload exists for this
    }
}

bool checkIntersection(PathCurve curve, Polygon polygon, int pointsCount) {
    std::vector<Segment2D> segments;
    auto points = curve.toPoints(pointsCount); // Use toPoints method adapted to PathCurve

    // Link consecutive points with segments
    for (size_t i = 1; i < points.size(); i++) {
        Point2D startPoint(points[i - 1].xPos, points[i - 1].yPos);
        Point2D endPoint(points[i].xPos, points[i].yPos);
        segments.emplace_back(startPoint, endPoint);
    }

    // Check if any segment of the approximated curve intersects with any polygon side
    for (const auto& curveSegment : segments) {
        for (const auto& polygonSide : polygon.getSides()) { // Assuming Polygon::getSides() returns a vector of Segment2D
            if (intersect(curveSegment, polygonSide)) { // Assuming an appropriate intersect function for Segment2D types
                return true;
            }
        }
    }
    return false;
}

std::string operator + (std::string s, const PathPoint& point) {
    return s + "(" + std::to_string(point.xPos) + ";" + std::to_string(point.yPos) + ";" + std::to_string(static_cast<int>(point.orientation * 180 / M_PI)) + ")";
}

std::string operator + (std::string s, const PathArc& arc) {
    return s + (arc.curvature == -1 ? "R" : (arc.curvature == 1 ? "L" : "S"));
}

std::string operator + (std::string s, const PathCurve& curve) {
    for (const auto& arc : curve.arcs) {
        s += arc; // Assuming PathArc has an operator+ overload to append string representation
    }
    return s;
}