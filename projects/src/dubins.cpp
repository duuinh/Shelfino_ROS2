#include "dubins.hpp"
using namespace std;

// Returns a vector containing the approximation of a DubinsCurve using n points.
// If a straight arc is found inside the curve, its points will compose 10% of the total points.
// Points that compose a curve arc will be proportional to the length of the arc inside the curve.
std::vector<WayPoint> DubinsCurve::toPoints(int n) const {
    std::vector<WayPoint> points;
    const double straightCoeff = 0.1;
    double curvesLength = get_length();
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

std::vector<WayPoint> DubinsCurve::toPointsUniform(double step) const{
    std::vector<WayPoint> points;
    if (arcs.empty()) {
        return points;
    }
    points.push_back(arcs[0].startPoint);
    for (auto& arc : arcs) {
        int pointsCount = round(arc.arcLength / step) + 2;
        auto arcPoints = toPoints(pointsCount);
        if (!arcPoints.empty()) {
            points.insert(points.end(), arcPoints.begin() + 1, arcPoints.end());
        }
    }
    return points;
}

// Returns a vector of n points that approximates the arc.
std::vector<WayPoint> DubinsArc::toPoints(int n) const{
    std::vector<WayPoint> points;
    if (n < 2) {
        return points;
    }
    double unitSegmentLength = this->arcLength / (n - 1);
    for (int i = 0; i < n; i++) {
        double arcLengthPortion = unitSegmentLength * i;
        DubinsArc smallerArc(this->startPoint, arcLengthPortion, this->curvature);
        WayPoint point = smallerArc.getDestination();
        points.push_back(point);
    }
    return points;
}

WayPoint DubinsArc::getDestination() const{
    double x = this->startPoint.x + this->arcLength * sinc(this->curvature * this->arcLength / 2.0) * cos(this->startPoint.orientation + this->curvature * this->arcLength / 2);
    double y = this->startPoint.y + this->arcLength * sinc(this->curvature * this->arcLength / 2.0) * sin(this->startPoint.orientation + this->curvature * this->arcLength / 2);
    double orientation = mod2pi(this->startPoint.orientation + this->curvature * this->arcLength);
    return WayPoint(x, y, orientation);
}

/////////////////////////

Eigen::Vector4d scale_to_standard(WayPoint startPoint, WayPoint endPoint, const double curvature) {
    double dx = endPoint.x - startPoint.x;
    double dy = endPoint.y - startPoint.y;
    double phi = atan2(dy, dx);
    double lambda = hypot(dx, dy) / 2;

    double scaledThetaStart = mod2pi(startPoint.orientation - phi);
    double scaledThetaEnd = mod2pi(endPoint.orientation - phi);
    double scaledCurvatureMax = curvature * lambda;
    return Eigen::Vector4d(scaledThetaStart, scaledThetaEnd, scaledCurvatureMax, lambda);
}

Eigen::Vector3d scale_from_standard(double lambda, double scaledS1, double scaledS2, double scaledS3) {
    return Eigen::Vector3d(scaledS1 * lambda, scaledS2 * lambda, scaledS3 * lambda);
}

PrimitiveResult calc_LSL(double scaledThetaStart, double scaledThetaEnd, double scaledCurvatureMax) {
    double const inverseK = 1 / scaledCurvatureMax;
    PrimitiveResult result;

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

PrimitiveResult calc_RSR(double scaledThetaStart, double scaledThetaEnd, double scaledCurvatureMax) {
    double const inverseK = 1 / scaledCurvatureMax;
    PrimitiveResult result;

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

PrimitiveResult calc_LSR(double scaledThetaStart, double scaledThetaEnd, double scaledCurvatureMax) {
    double const inverseK = 1 / scaledCurvatureMax;
    PrimitiveResult result;

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

PrimitiveResult calc_RSL(double scaledThetaStart, double scaledThetaEnd, double scaledCurvatureMax) {
    double const inverseK = 1 / scaledCurvatureMax;
    PrimitiveResult result;

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

PrimitiveResult calc_RLR(double scaledThetaStart, double scaledThetaEnd, double scaledCurvatureMax) {
    double const inverseK = 1 / scaledCurvatureMax;
    PrimitiveResult result;

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

PrimitiveResult calc_LRL(double scaledThetaStart, double scaledThetaEnd, double scaledCurvatureMax) {
    double const inverseK = 1 / scaledCurvatureMax;
    PrimitiveResult result;

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

PrimitiveResult calc_primitive_result(CurveType curveType, double scaledThetaStart, double scaledThetaEnd, double scaledCurvatureMax) {
    switch (curveType) {
        case CurveType::LeftStraightLeft:
            return calc_LSL(scaledThetaStart, scaledThetaEnd, scaledCurvatureMax);
        case CurveType::RightStraightRight:
            return calc_RSR(scaledThetaStart, scaledThetaEnd, scaledCurvatureMax);
        case CurveType::LeftStraightRight:
            return calc_LSR(scaledThetaStart, scaledThetaEnd, scaledCurvatureMax);
        case CurveType::RightStraightLeft:
            return calc_RSL(scaledThetaStart, scaledThetaEnd, scaledCurvatureMax);
        case CurveType::RightLeftRight:
            return calc_RLR(scaledThetaStart, scaledThetaEnd, scaledCurvatureMax);
        case CurveType::LeftRightLeft:
            return calc_LRL(scaledThetaStart, scaledThetaEnd, scaledCurvatureMax);
        default:
            PrimitiveResult result;
            result.isValid = false;
            return result;
    }
}

DubinsCurve find_shortest_curve(WayPoint startPoint, WayPoint endPoint, const double curvature) {
    // generate curves
    std::vector<DubinsCurve> curves;
    constexpr int optimalCurvesCount = 6;

    const std::map<CurveType, std::vector<ArcDirection>> curveToArcTypes = {
        {CurveType::LeftStraightLeft, {ArcDirection::LeftTurn, ArcDirection::Straight, ArcDirection::LeftTurn}},
        {CurveType::RightStraightRight, {ArcDirection::RightTurn, ArcDirection::Straight, ArcDirection::RightTurn}},
        {CurveType::LeftStraightRight, {ArcDirection::LeftTurn, ArcDirection::Straight, ArcDirection::RightTurn}},
        {CurveType::RightStraightLeft, {ArcDirection::RightTurn, ArcDirection::Straight, ArcDirection::LeftTurn}},
        {CurveType::RightLeftRight, {ArcDirection::RightTurn, ArcDirection::LeftTurn, ArcDirection::RightTurn}},
        {CurveType::LeftRightLeft, {ArcDirection::LeftTurn, ArcDirection::RightTurn, ArcDirection::LeftTurn}}
    };

    Eigen::Vector4d scaled = scale_to_standard(startPoint, endPoint, curvature);

    for (int i = 0; i < optimalCurvesCount; i++) {
        CurveType ct = static_cast<CurveType>(i);
        PrimitiveResult prim = calc_primitive_result(ct, scaled(0), scaled(1), scaled(2));
        if (prim.isValid) {
            Eigen::Vector3d curveLengths = scale_from_standard(scaled(3), prim.firstArcLength, prim.secondArcLength, prim.thirdArcLength);
            auto arcTypes = curveToArcTypes.find(ct);
            DubinsArc arc0(startPoint, curveLengths(0), static_cast<double>(arcTypes->second[0]) * curvature);
            WayPoint intermediatePoint1 = arc0.getDestination();
            DubinsArc arc1(intermediatePoint1, curveLengths(1), static_cast<double>(arcTypes->second[1]) * curvature);
            WayPoint intermediatePoint2 = arc1.getDestination();
            DubinsArc arc2(intermediatePoint2, curveLengths(2), static_cast<double>(arcTypes->second[2]) * curvature);

            curves.push_back(DubinsCurve(arc0, arc1, arc2));
        }
    }

    // sort by length
    if (curves.size() > 1) {
        bool swapped = true;
        int n = curves.size();
        while (swapped) {
            swapped = false;
            for (int i = 1; i < n; i++) {
                if (curves[i - 1].get_length() > curves[i].get_length()) {
                    std::swap(curves[i - 1], curves[i]);
                    swapped = true;
                }
            }
            --n; 
        }
    }

    if (curves.empty()) {
        return DubinsCurve(); 
    }
    return curves.at(0); // return the shortest
}

bool checkDubinsArcIntersection(DubinsArc arc, Segment2D segment) {
    // If arc is straight, simply compute a line segment and check for intersection
    if (arc.curvature == 0) {
        Point2D source(arc.startPoint.x, arc.startPoint.y);
        WayPoint arcDest = arc.getDestination();
        Point2D dest(arcDest.x, arcDest.y);
        Segment2D arcSegment(source, dest);
        return intersect(segment, arcSegment); // Assuming intersect function exists for Segment2D types
    } else {
        // For a curved arc, build a circle to represent the arc and check intersection
        Point2D p1(arc.startPoint.x, arc.startPoint.y);

        DubinsArc arcMid(arc.startPoint, arc.arcLength * 0.5, arc.curvature);
        WayPoint midPoint = arcMid.getDestination();
        Point2D p2(midPoint.x, midPoint.y);

        WayPoint endPoint = arc.getDestination();
        Point2D p3(endPoint.x, endPoint.y);

        // Assuming getCircle function exists to find the circle from 3 points
        Circle circle = get_circle(p1, p2, p3);

        // Determine start and end points for intersection check based on arc curvature direction
        Point2D arcStart = (arc.curvature < 0) ? p3 : p1;
        Point2D arcEnd = (arc.curvature < 0) ? p1 : p3;

        return intersect(circle, segment, arcStart, arcEnd); // Assuming an intersect overload exists for this
    }
}

bool checkIntersection(DubinsCurve curve, Polygon polygon, int pointsCount) {
    std::vector<Segment2D> segments;
    auto points = curve.toPoints(pointsCount); // Use toPoints method adapted to DubinsCurve

    // Link consecutive points with segments
    for (size_t i = 1; i < points.size(); i++) {
        Point2D startPoint(points[i - 1].x, points[i - 1].y);
        Point2D endPoint(points[i].x, points[i].y);
        segments.emplace_back(startPoint, endPoint);
    }

    // Check if any segment of the approximated curve intersects with any polygon side
    for (const auto& curveSegment : segments) {
        for (const auto& polygonSide : polygon.get_sides()) { // Assuming Polygon::getSides() returns a vector of Segment2D
            if (intersect(curveSegment, polygonSide)) { // Assuming an appropriate intersect function for Segment2D types
                return true;
            }
        }
    }
    return false;
}

///////////////

std::vector<DubinsCurve> solve_multipoints_dubins (std::vector<WayPoint> &points, const double max_curvature) {
    int n = points.size();  // no. of points
    int k = 1000;             // no. of angle discretisation
    
    std::vector<double> total_length(2, 0);
    std::vector<DubinsCurve> dubins_path(n-1);
    
    for (int i = n-2; i >= 0; i--) { // start iterating from goal to start point
        if (i == 0) {
            DubinsCurve dubins = find_shortest_curve(points[i], points[i+1], max_curvature);
            dubins_path[i] = dubins;
            std::cout<<"MPDB>> (x,y,theta): "<<points[i].x<<", "<<points[i].y<<", "<<points[i].orientation<<std::endl;
        } else {
            double min_length = numeric_limits<double>::infinity();
            double opt_theta = numeric_limits<double>::infinity();
            double step_begin = points[i+1].orientation - M_PI;
            double step_end = points[i+1].orientation + M_PI;
            double step = (step_end - step_begin)/k;
            std::cout<<"MPDB>> th_start: "<<step_begin<<", th_end: "<<step_end<<", th_step: "<<step<<std::endl;

            for (double theta = step_begin; theta <= step_end; theta+=step) {
                points[i].orientation = mod2pi(theta);
                DubinsCurve dubins = find_shortest_curve(points[i], points[i+1], max_curvature);
                if (dubins.get_length() == 0) {
                    continue;
                }
                if (dubins.get_length() < min_length) {
                    dubins_path[i] = dubins;
                    opt_theta = mod2pi(theta);
                }
            }
            points[i].orientation = opt_theta;
            std::cout<<"MPDB>> (x,y,theta)_i+1: "<<points[i+1].x<<", "<<points[i+1].y<<", "<<points[i+1].orientation<<std::endl;
            std::cout<<"MPDB>> (x,y,theta)_i: "<<points[i].x<<", "<<points[i].y<<", "<<points[i].orientation<<std::endl;
        }
        
        total_length[0] = dubins_path[i].get_length() + total_length[1];
        total_length[1] = total_length[0];
    }

    std::cout<<"MPDB>> total_length: "<<total_length[0]<<std::endl;
        // print log
    for (size_t i = 0; i < dubins_path.size(); ++i) {
        std::cout<<"MPDB>> length: "<<dubins_path[i].get_length()<<std::endl;
    }
    return dubins_path;
}