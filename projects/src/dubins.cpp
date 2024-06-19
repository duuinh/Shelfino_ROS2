#include "dubins.hpp"
using namespace std;

std::vector<WayPoint> DubinsCurve::get_points() const{
    std::vector<WayPoint> points;
    if (arcs.empty()) {
        return points;
    }
    for (auto& arc : arcs) {
        auto arc_points = arc.get_points();
        if (!arc_points.empty()) {
            points.insert(points.end(), arc_points.begin() + 1, arc_points.end());
        }
    }
    return points;
}

std::vector<WayPoint> DubinsArc::get_points() const{
    std::vector<WayPoint> points;
    double step = 0.1;
    for (double l = 0.0; l < this->length; l+=step) {
        DubinsArc segment(this->start, l, this->curvature);
        WayPoint point = segment.get_final_point();
        points.push_back(point);
    }
    return points;
}

///////////////
// function to transform general setting to standard setting (reduces the numbers of params)
// @return (scaled_theta_0, scaled_theta_f, scaled_k_max, scaling_factor)
std::tuple<double, double, double, double> scale_to_standard_setting(WayPoint initial, WayPoint final, const double max_curvature) {
    double dx = final.x - initial.x;
    double dy = final.y - initial.y;
    double phi = atan2(dy, dx);
    double scaling_factor = 0.5 * hypot(dx, dy);

    double scaled_theta_0 = mod2pi(initial.orientation - phi);
    double scaled_theta_f = mod2pi(final.orientation - phi);
    double scaled_k_max = max_curvature * scaling_factor;

    return std::make_tuple(scaled_theta_0, scaled_theta_f, scaled_k_max, scaling_factor);
}

PrimitiveResult get_result_LSL(double scaled_theta_0, double scaled_theta_f, double scaled_k_max) {
    PrimitiveResult result;
    double inverse_k = 1 / scaled_k_max;
    double C = cos(scaled_theta_f) - cos(scaled_theta_0);
    double S = 2 * scaled_k_max + sin(scaled_theta_0) - sin(scaled_theta_f);
    double temp1 = atan2(C, S);
    double temp2 = 2 + 4 * (scaled_k_max * scaled_k_max) - 2 * cos(scaled_theta_0 - scaled_theta_f) + 4 * scaled_k_max * (sin(scaled_theta_0) - sin(scaled_theta_f));
    
    if (temp2 >= 0) {
        result.is_valid = true;
        result.s1 = inverse_k * mod2pi(temp1 - scaled_theta_0);
        result.s2 = inverse_k * sqrt(temp2);
        result.s3 = inverse_k * mod2pi(scaled_theta_f - temp1);
    }
    return result;
}

PrimitiveResult get_result_RSR(double scaled_theta_0, double scaled_theta_f, double scaled_k_max) {
    PrimitiveResult result;
    double inverse_k = 1 / scaled_k_max;
    double C = cos(scaled_theta_0) - cos(scaled_theta_f);
    double S = 2 * scaled_k_max - sin(scaled_theta_0) + sin(scaled_theta_f);
    double temp1 = atan2(C, S);
    double temp2 = 2 + 4 * (scaled_k_max * scaled_k_max) - 2 * cos(scaled_theta_0 - scaled_theta_f) - 4 * scaled_k_max * (sin(scaled_theta_0) - sin(scaled_theta_f));
    
    if (temp2 >= 0) {
        result.is_valid = true;
        result.s1 = inverse_k * mod2pi(scaled_theta_0 - temp1);
        result.s2 = inverse_k * sqrt(temp2);
        result.s3 = inverse_k * mod2pi(temp1 - scaled_theta_f);
    }
    return result;
}

PrimitiveResult get_result_LSR(double scaled_theta_0, double scaled_theta_f, double scaled_k_max) {
    PrimitiveResult result;
    double inverse_k = 1 / scaled_k_max;
    double C = cos(scaled_theta_0) + cos(scaled_theta_f);
    double S = 2 * scaled_k_max + sin(scaled_theta_0) + sin(scaled_theta_f);
    double temp1 = atan2(-C, S);
    double temp2 = -2 + 4 * (scaled_k_max * scaled_k_max) + 2 * cos(scaled_theta_0 - scaled_theta_f) + 4 * scaled_k_max * (sin(scaled_theta_0) + sin(scaled_theta_f));

    if (temp2 >= 0) {
        result.is_valid = true;
        result.s2 = inverse_k * sqrt(temp2);
        double temp3 = atan2(-2, scaled_k_max * result.s2);
        result.s1 = inverse_k * mod2pi(temp1 - temp3 - scaled_theta_0);
        result.s3 = inverse_k * mod2pi(temp1 - temp3 - scaled_theta_f);
    }
    return result;
}

PrimitiveResult get_result_RSL(double scaled_theta_0, double scaled_theta_f, double scaled_k_max) {
    PrimitiveResult result;
    double inverse_k = 1 / scaled_k_max;
    double C = cos(scaled_theta_0) + cos(scaled_theta_f);
    double S = 2 * scaled_k_max - sin(scaled_theta_0) - sin(scaled_theta_f);
    double temp1 = atan2(C, S);
    double temp2 = -2 + 4 * (scaled_k_max * scaled_k_max) + 2 * cos(scaled_theta_0 - scaled_theta_f) - 4 * scaled_k_max * (sin(scaled_theta_0) + sin(scaled_theta_f));
    
    if (temp2 >= 0) {
        result.is_valid = true;
        result.s2 = inverse_k * sqrt(temp2);
        double temp3 = atan2(2, scaled_k_max * result.s2);
        result.s1 = inverse_k * mod2pi(scaled_theta_0 - temp1 + temp3);
        result.s3 = inverse_k * mod2pi(scaled_theta_f - temp1 + temp3);
    }
    return result;
}

PrimitiveResult get_result_RLR(double scaled_theta_0, double scaled_theta_f, double scaled_k_max) {
    PrimitiveResult result;
    double inverse_k = 1 / scaled_k_max;
    double C = cos(scaled_theta_0) - cos(scaled_theta_f);
    double S = 2 * scaled_k_max - sin(scaled_theta_0) + sin(scaled_theta_f);
    double temp1 = atan2(C, S);
    double temp2 = 0.125 * (6 - 4 * (scaled_k_max * scaled_k_max) + 2 * cos(scaled_theta_0 - scaled_theta_f) + 4 * scaled_k_max * (sin(scaled_theta_0) - sin(scaled_theta_f)));
    
    if (abs(temp2) <= 1) {
        result.is_valid = true;
        result.s2 = inverse_k * mod2pi(2 * M_PI - acos(temp2));
        result.s1 = inverse_k * mod2pi(scaled_theta_0 - temp1 + 0.5 * result.s2 * scaled_k_max);
        result.s3 = inverse_k * mod2pi(scaled_theta_0 - scaled_theta_f + scaled_k_max * (result.s2 - result.s1));
    }
    return result;
}

PrimitiveResult get_result_LRL(double scaled_theta_0, double scaled_theta_f, double scaled_k_max) {
    PrimitiveResult result;
    double inverse_k = 1 / scaled_k_max;
    double C = cos(scaled_theta_f) - cos(scaled_theta_0);
    double S = 2 * scaled_k_max + sin(scaled_theta_0) - sin(scaled_theta_f);
    double temp1 = atan2(C, S);
    double temp2 = 0.125 * (6 - 4 * (scaled_k_max * scaled_k_max) + 2 * cos(scaled_theta_0 - scaled_theta_f) - 4 * scaled_k_max * (sin(scaled_theta_0) - sin(scaled_theta_f)));
    
    if (abs(temp2) <= 1) {
        result.is_valid = true;
        result.s2 = inverse_k * mod2pi(2 * M_PI - acos(temp2));
        result.s1 = inverse_k * mod2pi(-scaled_theta_0 + temp1  + 0.5 * result.s2 * scaled_k_max);
        result.s3 = inverse_k * mod2pi(scaled_theta_f - scaled_theta_0 + scaled_k_max * (result.s2 - result.s1));
    }
    return result;
}

PrimitiveResult calculate_primitive_result(DubinsWord word, double scaled_theta_0, double scaled_theta_f, double scaled_k_max) {
    PrimitiveResult result;
    switch (word) {
        case DubinsWord::LSL:
            result = get_result_LSL(scaled_theta_0, scaled_theta_f, scaled_k_max);
            break;
        case DubinsWord::RSR:
            result = get_result_RSR(scaled_theta_0, scaled_theta_f, scaled_k_max);
            break;
        case DubinsWord::LSR:
            result = get_result_LSR(scaled_theta_0, scaled_theta_f, scaled_k_max);
            break;
        case DubinsWord::RSL:
            result = get_result_RSL(scaled_theta_0, scaled_theta_f, scaled_k_max);
            break;
        case DubinsWord::RLR:
            result = get_result_RLR(scaled_theta_0, scaled_theta_f, scaled_k_max);
            break;
        case DubinsWord::LRL:
            result = get_result_LRL(scaled_theta_0, scaled_theta_f, scaled_k_max);
            break;
        default:
            break;
    }
    return result;
}

bool is_solution_valid(PrimitiveResult result, std::vector<double> k, double scaled_theta_0, double scaled_theta_f)
{
    if (!result.is_valid) {
        return false;
    }

    int x0 = -1, y0 = 0;
    int xf = 1, yf = 0;

    double eq1, eq2, eq3;
    bool length_positive;

    // 3 cardinal equations
    eq1 = result.s1 * sinc(1/2. * k[0] * result.s1) * cos(scaled_theta_0 + 1/2. * k[0] * result.s1)
        + result.s2 * sinc(1/2. * k[1] * result.s2) * cos(scaled_theta_0 + k[0] * result.s1 + 1/2. * k[1] * result.s2)
        + result.s3 * sinc(1/2. * k[2] * result.s3) * cos(scaled_theta_0 + k[0] * result.s1 + k[1] * result.s2 + 1/2. * k[2] * result.s3)
        + x0 - xf;
    eq2 = result.s1 * sinc(1/2. * k[0] * result.s1) * sin(scaled_theta_0 + 1/2. * k[0] * result.s1)
        + result.s2 * sinc(1/2. * k[1] * result.s2) * sin(scaled_theta_0 + k[0] * result.s1 + 1/2. * k[1] * result.s2)
        + result.s3 * sinc(1/2. * k[2] * result.s3) * sin(scaled_theta_0 + k[0] * result.s1 + k[1] * result.s2 + 1/2. * k[2] * result.s3)
        + y0 - yf;
    eq3 = modpi(k[0] * result.s1 + k[1] * result.s2 + k[2] * result.s3 + scaled_theta_0 - scaled_theta_f);

    // s(i) >= 0
    length_positive = (result.s1 > 0) || (result.s2 > 0) || (result.s3 > 0);

    return (sqrt((eq1*eq1) + (eq2*eq2) + (eq3*eq3)) < 2.e-1) && length_positive;
}

DubinsCurve find_shortest_curve(WayPoint start_point, WayPoint endPoint, const double max_curvature, std::vector<GraphNode> &borders, std::vector<Obstacle> &obstacles) {
    DubinsCurve shortest_curve = DubinsCurve();
    DubinsWord shortest_dubins = DubinsWord::UNDEFINED;
    double shortest_length = std::numeric_limits<double>::max();
    const int possible_solutions = 6;

    double scaled_theta_0, scaled_theta_f, scaled_k_max, scaling_factor;
    std::tie(scaled_theta_0, scaled_theta_f, scaled_k_max, scaling_factor) = scale_to_standard_setting(start_point, endPoint, max_curvature);

    for (int i = 0; i < possible_solutions; i++) {
        // calculate primitive result for each solution
        DubinsWord dubins_word = static_cast<DubinsWord>(i);
        PrimitiveResult result = calculate_primitive_result(dubins_word, scaled_theta_0, scaled_theta_f, scaled_k_max);
        double curve_length = result.s1 + result.s2 + result.s3;
        if (result.is_valid && curve_length < shortest_length) {
            shortest_length = curve_length;
            shortest_dubins = dubins_word;
        }
    }

    if (shortest_dubins != DubinsWord::UNDEFINED) {
        PrimitiveResult result = calculate_primitive_result(shortest_dubins, scaled_theta_0, scaled_theta_f, scaled_k_max);
        
        // scale back to original length
        double s1 = result.s1 * scaling_factor;
        double s2 = result.s2 * scaling_factor;
        double s3 = result.s3 * scaling_factor;
        
        auto motions = dubins_word_to_motions.find(shortest_dubins)->second;
        std::vector<double> scaled_k = { static_cast<double>(motions[0]) * scaled_k_max, 
                                          static_cast<double>(motions[1]) * scaled_k_max, 
                                          static_cast<double>(motions[2]) * scaled_k_max };
        
        bool is_valid = is_solution_valid(result, scaled_k, scaled_theta_0, scaled_theta_f);
        if (is_valid) {
            DubinsArc arc1(start_point, s1, static_cast<double>(motions[0]) * max_curvature);
            DubinsArc arc2(arc1.get_final_point(), s2, static_cast<double>(motions[1]) * max_curvature);
            DubinsArc arc3(arc2.get_final_point(), s3, static_cast<double>(motions[2]) * max_curvature);
            shortest_curve = DubinsCurve(arc1, arc2, arc3);
        }
    }

    return shortest_curve;
}
///////////////

// bool checkDubinsArcIntersection(DubinsArc arc, Segment2D segment) {
//     // If arc is straight, simply compute a line segment and check for intersection
//     if (arc.curvature == 0) {
//         Point2D source(arc.start.x, arc.start.y);
//         WayPoint arcDest = arc.get_final_point();
//         Point2D dest(arcDest.x, arcDest.y);
//         Segment2D arcSegment(source, dest);
//         return intersect(segment, arcSegment); // Assuming intersect function exists for Segment2D types
//     } else {
//         // For a curved arc, build a circle to represent the arc and check intersection
//         Point2D p1(arc.start.x, arc.start.y);

//         DubinsArc arcMid(arc.start, arc.length * 0.5, arc.curvature);
//         WayPoint midPoint = arcMid.get_final_point();
//         Point2D p2(midPoint.x, midPoint.y);

//         WayPoint endPoint = arc.get_final_point();
//         Point2D p3(endPoint.x, endPoint.y);

//         // Assuming getCircle function exists to find the circle from 3 points
//         Circle circle = get_circle(p1, p2, p3);

//         // Determine start and end points for intersection check based on arc curvature direction
//         Point2D arcStart = (arc.curvature < 0) ? p3 : p1;
//         Point2D arcEnd = (arc.curvature < 0) ? p1 : p3;

//         return intersect(circle, segment, arcStart, arcEnd); // Assuming an intersect overload exists for this
//     }
// }

// bool checkIntersection(DubinsCurve curve, Polygon polygon, int pointsCount) {
//     std::vector<Segment2D> segments;
//     auto points = curve.toPoints(pointsCount); // Use toPoints method adapted to DubinsCurve

//     // Link consecutive points with segments
//     for (size_t i = 1; i < points.size(); i++) {
//         Point2D start_point(points[i - 1].x, points[i - 1].y);
//         Point2D endPoint(points[i].x, points[i].y);
//         segments.emplace_back(start_point, endPoint);
//     }

//     // Check if any segment of the approximated curve intersects with any polygon side
//     for (const auto& curveSegment : segments) {
//         for (const auto& polygonSide : polygon.get_sides()) { // Assuming Polygon::getSides() returns a vector of Segment2D
//             if (intersect(curveSegment, polygonSide)) { // Assuming an appropriate intersect function for Segment2D types
//                 return true;
//             }
//         }
//     }
//     return false;
// }

///////////////

std::vector<DubinsCurve> solve_multipoints_dubins (std::vector<WayPoint> &points, const double max_curvature, std::vector<GraphNode> &borders, std::vector<Obstacle> &obstacles) {
    int n = points.size();  // no. of points
    
    std::vector<double> total_length(2, 0);
    std::vector<DubinsCurve> dubins_path(n-1);
    
    for (int i = n-2; i >= 0; i--) { // start iterating from goal to start point
        if (i == 0) {
            DubinsCurve dubins = find_shortest_curve(points[i], points[i+1], max_curvature, borders, obstacles);
            dubins_path[i] = dubins;
        } else {
            double min_length = numeric_limits<double>::infinity();
            double opt_theta = numeric_limits<double>::infinity();
            double step_begin = 0;
            double step_end = 2* M_PI;
            double step = M_PI/8;

            for (double theta = step_begin; theta < step_end; theta+=step) {
                points[i].orientation = theta;
                DubinsCurve dubins = find_shortest_curve(points[i], points[i+1], max_curvature, borders, obstacles);
                if (dubins.get_length() == 0) {
                    continue;
                }
                if (dubins.get_length() < min_length) {
                    dubins_path[i] = dubins;
                    opt_theta = theta;
                    min_length = dubins.get_length();
                }
            }
            points[i].orientation = opt_theta;
        }
        
        total_length[0] = dubins_path[i].get_length() + total_length[1];
        total_length[1] = total_length[0];
    }

    std::cout<<"MPDB| total_length: "<<total_length[0]<<std::endl;
        // print log
    for (size_t i = 0; i < dubins_path.size(); ++i) {
        std::cout<<"MPDB| length: "<<dubins_path[i].get_length()<<std::endl;
    }
    return dubins_path;
}