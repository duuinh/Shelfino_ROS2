#include "dubins.hpp"
using namespace std;

std::vector<WayPoint> DubinsCurve::get_points(double step) const{
    std::vector<WayPoint> points;
    if (arcs.empty()) {
        return points;
    }
    for (auto& arc : arcs) {
        auto arc_points = arc.get_points(step);
        if (!arc_points.empty()) {
            points.insert(points.end(), arc_points.begin(), arc_points.end());
        }
    }
    return points;
}

std::vector<WayPoint> DubinsArc::get_points(double step) const{
    std::vector<WayPoint> points;
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

    return (sqrt((eq1*eq1) + (eq2*eq2) + (eq3*eq3)) < 1.e-2) && length_positive;
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
        auto motions = dubins_word_to_motions.find(dubins_word)->second;
        std::vector<double> scaled_k = { static_cast<double>(motions[0]) * scaled_k_max, 
                                          static_cast<double>(motions[1]) * scaled_k_max, 
                                          static_cast<double>(motions[2]) * scaled_k_max };

        if (result.is_valid && curve_length < shortest_length && is_solution_valid(result, scaled_k, scaled_theta_0, scaled_theta_f)) {
            // scale back to original length
            double s1 = result.s1 * scaling_factor;
            double s2 = result.s2 * scaling_factor;
            double s3 = result.s3 * scaling_factor;
            
            DubinsArc arc1(start_point, s1, static_cast<double>(motions[0]) * max_curvature);
            DubinsArc arc2(arc1.get_final_point(), s2, static_cast<double>(motions[1]) * max_curvature);
            DubinsArc arc3(arc2.get_final_point(), s3, static_cast<double>(motions[2]) * max_curvature);
            DubinsCurve curve = DubinsCurve(arc1, arc2, arc3);

            if (check_collision(curve, borders, obstacles)) {
                continue;
            } else {
                shortest_length = curve_length;
                shortest_dubins = dubins_word;
                shortest_curve = curve;
            }
        }
    }

    return shortest_curve;
}
///////////////

bool check_collision(DubinsCurve curve, std::vector<GraphNode> &borders, std::vector<Obstacle> &obstacles) {
    // initialize path segment
    std::vector<h2d::Segment> segments;
    std::vector<WayPoint> points = curve.get_points(0.2);

    for (size_t i = 1; i < points.size(); i++) {
        h2d::Segment path_segment = h2d::Segment(h2d::Point2d(points[i - 1].x, points[i - 1].y), h2d::Point2d(points[i].x, points[i].y));
        segments.push_back(path_segment);
    }

    h2d::CPolyline map_poly;
    std::vector<h2d::Point2d> vertexes;

    for (GraphNode &border : borders)
    {
        vertexes.push_back(h2d::Point2d(border.x, border.y));
    }

    for (const auto& path_segment : segments) {
        // check if segment intersects with obstacle
        for (Obstacle &obstacle : obstacles) {

            if (obstacle.type == ObstacleType::CYLINDER)
            {
                h2d::Circle obs_circle = h2d::Circle(h2d::Point2d(obstacle.x, obstacle.y), obstacle.radius);
                if (obs_circle.intersects(path_segment).size() > 0)
                {
                    return true;
                }
            }

            else if (obstacle.type == ObstacleType::BOX)
            {
                h2d::CPolyline obs_box = h2d::CPolyline(std::vector<h2d::Point2d>{
                    {obstacle.x - obstacle.dx / 2.0 - INFLATION_RADIUS, obstacle.y + obstacle.dy / 2.0 + INFLATION_RADIUS},
                    {obstacle.x - obstacle.dx / 2.0 - INFLATION_RADIUS, obstacle.y - obstacle.dy / 2.0 - INFLATION_RADIUS},
                    {obstacle.x + obstacle.dx / 2.0 + INFLATION_RADIUS, obstacle.y - obstacle.dy / 2.0 - INFLATION_RADIUS},
                    {obstacle.x + obstacle.dx / 2.0 + INFLATION_RADIUS, obstacle.y + obstacle.dy / 2.0 + INFLATION_RADIUS}});

                if (obs_box.intersects(path_segment).size() > 0)
                {
                    return true;
                }
            }
        }

        // check if segment intersects with borders
        if (map_poly.intersects(path_segment).size() > 0)
        {
            return true;
        }
    }
    return false;
}

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
            double step = M_PI/4;

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