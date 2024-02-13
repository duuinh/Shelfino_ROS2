#include "shapes.hpp"
#include "dubins.hpp"
using namespace std;

bool is_bounded(double val, double min, double max){
    return val>=min && val<=max;
}

bool intersect(Segment2D s1, Segment2D s2, double& t_s1, double& t_s2){
    double det = (s2.p2.x - s2.p1.x) * (s1.p1.y - s1.p2.y) - (s1.p1.x - s1.p2.x) * (s2.p2.y - s2.p1.y);

    if(det == 0){
        // check if the segments overlap
        t_s1=-1;
        t_s2=-1;
        return s1.contains(s2.p1) || s1.contains(s2.p2) || s2.contains(s1.p1) || s2.contains(s1.p2);
    }

    t_s1 = ((s2.p1.y - s2.p2.y) * (s1.p1.x - s2.p1.x) + (s2.p2.x - s2.p1.x) * (s1.p1.y - s2.p1.y)) / det;
    t_s2 = ((s1.p1.y - s1.p2.y) * (s1.p1.x - s2.p1.x) + (s1.p2.x - s1.p1.x) * (s1.p1.y - s2.p1.y)) / det;
    return is_bounded(t_s1, 0, 1) && is_bounded(t_s2, 0, 1);
}

bool intersect(Segment2D s1, Segment2D s2, Point2D& p){
    double t, u;
    bool inter = intersect(s1, s2, t, u);
    p = Point2D(t*s1.p2.x+(1-t)*s1.p1.x, t*s1.p2.y+(1-t)*s1.p1.y);
    return inter;
}

bool intersect(Segment2D s1, Segment2D s2){
    Point2D p;
    return intersect(s1, s2, p);
}

bool intersect(Circle circle, Segment2D s, Point2D start, Point2D end){
    std::vector<Point2D> intersections = {};

    if(!circle.contains(start) || !circle.contains(end)){
        return false;
    }

    double delta_x = s.p2.x - s.p1.x;
    double delta_y = s.p2.y - s.p1.y;
    double a = pow(delta_x,2) + pow(delta_y,2);
    double b = delta_x * (s.p1.x - circle.c.x) + delta_y * (s.p1.y - circle.c.y);
    double cc = pow(s.p1.x - circle.c.x, 2) + pow(s.p1.y - circle.c.y, 2) - pow(circle.r,2);
    double delta = b*b - a*cc;
    if(delta < 0 || a == 0){
        return false;
    }
    delta = sqrt(delta);
    double t1 = (-b + delta) / a;
    double t2 = (-b - delta) / a;
    
    if(is_bounded(t1, 0, 1)){
        intersections.push_back(s.get_interceptor(t1));
    }

    if(is_bounded(t2, 0, 1)){
        intersections.push_back(s.get_interceptor(t2));
    }

    double start_angle = circle.get_angle(start);
    double end_angle = circle.get_angle(end);


    // TODO optimize the for loop in case that t1 and t2 are bounded and equals 

    bool intersected = false;

    // check if at least one of the intersections stays between the bounds
    for(auto inter: intersections){
        double inter_angle = circle.get_angle(inter);

        
        if(start_angle < end_angle){
            if(is_bounded(inter_angle, start_angle, end_angle)){
                intersected = true;
            }
        }
        else{
            if(inter_angle < end_angle || inter_angle > start_angle){
                intersected = true;
            }
        }
    }

    return intersected;
}

bool intersect(Polygon p, Segment2D s){
    if(p.contains(s.p1) || p.contains(s.p2)){
        return true;
    }
    auto sides = p.get_sides();
    for(auto side: sides){
        if(intersect(s, side)){
            return true;
        }
    }
    return false;
}

bool Segment2D::contains(Point2D p){
    double m = (p1.y - p2.y) / (p1.x - p2.x);
    double q = (p1.x * p2.y - p2.x * p1.y) / (p1.x - p2.x);
    double const err_threshold = 0.05;
    double err = abs(p.y - (m * p.x) - q);
    bool line_contains_p = is_bounded(err, 0, err_threshold);
    return line_contains_p && is_bounded(p.x, p1.x, p2.x) && is_bounded(p.y, p1.y, p2.y);
}


Point2D Segment2D::get_interceptor(double t){
    return Point2D(p1.x + t*(p2.x - p1.x), p1.y + t*(p2.y - p1.y));
}

double Circle::get_angle(Point2D p){
    return mod2pi(atan2(p.y - c.y, p.x - c.x));
}

bool Circle::contains(Point2D p){
    double const err_threshold = 0.05;
    double err = abs(pow(p.x - c.x, 2) + pow(p.y - c.y, 2) - pow(r,2));
    return is_bounded(err, 0, err_threshold);
}

Circle get_circle(Point2D p1, Point2D p2, Point2D p3){
    double x12 = p1.x - p2.x;
    double x13 = p1.x - p3.x;
 
    double y12 = p1.y - p2.y;
    double y13 = p1.y - p3.y;
 
    double y31 = p3.y - p1.y;
    double y21 = p2.y - p1.y;
 
    double x31 = p3.x - p1.x;
    double x21 = p2.x - p1.x;
 
    // x1^2 - x3^2
    double sx13 = pow(p1.x, 2) - pow(p3.x, 2);
 
    // y1^2 - y3^2
    double sy13 = pow(p1.y, 2) - pow(p3.y, 2);
 
    double sx21 = pow(p2.x, 2) - pow(p1.x, 2);
    double sy21 = pow(p2.y, 2) - pow(p1.y, 2);
 
    double f = ((sx13) * (x12)
             + (sy13) * (x12)
             + (sx21) * (x13)
             + (sy21) * (x13))
            / (2 * ((y31) * (x12) - (y21) * (x13)));
    double g = ((sx13) * (y12)
             + (sy13) * (y12)
             + (sx21) * (y13)
             + (sy21) * (y13))
            / (2 * ((x31) * (y12) - (x21) * (y13)));
 
    double c = -pow(p1.x, 2) - pow(p1.y, 2) - 2 * g * p1.x - 2 * f * p1.y;
 
    // eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0
    // where centre is (h = -g, k = -f) and radius r
    // as r^2 = h^2 + k^2 - c
    Point2D center(-g, -f);
    double sqr_of_r = center.x * center.x + center.y * center.y - c;
 
    // r is the radius
    float r = sqrt(sqr_of_r);

    return Circle(center, r);

}

std::vector<Segment2D> Polygon::get_sides(){
    std::vector<Segment2D> sides = {};
    for(int i=1;i<(int)vertexes.size();i++){
        Segment2D side(vertexes[i-1],vertexes[i]);
        sides.push_back(side);
    }
    Segment2D side(vertexes.back(),vertexes.front());
    sides.push_back(side);
    return sides;
}

bool Polygon::contains(Point2D p){
    int pos = 0;
    int neg = 0;

    auto sides = this->get_sides();
    for (auto side: sides){
        //Compute the cross product
        double d = (p.x - side.p1.x) * (side.p2.y - side.p1.y) - (p.y - side.p1.y) * (side.p2.x - side.p1.x);

        if (d > 0) pos++;
        if (d < 0) neg++;

        //If the sign changes, then point is outside
        if (pos > 0 && neg > 0)
            return false;
    }

    //If no change in direction, then on same side of all segments, and thus inside
    return true;
}

double distance(Point2D p1, Point2D p2){
    return sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
}

Point2D translate(Point2D p, double offset, double th){
    double offset_x = offset * cos(th);
    double offset_y = offset * sin(th);
    return Point2D(p.x + offset_x, p.y + offset_y);
}

Segment2D translate(Segment2D s, double offset, double th){
    Point2D node1_new = translate(s.p1, offset, th);
    Point2D node2_new = translate(s.p2, offset, th);
    return Segment2D(node1_new, node2_new);
}

double angle_between(Segment2D s1, Segment2D s2){
    double y = s2.get_slope() - s1.get_slope();
    double x = 1 + s1.get_slope() * s2.get_slope();
    return mod2pi(atan2(y,x));
}

// TODO add test with close angles
Polygon inflate(Polygon p, double offset){
    Polygon p_new;
    auto sides = p.get_sides();

    for(int i=0;i<(int)sides.size();i++){
        int index = i;
        auto curr_side = sides[index];
        double th = curr_side.get_angle();
        // compute the perpendicular angle
        th += (M_PI * 0.5);
        th = mod2pi(th);
        auto new_side = translate(curr_side, offset, th);
        p_new.add_v(new_side.p1);
        p_new.add_v(new_side.p2);

        // doing this we can check the last side with the first one
        if(index + 1 == (int)sides.size()){
            index = -1;
        }
        auto next_side = sides[index + 1];
        if(angle_between(curr_side, next_side) < M_PI * 0.3){
            auto curr_p_new = translate(curr_side.p2, offset, curr_side.get_angle());
            auto next_p_new = translate(next_side.p1, offset, mod2pi(next_side.get_angle() + M_PI));
            p_new.add_v(next_p_new);
            p_new.add_v(curr_p_new);
        }
    }
    return p_new;
}

Polygon deflate(Polygon p, double offset){
    Polygon p_new;
    auto sides = p.get_sides();

    //last side
    double th = sides.at(p.get_size()-1).get_angle();
    th += (M_PI * 1.5);
    th = mod2pi(th);
    auto prev_side = translate(sides.at(p.get_size()-1), offset, th);

    for(auto side: sides){
        double th = side.get_angle();
        th += (M_PI * 1.5);
        th = mod2pi(th);
        auto new_side = translate(side, offset, th);
        Point2D p;
        intersect(new_side, prev_side, p);
        p_new.add_v(p);
        prev_side = new_side;
    }
    return p_new;
}

Polygon inflate_intersect(Polygon p, double offset){
    Polygon p_new;
    std::vector<Segment2D> new_sides;
    auto sides = p.get_sides();

    for(auto side: sides){
        double th = side.get_angle();
        // compute the perpendicular angle
        double th_norm = th + (M_PI * 0.5);
        th_norm = mod2pi(th_norm);
        auto new_side = translate(side, offset, th_norm);
        // crazy scale the size of the new side so we ensure that the collision happens
        double const scale = 10;        
        double s_length = distance(new_side.p1, new_side.p2);
        auto new_node1 = translate(new_side.p1, s_length * scale, th + M_PI);
        auto new_node2 = translate(new_side.p2, s_length * scale, th);
        new_sides.push_back(Segment2D(new_node1, new_node2));
    }

    double t_s1, t_s2;

    for(int i=1;i<(int)new_sides.size();i++){
        auto s1 = new_sides.at(i-1);
        auto s2 = new_sides.at(i);
        // get the intersection point of every pair of consecutive segments
        if(intersect(s1, s2, t_s1, t_s2)){
            auto new_point = s1.get_interceptor(t_s1);
            p_new.add_v(new_point);
        }
    }

    if(intersect(new_sides.back(), new_sides.front(), t_s1, t_s2)){
        auto new_point = new_sides.back().get_interceptor(t_s1);
        p_new.add_v(new_point);
    }

    return p_new;
}

// returns the angle [0, 2*PI] between the segment and the x axis
// the direction is from p1 to p2
double Segment2D::get_angle(){
    double delta_x = p2.x - p1.x;
    double delta_y = p2.y - p1.y;
    return mod2pi(atan2(delta_y, delta_x));
}

double Segment2D::get_slope(){
    double delta_x = p2.x - p1.x;
    double delta_y = p2.y - p1.y;
    if(delta_x == 0){
        return std::numeric_limits<double>::max();
    }
    return delta_y / delta_x;
}

// build a regular polygon of n vertices given a radius and a center 
Polygon regular_polygon(Point2D center, double radius, int n){
    Polygon p;
    if(n < 3 || radius == 0){
        return p;
    }
    double unit_angle = (2 * M_PI) / n;
    for(int i = 0; i < n; i++){
        // -i so the polygon is built clockwise 
        double curr_angle = mod2pi(unit_angle * (-i));
        double x = center.x + radius * cos(curr_angle);
        double y = center.y + radius * sin(curr_angle);
        p.add_v(Point2D(x, y));
    }
    return p;
}

Point2D avg_point(Point2D p1, Point2D p2)
{
    return Point2D((p1.x+p2.x)/2, (p1.y+p2.y)/2);
}

string operator + (string s, Point2D& p)
{
    return s+"("+to_string(p.x)+";"+to_string(p.y)+")";
}

Segment2D belong(Polygon pol, Point2D p, double offset)
{
    for(auto side: pol.get_sides())
    {
        Polygon temp;
        Point2D t = translate(side.p1, offset, 0.5*M_PI);
        temp.add_v(t);
        t = translate(side.p1, offset, 1.5*M_PI);
        temp.add_v(t);
        t = translate(side.p2, offset, 1.5*M_PI);
        temp.add_v(t);
        t = translate(side.p2, offset, 0.5*M_PI);
        temp.add_v(t);
        if(temp.contains(p))
            return side;
    }
    return Segment2D();
}