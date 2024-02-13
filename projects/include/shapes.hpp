#ifndef SHAPES_H
#define SHAPES_H

#include <cmath>
#include <vector>
#include <iostream>
#include <limits>
#include <string>
//#include "utils/utils.h"

class Point2D{
    public:
        double x,y;

        Point2D(){}
        Point2D(double x, double y):x{x},y{y}{}

        friend bool operator== (const Point2D& first, const Point2D& second) {return first.x == second.x && first.y == second.y;}
};
std::string operator + (std::string s, Point2D& p);

class Segment2D {
    public:
        Point2D p1, p2;
        Segment2D(){};
        Segment2D(Point2D p1, Point2D p2):p1{p1},p2{p2}{};

        bool contains(Point2D p);
        Point2D get_interceptor(double t);
        double get_angle();
        double get_slope();
};

class Circle{
    public:
        // radius
        double r;
        // center
        Point2D c;

        Circle(Point2D c, double r):r{r},c(c){}

        bool contains(Point2D p);
        double get_angle(Point2D p);
};

class Polygon{
    public:
        std::vector<Point2D> vertexes;

        Polygon():vertexes{}{}
        Polygon(std::vector<Point2D> vertexes):vertexes{vertexes}{}

        std::vector<Segment2D> get_sides();
        bool contains(Point2D p);
        void add_v(Point2D v){vertexes.push_back(v);}
        Point2D get_v(int index){return vertexes.at(index);}
        int get_size(){return vertexes.size();}
};

Point2D avg_point(Point2D p1, Point2D p2);
Circle get_circle(Point2D p1, Point2D p2, Point2D p3);
bool intersect(Segment2D s1, Segment2D s2);
bool intersect(Circle c, Segment2D s, Point2D start, Point2D end);
bool intersect(Polygon p, Segment2D s);
double distance(Point2D p1, Point2D p2);
Polygon inflate(Polygon p, double offset);
Polygon deflate(Polygon p, double offset);
Polygon inflate_intersect(Polygon p, double offset);
Polygon regular_polygon(Point2D center, double radius, int n);
Segment2D belong(Polygon pol, Point2D p, double offset);
Point2D translate(Point2D p, double offset, double th);

#endif
