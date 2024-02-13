#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <random>
#include <iostream>
#include <sstream>

using namespace std;

class Seed
{
    private:
        static int seed;
    public:
        static void init_seed(int seed); //seed=0 equal to random seed
        static int get_seed();
};

class ROSTimer
{
    private:
        clock_t t;
    public:
        ROSTimer(){t = clock();}
        double chk(){ return (clock()-t)/(double)CLOCKS_PER_SEC; }
        void rst(){t = clock();}
        double chk_rst(){ double ret=chk(); rst(); return ret; }
};
string operator + (string s, ROSTimer &timer);

class Logger
{
    private:
        static ROSTimer start;
    public:
        enum type {ERROR, WARNING, INFO};
        Logger(type t, string message);
        Logger(type t, stringstream& message);
};
#endif