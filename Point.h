#ifndef PROJECT_ROUTE_POINT_H
#define PROJECT_ROUTE_POINT_H

#include <string>
using std::string;


struct Point {
    int route;
    int index;
    string name;
    double from_prev;
    double wait_time;
    bool end = false;

    Point(int route_, int index_, string name_,
          double from_prev_, double wait_time_);

    ~Point()=default;

    void set_endpoint();
};


#endif //PROJECT_ROUTE_POINT_H
