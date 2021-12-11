#include "Point.h"

Point::Point(int route_, int index_, string name_,
             double from_prev_, double wait_time_) {
    this->route = route_;
    this->index = index_;
    this->name = name_;
    this->from_prev = from_prev_;
    this->wait_time = wait_time_;
}

void Point::set_endpoint() {
    this->end = true;
}