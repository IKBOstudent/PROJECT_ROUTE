#include <fstream>

#include "readFile.h"

using std::ifstream;


void readFile(vector<Point>& points) {
    cout << "reading data from file\n";

    string filename = "../routes.txt";
    ifstream fin;
    fin.open(filename);

    int k = 0;

    Point* prev_point = nullptr;
    int prev_route = -1;
    if (fin.is_open()) {
        while (!fin.eof()) {
            int route_num;
            fin >> route_num;

            char a;
            fin.get(a);  // a = ' '
            while (a != '\n') {
                string name;

                fin.get(a);
                if (fin.eof()) break;

                if (a == '|') {
                    fin.get(a);
                    while (a != ':') {
                        name += a;
                        fin.get(a);
                    }

                    double from_prev, wait_time;
                    fin >> from_prev >> wait_time;
                    Point stop(route_num, k, name, from_prev, wait_time);

                    if (route_num != prev_route) {
                        if (prev_point != nullptr)
                            prev_point->set_endpoint();

                        stop.set_endpoint();
                        prev_route = route_num;
                    }

                    points.push_back(stop);
                    prev_point = &points[k];

                    k++;

                    fin.get(a);  // a = ':' or '|'
                }
                if (fin.eof()) break;
            }
        }
    }
    prev_point->set_endpoint();
    fin.close();
}


