#include "readFile.h"
#include "Point.h"

#include <map>
#include <set>
#include <algorithm>

using std::pair, std::make_pair, std::map, std::set;
using std::reverse;


int main() {
    vector<Point> points;
    // reading from file
    readFile(points);

    cout << "user input: start and end point\n";

    string start_name, end_name;
    while (true) {
        getline(cin, start_name);
        getline(cin, end_name);

        bool found_A = false, found_B = false;
        for (auto & i : points) {
            if (!found_A and i.name == start_name)
                found_A = true;

            if (!found_B and i.name == end_name)
                found_B = true;
        }

        if (found_A and found_B and start_name != end_name) break;
        else cout << "invalid name\n";
    }

    cout << "building a graph\n";

    int N = (int) points.size();
    Point fantom_point(0, N, "", 0, 0);
    points.push_back(fantom_point);

    // building a graph
    vector<vector<pair<int, double>>> GRAPH(N + 1);

    for (int i = 0; i < N + 1; ++i) {
        for (int j = 0; j < N; ++j) {
            double time = 0;

            if (i != N) {
                if (points[i].route == points[j].route and abs(j - i) == 1) {
                    if (i < j) {
                        if (!points[i].end)
                            time += points[i].from_prev / 2;

                        time += points[j].from_prev / 2;
                    } else {
                        if (!points[i].end)
                            time += points[i+1].from_prev / 2;

                        time += points[i].from_prev / 2;
                    }

                    time += points[i].wait_time;
                    GRAPH[i].push_back(make_pair(j, time));
                } else if (points[i].name == points[j].name and i != j) {
                    GRAPH[i].push_back(make_pair(j, 0));
                }
            } else {
                if (points[j].name == start_name) {
                    GRAPH[i].push_back(make_pair(j, 0));
                }
            }
        }
    }

    cout << "finding path...\n";

    // Dijkstra's ALGORITHM
    set<pair<double, int>> results;

    map<Point*, pair<Point*, double>> came_from;
    came_from[&fantom_point] = make_pair(nullptr, 0);

    const int INF = 5000;

    vector<double> total_cost(N+1, INF);
    total_cost[N] = 0;

    set<pair<double, int>> not_used;  // queue (time, index)
    not_used.insert(make_pair(0, N));

    while (!not_used.empty()) {
        // pop the first elem in a queue
        int current = not_used.begin()->second;
        not_used.erase(not_used.begin());

        for (auto neighbour : GRAPH[current]) {
            int next = neighbour.first;
            double to_next = neighbour.second;

            bool visited = false;
            for (auto i : came_from) {
                if (i.second.first != nullptr and i.first->index == current and i.second.first->index == next) {
                    visited = true;
                    break;
                }
            }

            if (!visited) {
                bool flag_B = false;

                if (points[next].name == end_name) {
                    flag_B = true;
                    if (current < next)
                        to_next += points[next].from_prev / 2;
                    else
                        to_next += points[current].from_prev / 2;
                } else if (points[current].name == points[next].name and points[next].name != start_name)
                    to_next += came_from.find(&points[current])->second.second; // transfer

                // time fix recalculation
                double time_fix = 0;
                if (points[current].name != points[next].name) {
                    if (current < next)
                        time_fix = points[next].from_prev / 2;
                    else
                        time_fix = points[current].from_prev / 2;
                }

                // if moving to next is reasonable
                if (total_cost[current] + to_next < total_cost[next]) {
                    came_from[&points[next]] = make_pair(&points[current], time_fix);

                    not_used.erase(make_pair(total_cost[next], next));

                    total_cost[next] = total_cost[current] + to_next;
                    auto pair = make_pair(total_cost[next], next);

                    if (!flag_B)
                        not_used.insert(pair);
                    else
                        results.insert(pair);
                }
            }
        }
    }

    double result = results.begin()->first;
    int finish = results.begin()->second;

    vector<Point*> PATH;
    auto current = &points[finish];
    while (current != &points[N]) {
        current = came_from[current].first;
        if (current == &points[N]) break;
        PATH.push_back(current);
    }
    reverse(PATH.begin(),  PATH.end()); // found path

    cout << "\nTotal time: " << result << " minutes" << endl;

    int prev_route = 0;
    bool begin = true;
    for (auto i : PATH) {
        if (!begin and i->route != prev_route)
            cout << "transfer -> ";
        else
            cout << i->name;
        cout << " (bus No. " << i->route << ") \n";
        prev_route = i->route;
        begin = false;
    }
    cout << points[finish].name << " (get off)\n";

    return 0;
}