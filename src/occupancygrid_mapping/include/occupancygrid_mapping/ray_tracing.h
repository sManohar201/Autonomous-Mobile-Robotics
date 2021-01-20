//
// Created by sam on 20/01/21.
//
#ifndef OCCUPANCYGRID_MAPPING_RAY_TRACING_H
#define OCCUPANCYGRID_MAPPING_RAY_TRACING_H

#include <vector>
#include <utility>

typedef std::vector<std::pair<int, int>> vecPair;

void findRay(int x0, int y0, int x1, int y1, vecPair &result) {
    int dx = abs(x1 - x0);
    int step_x = (x0 < x1)? 1 : -1;
    int dy = -abs(y1 - y0);
    int step_y = (y0 < y1) ? 1 : -1;
    int error = dx + dy;
    while (true) {
        result.push_back(std::make_pair(x0, y0));
        if ((x0==x1) && (y0==y1))
            break;
        int error2 = 2*error;
        if (error2 >= dy) {
            error += dy;
            x0 += step_x;
        }
        if (error2 < dx) {
            error += dx;
            y0 += step_y;
        }
    }
}

#endif //OCCUPANCYGRID_MAPPING_RAY_TRACING_H
