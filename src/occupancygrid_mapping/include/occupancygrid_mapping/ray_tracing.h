//
// Created by sam on 20/01/21.
//
#ifndef OCCUPANCYGRID_MAPPING_RAY_TRACING_H
#define OCCUPANCYGRID_MAPPING_RAY_TRACING_H

#include <vector>
#include <utility>
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace ray_tracing {
    typedef std::vector<std::pair<int,int>> vecPair;
    typedef std::pair<int, int> cellPoint;

    // define minimum and maximum allowable ranges
    float MINIMUM_RANGE = 0.15;
    float MAXIMUM_RANGE = 3.5;

    int euclideanDistance(cellPoint p0, cellPoint p1) {
        return sqrt(pow((p0.first-p1.first), 2) + pow((p0.second-p1.second), 2));
    }

    bool findRay(cellPoint p0, cellPoint p1, vecPair &result) {
        int x0 = p0.first;
        int y0 = p0.second;
        int x1 = p1.first;
        int y1 = p1.second;
        int distance = euclideanDistance(p0, p1);
        if ((distance < MINIMUM_RANGE)||(distance > MAXIMUM_RANGE))
            return false;
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
        return true;
    }
}

// TODO: convert x and y index to row major index
// TODO: add namespaces
// TODO: Determine occupancy
// TODO: add an object for points


#endif //OCCUPANCYGRID_MAPPING_RAY_TRACING_H
