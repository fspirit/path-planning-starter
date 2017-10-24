//
// Created by Stanislav Olekhnovich on 13/10/2017.
//

#ifndef PATH_PLANNING_HIGHWAYMAP_H
#define PATH_PLANNING_HIGHWAYMAP_H

#include <string>
#include <vector>
#include <cmath>

#include "CartesianPoint.h"
#include "FrenetPoint.h"

class HighwayMap
{
public:
    HighwayMap(const std::string& highwayMapCsvPath);
    CartesianPoint FrenetToCartesian(const FrenetPoint& frenetPoint) const;
    FrenetPoint CartesianToFrenet(const CartesianPoint& cartesianPoint) const;
    int NextWaypoint(CartesianPoint currentVehicleLocation) const;
    int ClosestWaypoint(CartesianPoint currentVehicleLocation) const;

private:
    std::vector<double> mapPointsX;
    std::vector<double> mapPointsY;
    std::vector<double> mapPointsS;
    std::vector<double> mapPointsDX;
    std::vector<double> mapPointsDY;

    void ReadMapFromCsvFile(const std::string& highwayMapCsvPath);
    inline double EuclidDistance(CartesianPoint p1, CartesianPoint p2) const
    {
        return sqrt((p2.X-p1.X)*(p2.X-p1.X)+(p2.Y-p1.Y)*(p2.Y-p1.Y));
    }
};

#endif //PATH_PLANNING_HIGHWAYMAP_H
