//
// Created by Stanislav Olekhnovich on 12/10/2017.
//

#include "KeepLanePathPlanner.h"

std::vector<CartesianPoint> KeepLanePathPlanner::GeneratePath(PathPlannerInput input)
{
    std::vector<CartesianPoint> outputPath;
    double dist_inc = 0.3;
    for (int i= 0; i < 50; i++)
    {
        double next_s = input.LocationFrenet.S + (i + 1) * dist_inc;
        double next_d = 6;

        outputPath.push_back(map.FrenetToCartesian({next_s, next_d}));
    }
    return outputPath;
}
