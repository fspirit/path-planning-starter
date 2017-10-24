//
// Created by Stanislav Olekhnovich on 12/10/2017.
//

#ifndef PATH_PLANNING_PATHPLANNERINPUT_H
#define PATH_PLANNING_PATHPLANNERINPUT_H

#include <vector>

#include "CartesianPoint.h"
#include "FrenetPoint.h"
#include "OtherCar.h"

struct PathPlannerInput
{
    CartesianPoint LocationCartesian;
    FrenetPoint LocationFrenet;
    FrenetPoint PathEndpointFrenet;

    double Speed;

    std::vector<CartesianPoint> Path;

    std::vector<double> PreviousPathX;
    std::vector<double> PreviousPathY;

    std::vector<OtherCar> OtherCars;
};


#endif //PATH_PLANNING_PATHPLANNERINPUT_H
