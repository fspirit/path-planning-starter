//
// Created by Stanislav Olekhnovich on 21/10/2017.
//

#ifndef PATH_PLANNING_OTHERCAR_H
#define PATH_PLANNING_OTHERCAR_H

#include <cmath>

#include "CartesianPoint.h"
#include "FrenetPoint.h"


struct OtherCar
{
    CartesianPoint LocationCartesian;
    double XAxisSpeed;
    double YAxisSpeed;
    FrenetPoint LocationFrenet;
    inline double Speed2DMagnitude() const { return sqrt(XAxisSpeed * XAxisSpeed + YAxisSpeed * YAxisSpeed); }
    inline bool IsInLane(int laneNumber) const { return LocationFrenet.IsInLane(laneNumber); }
};


#endif //PATH_PLANNING_OTHERCAR_H
