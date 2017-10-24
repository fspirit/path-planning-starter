//
// Created by Stanislav Olekhnovich on 16/10/2017.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include "HighwayMap.h"
#include "PathPlannerInput.h"

class PathPlanner
{
public:
    explicit PathPlanner(const HighwayMap &map, int startingLane) : map(map), targetLane(startingLane) {}
    virtual std::vector<CartesianPoint> GeneratePath(PathPlannerInput input) = 0;

protected:
    const HighwayMap& map;
    int targetLane;
};


#endif //PATH_PLANNING_PATHPLANNER_H
