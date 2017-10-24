//
// Created by Stanislav Olekhnovich on 17/10/2017.
//

#ifndef PATH_PLANNING_SIMPLESPLINEBASEDPLANNER_H
#define PATH_PLANNING_SIMPLESPLINEBASEDPLANNER_H

#include "PathPlanner.h"
#include "spline.h"

class SimpleSplineBasedPlanner : public PathPlanner
{
public:
    explicit SimpleSplineBasedPlanner(const HighwayMap &map, int startingLane): PathPlanner(map, startingLane), targetSpeed(.0) {};
    std::vector<CartesianPoint> GeneratePath(PathPlannerInput input) override;
private:
    double targetSpeed;

    bool IsTooCloseToOtherCar(const PathPlannerInput &input) const;

    std::vector<CartesianPoint> ConvertPointsToLocalSystem(const std::vector<CartesianPoint> &newPathAnchorPoints,
                               const CartesianPoint &localReferencePoint) const;

    tk::spline GetSplineFromAnchorPoints(const std::vector<CartesianPoint> &newPathAnchorPoints) const;

    std::vector<CartesianPoint> GenerateNewPointsWithSpline(const tk::spline &newPathSpline, int pointsLeftInCurrentPath) const;

    struct AnchorPointsGenerationResult
    {
        CartesianPoint ReferencePoint;
        std::vector<CartesianPoint> AnchorPoints;

        AnchorPointsGenerationResult(const CartesianPoint &ReferencePoint, const std::vector<CartesianPoint> &AnchorPoints)
                : ReferencePoint(ReferencePoint), AnchorPoints(AnchorPoints) {}
    };

    AnchorPointsGenerationResult GenerateAnchorPoints(const PathPlannerInput& input) const;
};



#endif //PATH_PLANNING_SIMPLESPLINEBASEDPLANNER_H
