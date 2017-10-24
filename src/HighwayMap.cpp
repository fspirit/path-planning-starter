//
// Created by Stanislav Olekhnovich on 13/10/2017.
//
#include "HighwayMap.h"

#include <thread>
#include <uWS/uWS.h>
#include <fstream>
#include <sstream>

HighwayMap::HighwayMap(const std::string &highwayMapCsvPath)
{
    ReadMapFromCsvFile(highwayMapCsvPath);
}

CartesianPoint HighwayMap::FrenetToCartesian(const FrenetPoint& frenetPoint) const
{
    int previousPoint = -1;

    while(frenetPoint.S > mapPointsS[previousPoint+1] && (previousPoint < (int)(mapPointsS.size()-1) ))
    {
        previousPoint++;
    }

    auto wp2 = static_cast<int>((previousPoint + 1) % mapPointsX.size());

    double heading = atan2((mapPointsY[wp2]-mapPointsY[previousPoint]),(mapPointsX[wp2]-mapPointsX[previousPoint]));
    // the x,y,s along the segment
    double seg_s = (frenetPoint.S-mapPointsS[previousPoint]);

    double seg_x = mapPointsX[previousPoint]+seg_s*cos(heading);
    double seg_y = mapPointsY[previousPoint]+seg_s*sin(heading);

    double perp_heading = heading-M_PI_2;

    double x = seg_x + frenetPoint.D * cos(perp_heading);
    double y = seg_y + frenetPoint.D * sin(perp_heading);

    return {x,y};
}

FrenetPoint HighwayMap::CartesianToFrenet(const CartesianPoint& cartesianPoint) const
{
    int next_wp = NextWaypoint(cartesianPoint);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = mapPointsX.size()-1;
    }

    double n_x = mapPointsX[next_wp]-mapPointsX[prev_wp];
    double n_y = mapPointsY[next_wp]-mapPointsY[prev_wp];
    double x_x = cartesianPoint.X - mapPointsX[prev_wp];
    double x_y = cartesianPoint.Y - mapPointsY[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = EuclidDistance(cartesianPoint, {proj_x,proj_y});

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-mapPointsX[prev_wp];
    double center_y = 2000-mapPointsY[prev_wp];
    double centerToPos = EuclidDistance({center_x,center_y}, {x_x,x_y});
    double centerToRef = EuclidDistance({center_x,center_y}, {proj_x,proj_y});

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += EuclidDistance({mapPointsX[i], mapPointsY[i]}, {mapPointsX[i+1],mapPointsY[i+1]});
    }

    frenet_s += EuclidDistance({0,0} ,{proj_x,proj_y});

    return {frenet_s,frenet_d};
}

int HighwayMap::NextWaypoint(CartesianPoint currentVehicleLocation) const
{
    int closestWaypoint = ClosestWaypoint(currentVehicleLocation);

    double map_x = mapPointsX[closestWaypoint];
    double map_y = mapPointsY[closestWaypoint];

    double heading = atan2( (map_y- currentVehicleLocation.Y),(map_x- currentVehicleLocation.X) );

    double angle = std::abs(currentVehicleLocation.Theta - heading);

    if(angle > M_PI_4)
    {
        closestWaypoint++;
    }

    return closestWaypoint;
}

int HighwayMap::ClosestWaypoint(CartesianPoint currentVehicleLocation) const
{
    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < mapPointsX.size(); i++)
    {
        double map_x = mapPointsX[i];
        double map_y = mapPointsY[i];
        double dist = EuclidDistance(currentVehicleLocation, {map_x, map_y});
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;
}

void HighwayMap::ReadMapFromCsvFile(const std::string &highwayMapCsvPath)
{
    std::ifstream in_map_(highwayMapCsvPath.c_str(), std::ifstream::in);

    std::string line;
    while (getline(in_map_, line))
    {
        std::istringstream iss(line);
        double x, y, s, d_x, d_y;
        iss >> x >> y >> s >> d_x >> d_y;

        mapPointsX.push_back(x);
        mapPointsY.push_back(y);
        mapPointsS.push_back(s);
        mapPointsDX.push_back(d_x);
        mapPointsDY.push_back(d_y);
    }
}


