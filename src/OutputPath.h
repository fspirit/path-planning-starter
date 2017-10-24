//
// Created by Stanislav Olekhnovich on 12/10/2017.
//

#ifndef PATH_PLANNING_OUTPUTPATH_H
#define PATH_PLANNING_OUTPUTPATH_H


struct OutputPath
{
    std::vector<double> PathX;
    std::vector<double> PathY;

    std::vector<CartesianPoint> Path;
};


#endif //PATH_PLANNING_OUTPUTPATH_H
