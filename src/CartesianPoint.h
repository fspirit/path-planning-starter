//
// Created by Stanislav Olekhnovich on 13/10/2017.
//

#ifndef PATH_PLANNING_CARTESIANPOINT_H
#define PATH_PLANNING_CARTESIANPOINT_H


struct CartesianPoint
{
    double X;
    double Y;
    double Theta;

    CartesianPoint() = default;
    CartesianPoint(double X, double Y, double Theta = 0.0) : X(X), Y(Y), Theta(Theta) {}
    inline CartesianPoint ToLocal(const CartesianPoint &localReferencePoint) const
    {
        double shiftX = X - localReferencePoint.X;
        double shiftY = Y - localReferencePoint.Y;
        return {(shiftX * cos(-localReferencePoint.Theta) - shiftY * sin(-localReferencePoint.Theta)),
                (shiftX * sin(-localReferencePoint.Theta) + shiftY * cos(-localReferencePoint.Theta))};
    };
    inline CartesianPoint ToGlobal(const CartesianPoint& localReferencePoint) const
    {
        return { localReferencePoint.X + (X * cos(localReferencePoint.Theta) - Y * sin(localReferencePoint.Theta)),
                 localReferencePoint.Y + (X * sin(localReferencePoint.Theta) + Y * cos(localReferencePoint.Theta))};
    };

};


#endif //PATH_PLANNING_CARTESIANPOINT_H
