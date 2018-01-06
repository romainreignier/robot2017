#ifndef TRAJECTO_H
#define TRAJECTO_H


class Trajecto
{
public:
    Trajecto();
    void MoveSquare();
    void GoingToPoint(const float & x, const float & y);
    void GoingToPoint(const float & x, const float & y, const float & theta);
};

#endif // TRAJECTO_H
