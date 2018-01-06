#include "trajecto.h"

#include "Board.h"

Trajecto::Trajecto()
{

}


void
Trajecto::MoveSquare(){
    //SetInitPosition();
    GoingToPoint(0,0,90);
    GoingToPoint(0,0,90);
    GoingToPoint(0,0,90);
    GoingToPoint(0,0,90);
}

void
Trajecto::GoingToPoint(const float & x, const float & y){
}

void
Trajecto::GoingToPoint(const float & x, const float & y, const float & theta){
}
