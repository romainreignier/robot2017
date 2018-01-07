#include "trajecto.h"
#include "Board.h"
#include <cmath>
#include <cstdlib>

void
MoveSquare(){
    gBoard.SetInitPosition(135,117.5,90);
//    GoingToPoint(0,100);
//    GoingToPoint(-100,100);
//    GoingToPoint(-100,0);
//    GoingToPoint(0,0);
      GoingToPoint(211,145);
      GoingToPoint(292,200);
      GoingToPoint(354,275);
      GoingToPoint(402,392);
      GoingToPoint(410,467.5,90);
}

void
GoingToPoint(const float & pX, const float & pY){
    float directionTheta = atan2(pY - gBoard.G_Y_mm,pX - gBoard.G_X_mm);
    float ldA = gBoard.normalize_angle(directionTheta - gBoard.G_Theta_rad);

    gBoard.moveAngular(ldA);

    float distance = hypot((pX - gBoard.G_X_mm),(pY - gBoard.G_Y_mm));

    gBoard.moveLinear(distance);
}

void
GoingToPoint(const float & pX, const float & pY, const float & pTheta_deg){
    float lTheta_rad = pTheta_deg * DTOR;

    float directionTheta = atan2(pY - gBoard.G_Y_mm,pX - gBoard.G_X_mm);
    float ldA = gBoard.normalize_angle(directionTheta - gBoard.G_Theta_rad);

    gBoard.moveAngular(ldA);

    float distance = hypot((pX - gBoard.G_X_mm),(pY - gBoard.G_Y_mm));

    gBoard.moveLinear(distance);

    gBoard.moveAngular(gBoard.normalize_angle(lTheta_rad - gBoard.G_Theta_rad));

}
