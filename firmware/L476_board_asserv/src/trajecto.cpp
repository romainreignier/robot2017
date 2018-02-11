#include "trajecto.h"
#include "Board.h"
#include <cmath>
#include <cstdlib>
#include <vector>

#define UNDEFINED_THETA 999

struct Coord{
    float_t X;
    float_t Y;
    float_t Theta;
    Coord(float_t pX,float_t pY,float_t pTheta = UNDEFINED_THETA){
        X=pX;
        Y=pX;
        Theta=pTheta;
    }
};

void
MoveSquare(){
    gBoard.SetInitPosition(0,0,90);

    GoingToPoint(0,100);
    GoingToPoint(-100,100);
    GoingToPoint(-100,0);
    GoingToPoint(0,0);
    //ComplexMove();
}

void
ComplexMove(){
    float distance;
    float ldA;
    float directionTheta;
    gBoard.SetInitPosition(135,117.5,90);

//          GoingToPoint(211,145);
//    distance = hypot((211 - gBoard.G_X_mm),(145 - gBoard.G_Y_mm));
//    chprintf(dbg, "distance: %f\r\n", distance);
    //      GoingToPoint(292,200);
    //      GoingToPoint(354,275);
    //      GoingToPoint(402,392);
    //      GoingToPoint(410,467.5,90);

    std::vector <Coord> listCoord;
    listCoord.push_back(Coord(211,145,UNDEFINED_THETA));
//    listCoord.push_back(Coord(292,200,UNDEFINED_THETA));
//    listCoord.push_back(Coord(354,275,UNDEFINED_THETA));
//    listCoord.push_back(Coord(402,392,UNDEFINED_THETA));
//    listCoord.push_back(Coord(410,467.5,90));
    gBoard.finish=false;
    for(int8_t i=0 ; i < listCoord.size() ; ++i)
    {
        directionTheta = atan2(listCoord[i].Y - gBoard.G_Y_mm,listCoord[i].X - gBoard.G_X_mm);
        distance = hypot((listCoord[i].X - gBoard.G_X_mm),(listCoord[i].Y - gBoard.G_Y_mm));
        ldA = gBoard.normalize_angle(directionTheta - gBoard.G_Theta_rad);

        gBoard.move(distance,ldA);

        while (distance > 30.0){
            distance = hypot((listCoord[i].X - gBoard.G_X_mm),(listCoord[i].Y - gBoard.G_Y_mm));
            chprintf(dbg, "distance: %f\r\n", distance);
        }

        if(listCoord[i].Theta =! UNDEFINED_THETA){
            gBoard.moveAngular(
                        gBoard.normalize_angle(listCoord[i].Theta * DTOR - gBoard.G_Theta_rad));
        }

    }
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
