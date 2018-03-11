#include "RosSerial.h"

#include "Board.h"
#include "trajecto.h"

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <snd_msgs/Encoders.h>
#include <snd_msgs/GlobalStatus.h>
#include <snd_msgs/Feedback.h>
#include <geometry_msgs/Pose2D.h>
#include <snd_msgs/Position2D.h>


static ros::NodeHandle s_NodeHandle;

void CallbackDeplacement(const geometry_msgs::Pose2D & msg)
{
    GoingToPoint(msg.x,msg.y);
}


void CopyDataOnMessage_Encodeurs(snd_msgs::Encoders & pEncodeurs){
    pEncodeurs.header.stamp         = s_NodeHandle.now();

    pEncodeurs.left                 = gBoard.tickg;
    pEncodeurs.right                = gBoard.tickr;
}


void CopyDataOnMessage_GlobalStatus(snd_msgs::GlobalStatus & pGlobalStatus){
    pGlobalStatus.header.stamp      = s_NodeHandle.now();

    pGlobalStatus.starter           = gBoard.starter.read();
    pGlobalStatus.starting_side     = gBoard.startingSide.read();

    pGlobalStatus.position.x        = gBoard.G_X_mm;
    pGlobalStatus.position.y        = gBoard.G_Y_mm;
    pGlobalStatus.position.theta    = gBoard.G_Theta_rad;

    pGlobalStatus.speed             = gBoard.linear_speed;
}


void CopyDataOnMessage_Feedback(snd_msgs::Feedback & pFeedback){
    pFeedback.header.stamp          = s_NodeHandle.now();

    pFeedback.consigne_distance     = gBoard.consigneDistance;
    pFeedback.consigne_angle        = gBoard.consigneAngle;
    pFeedback.mesure_distance       = gBoard.mesureDistance;
    pFeedback.mesure_angle          = gBoard.mesureAngle;
    pFeedback.erreur_distance       = gBoard.erreurDistance;
    pFeedback.erreur_angle          = gBoard.erreurAngle;
    pFeedback.cible_distance        = gBoard.cibleDistance;
    pFeedback.cible_angle           = gBoard.cibleAngle;
    pFeedback.left_speed            = gBoard.leftSpeed;
    pFeedback.right_speed           = gBoard.rightSpeed;
    pFeedback.left_pwm              = gBoard.leftPwm;
    pFeedback.right_pwm             = gBoard.rightPwm;
    pFeedback.smooth_rotation       = gBoard.smoothRotation;
}

void CopyDataOnMessage_RealTimePosition(snd_msgs::Position2D & pRealTimePosition){
    pRealTimePosition.header.stamp      = s_NodeHandle.now();

    pRealTimePosition.x             = gBoard.G_X_mm;
    pRealTimePosition.y             = gBoard.G_Y_mm;
    pRealTimePosition.theta         = gBoard.G_Theta_rad;
}

THD_WORKING_AREA(waThreadRosserial, 2048);
THD_FUNCTION(ThreadRosserial, arg)
{
  chRegSetThreadName("rosserial");

  (void)arg;

  int _modulo           = 0;
  systime_t time        = chVTGetSystemTimeX();
  constexpr systime_t   kPublishPeriodMs{10};

  // ROS
  //ros::NodeHandle_<ChibiOSHardware,25,25,512,512> nh;
  // Messages
  snd_msgs::Encoders        msg_Encodeurs;
  snd_msgs::GlobalStatus    msg_GlobalStatus;
  snd_msgs::Feedback        msg_Feedback;
  snd_msgs::Position2D      msg_RealTimePosition;

  // Publishers
  ros::Publisher msg_Encodeurs_pub          ("encodeurs",           &msg_Encodeurs);
  ros::Publisher msg_GlobalStatus_pub       ("global_status",       &msg_GlobalStatus);
  ros::Publisher msg_Feedback_pub           ("feedback",            &msg_Feedback);
  ros::Publisher msg_RealTimePosition_pub   ("realtime_position",   &msg_RealTimePosition);

  // Subscribers
  ros::Subscriber <geometry_msgs::Pose2D>
          consigne_deplacement_sub("consigne_deplacement", CallbackDeplacement);

  //Init Subscribers
  s_NodeHandle.subscribe(consigne_deplacement_sub);

  // Init Node Handle
  s_NodeHandle.getHardware()->setDriver(&SD1);
  s_NodeHandle.initNode();

  // Init Publishers
  s_NodeHandle.advertise(msg_Encodeurs_pub);
  s_NodeHandle.advertise(msg_GlobalStatus_pub);
  s_NodeHandle.advertise(msg_Feedback_pub);

  while(true)
  {
    time += MS2ST(kPublishPeriodMs);

    if(_modulo % 2)// toutes les 20MS
    {
        /*** Copy Data ***/
         CopyDataOnMessage_Feedback(msg_Feedback);
         CopyDataOnMessage_RealTimePosition(msg_RealTimePosition);

         /*** Publish ***/
         msg_Feedback_pub.publish(&msg_Feedback);
         msg_RealTimePosition_pub.publish((&msg_RealTimePosition));
    }

    if(_modulo % 2)// toutes les 500MS
    {
       /*** Copy Data ***/
        CopyDataOnMessage_Encodeurs(msg_Encodeurs);
        CopyDataOnMessage_GlobalStatus(msg_GlobalStatus);

        /*** Publish ***/
        msg_Encodeurs_pub.publish(&msg_Encodeurs);
        msg_GlobalStatus_pub.publish(&msg_GlobalStatus);
    }

    if(_modulo >= 1000)//Reset if > 10sec
        _modulo=0;
    else
        _modulo++;
    // Spin
    s_NodeHandle.spinOnce();
    chThdSleepUntil(time);
  }
}
