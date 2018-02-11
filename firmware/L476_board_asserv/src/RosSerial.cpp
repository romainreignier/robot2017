#include "RosSerial.h"

#include "Board.h"
#include "trajecto.h"

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose2D.h>




void CallbackDeplacement(const geometry_msgs::Pose2D & msg)
{
    GoingToPoint(msg.x,msg.y);
}
THD_WORKING_AREA(waThreadRosserial, 2048);
THD_FUNCTION(ThreadRosserial, arg)
{
  (void)arg;
  chRegSetThreadName("rosserial");

  // ROS
  ros::NodeHandle nh;
  //ros::NodeHandle_<ChibiOSHardware,25,25,512,512> nh;
  // Messages
  std_msgs::Float32 consigne_distance;
  std_msgs::Float32 consigne_angle;
  std_msgs::Float32 mesure_distance;
  std_msgs::Float32 mesure_angle;
  std_msgs::Float32 erreur_distance;
  std_msgs::Float32 erreur_angle;
  std_msgs::Float32 cible_distance;
  std_msgs::Float32 cible_angle;
  std_msgs::Float32 left_speed;
  std_msgs::Float32 right_speed;
  std_msgs::Float32 smoothRotation;
  std_msgs::Float32 linear_speed;
  std_msgs::Int16 left_pwm;
  std_msgs::Int16 right_pwm;
  // Publishers
  ros::Publisher consigne_distance_pub("consigne_distance", &consigne_distance);
  ros::Publisher consigne_angle_pub("consigne_angle", &consigne_angle);
  ros::Publisher mesure_distance_pub("mesure_distance", &mesure_distance);
  ros::Publisher mesure_angle_pub("mesure_angle", &mesure_angle);
  ros::Publisher erreur_distance_pub("erreur_distance", &erreur_distance);
  ros::Publisher erreur_angle_pub("erreur_angle", &erreur_angle);
  ros::Publisher cible_distance_pub("cible_distance", &cible_distance);
  ros::Publisher cible_angle_pub("cible_angle", &cible_angle);
  ros::Publisher left_speed_pub("left_speed", &left_speed);
  ros::Publisher right_speed_pub("right_speed", &right_speed);
  ros::Publisher right_pwm_pub("right_pwm", &right_pwm);
  ros::Publisher left_pwm_pub("left_pwm", &left_pwm);
  ros::Publisher smooth_rotation_pub("smooth_rotation", &smoothRotation);
  ros::Publisher linear_speed_pub("linear_speed", &linear_speed);


  // Subscribers
  ros::Subscriber <geometry_msgs::Pose2D>
          consigne_deplacement_sub("consigne_deplacement", CallbackDeplacement);

  //Init Subscribers
  nh.subscribe(consigne_deplacement_sub);

  // Init Node Handle
  nh.getHardware()->setDriver(&SD1);
  nh.initNode();
  // Init Publishers
  nh.advertise(consigne_distance_pub);
  nh.advertise(consigne_angle_pub);
  nh.advertise(mesure_distance_pub);
  nh.advertise(mesure_angle_pub);
  nh.advertise(erreur_distance_pub);
  nh.advertise(erreur_angle_pub);
  nh.advertise(cible_distance_pub);
  nh.advertise(cible_angle_pub);
  nh.advertise(left_speed_pub);
  nh.advertise(right_speed_pub);
  nh.advertise(right_pwm_pub);
  nh.advertise(left_pwm_pub);
  nh.advertise(smooth_rotation_pub);
  nh.advertise(linear_speed_pub);

  systime_t time = chVTGetSystemTimeX();
  constexpr systime_t kPublishPeriodMs{50};
  while(true)
  {
    time += MS2ST(kPublishPeriodMs);

        // Copy the values
        consigne_distance.data = gBoard.consigneDistance;
        consigne_angle.data = gBoard.consigneAngle;
        mesure_distance.data = gBoard.mesureDistance;
        mesure_angle.data = gBoard.mesureAngle;
        erreur_distance.data = gBoard.erreurDistance;
        erreur_angle.data = gBoard.erreurAngle;
        cible_distance.data = gBoard.cibleDistance;
        cible_angle.data = gBoard.cibleAngle;
        left_speed.data = gBoard.leftSpeed;
        right_speed.data = gBoard.rightSpeed;
        left_pwm.data = gBoard.leftPwm;
        right_pwm.data = gBoard.rightPwm;
        smoothRotation.data = gBoard.smoothRotation;
        linear_speed.data = gBoard.linear_speed;

        // Publish the data
        consigne_distance_pub.publish(&consigne_distance);
        consigne_angle_pub.publish(&consigne_angle);
        mesure_distance_pub.publish(&mesure_distance);
        mesure_angle_pub.publish(&mesure_angle);
        erreur_distance_pub.publish(&erreur_distance);
        erreur_angle_pub.publish(&erreur_angle);
        cible_distance_pub.publish(&cible_distance);
        cible_angle_pub.publish(&cible_angle);
        left_speed_pub.publish(&left_speed);
        right_speed_pub.publish(&right_speed);
        right_pwm_pub.publish(&right_pwm);
        left_pwm_pub.publish(&left_pwm);
        smooth_rotation_pub.publish(&smoothRotation);
        linear_speed_pub.publish(&linear_speed);
    // Spin
    nh.spinOnce();
    chThdSleepUntil(time);
  }
}