#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float32.h>

#include <gnuplot-iostream.h>

#include "Odometry.h"

struct OdomInputs
{
  float leftWheelRadius;
  float rightWheelRadius;
  float wheelsSeparation;
};

struct OdomResults
{
  std::vector<std::pair<double, double>> x;
  std::vector<std::pair<double, double>> y;
  std::vector<std::pair<double, double>> theta;
  std::string plotStr;
};

OdomResults computeOdom(rosbag::View& bagView, const OdomInputs& in)
{
  Odometry odom;
  OdomResults res;

  std::stringstream ss;
  ss << std::fixed << std::setw(5) << std::setprecision(3)
     << "'-' with lines title 'left:" << in.leftWheelRadius
     << " right:" << in.rightWheelRadius << " sep:" << in.wheelsSeparation << "', ";
  res.plotStr = ss.str();

  odom.leftWheelRadius = in.leftWheelRadius;
  odom.rightWheelRadius = in.rightWheelRadius;
  odom.wheelSeparationMM = in.wheelsSeparation;

  odom.reset();
  float currentLeftTicks = 0;
  float currentRightTicks = 0;

  foreach(rosbag::MessageInstance const m, bagView)
  {
    if(m.getTopic() == "/tickg")
    {
      std_msgs::Float32::ConstPtr s = m.instantiate<std_msgs::Float32>();
      if(s == nullptr) continue;
      currentLeftTicks = s->data;
    }
    else if(m.getTopic() == "/tickd")
    {
      std_msgs::Float32::ConstPtr s = m.instantiate<std_msgs::Float32>();
      if(s == nullptr) continue;
      currentRightTicks = s->data;

      const Pose pose = odom.computeOdom(currentLeftTicks, currentRightTicks);
      const double time = m.getTime().toSec();
      res.x.emplace_back(time, pose.x);
      res.y.emplace_back(time, pose.y);
      res.theta.emplace_back(time, pose.theta);
    }
  }

  std::cout << std::fixed << std::setw(5) << std::setprecision(3)
            << "Left: " << odom.leftWheelRadius << " mm Right: " << odom.rightWheelRadius
            << " mm Sep: " << odom.wheelSeparationMM << " mm ";
  std::cout << std::fixed << std::setw(6) << std::setprecision(2)
            << "Final X: " << res.x.back().second << " mm Y: " << res.y.back().second
            << " mm Theta: " << std::setprecision(4) << res.theta.back().second << '\n';

  return res;
}

int main(int argc, char** argv)
{
  std::string bagFilePath;
  if(argc < 2)
  {
    std::cout << "USAGE: " << argv[0] << " <bag_file>\n";
    return -1;
  }

  bagFilePath = argv[1];

  if(!boost::filesystem::exists(bagFilePath))
  {
    std::cout << "The bag " << bagFilePath << " does not exixt\n";
    return -1;
  }

  rosbag::Bag bag;
  bag.open(bagFilePath, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("/tickg"));
  topics.push_back(std::string("/tickd"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::vector<OdomInputs> inputs;
  /*
  inputs.emplace_back(OdomInputs{26.125f, 26.098f, 111.725f});
  inputs.emplace_back(OdomInputs{26.125f, 26.100f, 111.725f});
  inputs.emplace_back(OdomInputs{26.125f, 26.102f, 111.725f});
  inputs.emplace_back(OdomInputs{26.125f, 26.104f, 111.725f});
  inputs.emplace_back(OdomInputs{26.125f, 26.106f, 111.725f});
  */

  /*
  inputs.emplace_back(OdomInputs{26.125f, 26.f, 111.725f});
  inputs.emplace_back(OdomInputs{26.125f, 26.f, 111.6f});
  inputs.emplace_back(OdomInputs{26.125f, 26.f, 111.65f});
  inputs.emplace_back(OdomInputs{26.125f, 26.f, 111.7f});
  inputs.emplace_back(OdomInputs{26.125f, 26.f, 111.75f});
  inputs.emplace_back(OdomInputs{26.125f, 26.f, 111.8f});
  */

  inputs.emplace_back(OdomInputs{26.120f, 26.100f, 111.725f});
  inputs.emplace_back(OdomInputs{26.125f, 26.100f, 111.725f});
  inputs.emplace_back(OdomInputs{26.130f, 26.100f, 111.725f});
  inputs.emplace_back(OdomInputs{26.120f, 26.102f, 111.725f});
  inputs.emplace_back(OdomInputs{26.125f, 26.102f, 111.725f});
  inputs.emplace_back(OdomInputs{26.130f, 26.102f, 111.725f});

  std::vector<std::pair<OdomInputs, OdomResults>> results;
  for(const auto& input : inputs)
  {
    results.emplace_back(input, computeOdom(view, input));
  }

  bag.close();

  Gnuplot gpX;
  gpX << "set title 'X'\n";
  gpX << "set key left top\n";
  gpX << "plot ";
  Gnuplot gpY;
  gpY << "set title 'Y'\n";
  gpY << "set key left top\n";
  gpY << "plot ";
  Gnuplot gpTheta;
  gpTheta << "set title 'Theta'\n";
  gpTheta << "set key left top\n";
  gpTheta << "plot ";

  for(const auto& res : results)
  {
    gpX << res.second.plotStr;
    gpY << res.second.plotStr;
    gpTheta << res.second.plotStr;
  }

  gpX << '\n';
  gpY << '\n';
  gpTheta << '\n';

  for(const auto& res : results)
  {
    gpX.send1d(res.second.x);
    gpY.send1d(res.second.y);
    gpTheta.send1d(res.second.theta);
  }

  return 0;
}
