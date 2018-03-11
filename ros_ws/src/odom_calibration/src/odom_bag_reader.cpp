#include <iostream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#define foreach BOOST_FOREACH

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float32.h>

#include <gnuplot-iostream.h>

#include "Odometry.h"

namespace po = boost::program_options;

int main(int argc, char** argv)
{
  Odometry odom;
  std::string bagFilePath;
  bool mustPlot = false;

  // Declare the supported options.
  po::options_description desc("Allowed options");
  // clang-format off
  desc.add_options()
    ("help", "produce help message")
    ("bag", po::value<std::string>(&bagFilePath), "Bag file to process")
    ("left,l", po::value<float>(&odom.leftWheelRadius)->default_value(odom.leftWheelRadius), "Left wheel radius in mm")
    ("right,r", po::value<float>(&odom.rightWheelRadius)->default_value(odom.rightWheelRadius), "Right wheel radius in mm")
    ("separation,s", po::value<float>(&odom.wheelSeparationMM)->default_value(odom.wheelSeparationMM), "Wheels separation in mm")
    ("plot,p", po::bool_switch(&mustPlot)->default_value(false), "Plot the pose");
  // clang-format on
  po::positional_options_description p;
  p.add("bag", -1);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
  po::notify(vm);

  if(vm.count("help"))
  {
    std::cout << desc << '\n';
    return 1;
  }

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

  // odom.rightWheelRadius = 26.1f;
  std::cout << "[Odom] Left wheel rad: " << odom.leftWheelRadius
            << " mm Right wheel radius: " << odom.rightWheelRadius
            << " mm Wheels separation: " << odom.wheelSeparationMM << " mm\n";
  odom.reset();
  float currentLeftTicks = 0;
  float currentRightTicks = 0;
  std::vector<std::pair<double, double>> xPts;
  std::vector<std::pair<double, double>> yPts;
  std::vector<std::pair<double, double>> thetaPts;

  foreach(rosbag::MessageInstance const m, view)
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
      xPts.emplace_back(time, pose.x);
      yPts.emplace_back(time, pose.y);
      thetaPts.emplace_back(time, pose.theta);
    }
  }

  bag.close();

  std::cout << xPts.size() << " poses computed\n";
  std::cout << "Final Pose: X: " << xPts.back().second << " mm Y: " << yPts.back().second
            << " mm Theta: " << thetaPts.back().second << '\n';

  if(mustPlot)
  {
    Gnuplot gpXY;
    gpXY << "plot '-' with lines title 'X', '-' with lines title 'Y'\n";
    gpXY.send1d(xPts);
    gpXY.send1d(yPts);

    Gnuplot gpTheta;
    gpTheta << "plot '-' with lines title 'Theta'\n";
    gpTheta.send1d(thetaPts);
  }

  return 0;
}
