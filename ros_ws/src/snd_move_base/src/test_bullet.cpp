#include <iostream>
#include <ostream>

#include <bullet/LinearMath/btTransform.h>
#include <tf2_bullet/tf2_bullet.h>

#include <ros/ros.h>

std::ostream& operator<<(std::ostream& _out, const btTransform& _transform);
std::ostream& operator<<(std::ostream& _out, const btVector3& _vector);

int main(int _argc, char** _argv)
{
  btVector3 vect1{1, 1, 0};
  btQuaternion quat1;
  quat1.setEulerZYX(M_PI / 4, 0, 0);
  btTransform transform1{quat1, vect1};

  btVector3 vect2{4, 4, 0};
  btQuaternion quat2;
  quat2.setEulerZYX(M_PI / 4, 0, 0);
  btTransform transform2{quat2, vect2};

  // btTransform transform21 = transform2 - transform1;

  // std::cout << "From " << transform1 << " to " << transform2 << " = "
  //           << transform21 << "\n";

  float distance12 = vect1.distance(vect2);
  float distance21 = vect2.distance(vect1);
  std::cout << "distance12 = " << distance12 << "\n";
  std::cout << "distance21 = " << distance21 << "\n";

  btScalar dot12 = vect1.dot(vect2);
  btScalar dot21 = vect2.dot(vect1);
  std::cout << "dot12 = " << dot12 << "\n";
  std::cout << "dot21 = " << dot21 << "\n";

  btVector3 vect12 = vect1 - vect2;
  btVector3 vect21 = vect2 - vect1;
  btScalar length12 = vect12.length();
  btScalar length21 = vect21.length();
  std::cout << "length12 = " << length12 << "\n";
  std::cout << "length21 = " << length21 << "\n";

  // const float t = cv::dot(p - v, w - v) / l2;
  // t = ((p - v).dot(wv)) / l2;

  btVector3 vect0{0, 0, 0};

  btScalar t1 = (vect1 - vect2).dot(vect2 - vect0);
  btScalar t2 = (vect2 - vect1).dot(vect1 - vect0);
  std::cout << "avant: " << t1 << "\n";
  std::cout << "apres: " << t2 << "\n";

  btVector3 d{1, 1, 0};
  btVector3 a{5, 1, 0};
  btVector3 r1{4, 1, 0};
  btVector3 r2{6, 1, 0};

  std::cout << "r1: " << (r1 - d).dot(a - d) << "\n";
  std::cout << "r2: " << (r2 - d).dot(a - d) << "\n";

  btTransform tr{btQuaternion{0, 0, 0, 1}, r1};

  btVector3 aInR1 = tr * a;
  std::cout << "a in r1 " << aInR1 << "\n";

  return 0;
}

std::ostream& operator<<(std::ostream& _out, const btTransform& _transform)
{
  btScalar roll, pitch, yaw;
  _transform.getBasis().getEulerZYX(yaw, pitch, roll);
  _out << "(" << _transform.getOrigin().getX() << ", "
       << _transform.getOrigin().getY() << " | " << (yaw * 180 / M_PI);
  return _out;
}

std::ostream& operator<<(std::ostream& _out, const btVector3& _vector)
{
  _out << "(" << _vector.getX() << ", " << _vector.getY() << ")";
  return _out;
}
