#pragma once
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_geometry {

struct Point {
  double x;
  double y;
  double z;
};

struct Pose2D {
  double x;
  double y;
  double theta;
};

struct Quaternion {
  double x;
  double y;
  double z;
  double w;
};

struct Point32 {
  float x;
  float y;
  float z;
  serdes3(x,y,z)
};

struct Vector3 {
  double x;
  double y;
  double z;
  serdes3(x,y,z)
};

struct Accel {
  Vector3 linear;
  Vector3 angular;
};

struct Vector3Stamped {
  maf_std::Header header;
  Vector3 vector;
};

struct Transform {
  Vector3 translation;
  Quaternion rotation;
};

struct PointStamped {
  maf_std::Header header;
  Point point;
};

struct QuaternionStamped {
  maf_std::Header header;
  Quaternion quaternion;
};

struct Polygon {
  std::vector<Point32> points;
};

struct Twist {
  Vector3 linear;
  Vector3 angular;
};

struct Wrench {
  Vector3 force;
  Vector3 torque;
};

struct Inertia {
  double m;
  Vector3 com;
  double ixx;
  double ixy;
  double ixz;
  double iyy;
  double iyz;
  double izz;
};

struct Pose {
  Point position;
  Quaternion orientation;
};

struct PoseArray {
  maf_std::Header header;
  std::vector<Pose> poses;
};

struct PoseWithCovariance {
  Pose pose;
  std::vector<double> covariance;
};

struct TransformStamped {
  maf_std::Header header;
  std::string child_frame_id;
  Transform transform;
};

struct PoseStamped {
  maf_std::Header header;
  Pose pose;
};

struct TwistStamped {
  maf_std::Header header;
  Twist twist;
};

struct PolygonStamped {
  maf_std::Header header;
  Polygon polygon;
};

struct WrenchStamped {
  maf_std::Header header;
  Wrench wrench;
};

struct AccelStamped {
  maf_std::Header header;
  Accel accel;
};

struct TwistWithCovariance {
  Twist twist;
  std::vector<double> covariance;
};

struct AccelWithCovariance {
  Accel accel;
  std::vector<double> covariance;
};

struct InertiaStamped {
  maf_std::Header header;
  Inertia inertia;
};

struct AccelWithCovarianceStamped {
  maf_std::Header header;
  AccelWithCovariance accel;
};

struct PoseWithCovarianceStamped {
  maf_std::Header header;
  PoseWithCovariance pose;
};

struct TwistWithCovarianceStamped {
  maf_std::Header header;
  TwistWithCovariance twist;
};

} // namespace maf_geometry
