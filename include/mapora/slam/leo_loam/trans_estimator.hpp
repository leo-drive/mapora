// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__SLAM__LEO_LOAM__TRANS_ESTIMATOR_HPP_
#define MAPORA__SLAM__LEO_LOAM__TRANS_ESTIMATOR_HPP_

#include <Eigen/Dense>

#include <vector>

#include "mapora/pcl_point_types.hpp"

namespace mapora
{
namespace slam
{
namespace leo_loam
{
class TransEstimator
{
public:
  using Point = pcl_point_types::PointPcl;
  using Cloud = pcl_point_types::CloudPcl;

  const Eigen::VectorXd & getTransformation() const;

  void setTransformation(const Eigen::VectorXd & T);

  bool Compute(const std::vector<Eigen::VectorXd> & Alines, Eigen::VectorXd & X);

  bool coeffsCalculator(
    const Point & pointOri, Cloud::ConstPtr neighbors, bool isEdge, Eigen::VectorXd & Aline) const;

  bool point2Line(
    const Eigen::Vector3d & pointSel,
    const Point & pointOri,
    Cloud::ConstPtr neighbors,
    Eigen::VectorXd & Aline,
    const Eigen::Vector3d & RPY) const;

  bool point2Plane(
    const Eigen::Vector3d & newPoint,
    const Point & interestPoint,
    Cloud::ConstPtr neighbors,
    Eigen::VectorXd & Aline,
    const Eigen::Vector3d & RPY) const;

  bool distributionOfNeighbors(
    Cloud::ConstPtr neighbors,
    double & eigenValue,
    Eigen::Vector3d & eigenVector,
    Eigen::Vector3d & centerPoint) const;

  bool isWellDistributed(const Eigen::Vector3d & eigenValue, int & idx) const;

  Eigen::Matrix3d Covarience(Cloud::ConstPtr neighbors, Eigen::Vector3d & centerPoint) const;

  void makeTriangle(
    const Eigen::Vector3d & newtPoint,
    const Eigen::Vector3d & leftPoint,
    const Eigen::Vector3d & righttPoint,
    Eigen::Vector3d & A,
    Eigen::Vector3d & B,
    Eigen::Vector3d & C) const;

  Eigen::Vector3d jacobian(
    const Point & pointOri,
    const Eigen::Vector3d & RPY,
    const Eigen::Vector3d & Translations) const;

  bool pointsLieOnPlane(Cloud::ConstPtr neighbors, Eigen::Vector4d & planeCoeff) const;

  bool RMSE(const Eigen::VectorXd & X);

  Eigen::Vector3d pointAssociateToMap(const Point & pi) const;

  Point pointAssociateToMapPPP(Point pi);

private:
  Eigen::VectorXd Transformation;
};

}  // namespace leo_loam
}  // namespace slam
}  // namespace mapora

#endif  // MAPORA__SLAM__LEO_LOAM__TRANS_ESTIMATOR_HPP_
