// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__SLAM__LEO_LOAM__INIT_TRANS_HELPER_HPP_
#define MAPORA__SLAM__LEO_LOAM__INIT_TRANS_HELPER_HPP_

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

// #include "Helper.h"
#include "mapora/pcl_point_types.hpp"

namespace mapora
{
namespace slam
{
namespace leo_loam
{
class InitTransHelper
{
public:
  using Ptr = std::shared_ptr<InitTransHelper>;
  using Point = pcl_point_types::PointPcl;
  using Cloud = pcl_point_types::CloudPcl;

  bool Compute(const std::vector<Eigen::VectorXd> & Alines, Eigen::VectorXd & transform);

  void NeighborSelectionEdge(
    int closestIndex,
    Cloud::Ptr cloudEdgeLast,
    std::vector<int> pointSearchInd,
    std::vector<Point> & edgeNeighbors) const;

  void NeighborSelectionFlat(
    int closestIndex,
    const Cloud::ConstPtr & cloudPlanarLast,
    std::vector<int> pointSearchInd,
    std::vector<Point> & flatNeighbors) const;

  double PointToAngleVertical(const Point & point) const;

  double RadToDeg(double angle_rad) const;

  void PointInfo(Point point, double & pointVAngle, double & pointHAngle, int & pointLaseID) const;

  bool PointToLine(
    Eigen::VectorXd & Aline,
    const Eigen::VectorXd & T,
    Point currPoint,
    Point pointSel,
    const std::vector<Point> & edgeNeighbors,
    int iterCount) const;

  bool PointToPlane(
    Eigen::VectorXd & Aline,
    const Eigen::VectorXd & T,
    Point currPoint,
    Point pointSel,
    const std::vector<Point> & flatNeighbors,
    int iterCount) const;

  void MakeTriangle(
    Eigen::Vector3d & A,
    Eigen::Vector3d & B,
    Eigen::Vector3d & C,
    Point pointSel,
    std::vector<Point> edgeNeighbors) const;

  Eigen::MatrixXd Jacobian(
    double rx, double ry, double rz, double tx, double ty, double tz, double X, double Y, double Z)
  const;

  bool RMSE(Eigen::VectorXd X) const;

private:
};

}  // namespace leo_loam
}  // namespace slam
}  // namespace mapora

#endif  // MAPORA__SLAM__LEO_LOAM__INIT_TRANS_HELPER_HPP_
