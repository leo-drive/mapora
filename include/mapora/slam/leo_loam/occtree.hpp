// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__SLAM__LEO_LOAM__OCCTREE_HPP_
#define MAPORA__SLAM__LEO_LOAM__OCCTREE_HPP_

#include <pcl/octree/octree_search.h>

#include <execution>
#include <vector>
#include <memory>

#include "mapora/pcl_point_types.hpp"


namespace mapora
{
namespace slam
{
namespace leo_loam
{
class Occtree
{
public:
  using Point = pcl_point_types::PointPcl;
  using Cloud = pcl_point_types::CloudPcl;
  using Octree = pcl::octree::OctreePointCloudSearch<Point>;

  Cloud::Ptr cloud;
  Octree::Ptr octree;
  float resolution;
  typedef std::shared_ptr<Occtree> Ptr;

  explicit Occtree(float res)
  {
    resolution = res;
    cloud.reset(new Cloud);
    octree.reset(new Octree(res));
    octree->setInputCloud(cloud);
    octree->addPointsFromInputCloud();
  }

  void AddPointsFromCloud(const Cloud::ConstPtr & cloudIn)
  {
    for (const auto & point : cloudIn->points) {AddPoint(point);}
  }

  void AddPointsFromCloudIfVoxelEmpty(const Cloud::ConstPtr & cloudIn)
  {
    for (const auto & point : cloudIn->points) {AddPointIfVoxelEmpty(point);}
  }

  void AddPoint(const Point & point) {octree->addPointToCloud(point, cloud);}

  void AddPointIfVoxelEmpty(const Point & point)
  {
    if (!octree->isVoxelOccupiedAtPoint(point)) {AddPoint(point);}
  }

  Cloud::Ptr Box(Point pt_search, float width) const
  {
    float box_radius = width / 2;
    Eigen::Vector3f pt_min(
      pt_search.x - box_radius, pt_search.y - box_radius, pt_search.z - box_radius);
    Eigen::Vector3f pt_max(
      pt_search.x + box_radius, pt_search.y + box_radius, pt_search.z + box_radius);

    Cloud::Ptr cloud_box(new Cloud);
    std::vector<int> k_indices;

    if (octree->boxSearch(pt_min, pt_max, k_indices) > 0) {
      cloud_box->points.resize(k_indices.size());

      std::transform(
        std::execution::par,
        k_indices.begin(),
        k_indices.end(),
        cloud_box->points.begin(),
        [this](const size_t & index) {return cloud->points[index];});

      //      for (const int &k_indice : k_indices)
      //        cloud_box->points.push_back(cloud->points[k_indice]);
    }
    return cloud_box;
  }
};

}  // namespace leo_loam
}  // namespace slam
}  // namespace mapora
#endif  // MAPORA__SLAM__LEO_LOAM__OCCTREE_HPP_
