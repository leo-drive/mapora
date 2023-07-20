// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#include "mapora/mapora.hpp"

#include <tbb/iterators.h>

#include <Eigen/Geometry>
#include <mapora/point_xyzi.hpp>
#include <mapora/point_xyzit.hpp>
#include <mapora/utils.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <execution>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>


#include "mapora/points_provider_velodyne.hpp"
#include "mapora/transform_provider_sbg.hpp"
#include "mapora/slam/leo_loam/occtree.hpp"

namespace
{
const std::uint32_t QOS_HISTORY_DEPTH = 10;
}

namespace mapora
{
Mapora::Mapora(const rclcpp::NodeOptions & options)
: Node("mapora", options),
  index_pcap_start_{this->declare_parameter("index_pcap_start").get<int64_t>()},
  count_pcaps_to_process_{this->declare_parameter("count_pcaps_to_process").get<int64_t>()},
  count_clouds_to_extract_max_{
    this->declare_parameter("count_clouds_to_extract_max").get<int64_t>()},
  sub_cloud_velodyne_raw_{this->create_subscription<PointCloud2>(
      "velodyne_points_raw_fixed",
      10,
      std::bind(&Mapora::callback_cloud_velodyne_points_raw, this, std::placeholders::_1))}
{
  pub_ptr_cloud_current_ = this->create_publisher<PointCloud2>("cloud_current", 10);
  pub_ptr_cloud_current_trans_ = this->create_publisher<PointCloud2>("cloud_current_trans", 10);
  pub_ptr_path_sbg_ = this->create_publisher<nav_msgs::msg::Path>("path_sbg", 10);

  pub_ptr_poses_imu_ = this->create_publisher<geometry_msgs::msg::PoseArray>("poses_imu", 10);
  pub_ptr_poses_imu_origin_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>("poses_imu_origin", 10);
  pub_ptr_poses_slam_origin_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>("poses_slam_origin", 10);
  pub_ptr_poses_lidar_true_origin_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>("poses_lidar_true_origin", 10);

  {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster(this);
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.transform.rotation.set__w(1.0);
    transform_stamped.header.set__frame_id("map");

    transform_stamped.set__child_frame_id("imu_origin");
    static_broadcaster.sendTransform(transform_stamped);

    transform_stamped.set__child_frame_id("slam_origin");
    static_broadcaster.sendTransform(transform_stamped);

    transform_stamped.set__child_frame_id("velodyne");
    static_broadcaster.sendTransform(transform_stamped);
  }

  transform_provider_sbg_ = std::make_shared<transform_provider::TransformProviderSbg>(
    "/home/mfc/mapora_data/ascii-output.csv");

  transform_provider_sbg_->process();

  // To be filled as callback_cloud_surround_out is called through
  // blocking points_provider_velodyne->process_pcaps_into_clouds operation
  queue_clouds_raw_ =
    std::make_shared<folly::ProducerConsumerQueue<PointCloud2::ConstSharedPtr>>(9999999);
  queue_slam_transforms_ = std::make_shared<
    folly::ProducerConsumerQueue<geometry_msgs::msg::TransformStamped::ConstSharedPtr>>(9999999);
  poses_imu_.clear();

  // this will consume the queue elements to make slam
  slam::leo_loam::MapperOcc::TypeCallbackTransform callback_slam_transform =
    std::bind(&Mapora::callback_slam_transform, this, std::placeholders::_1);
  slam_handler_ = std::make_shared<SlamHandler>(queue_clouds_raw_, callback_slam_transform, *this);

  points_provider_velodyne_ = std::make_shared<points_provider::PointsProviderVelodyne>(
    std::string("/home/mfc/mapora_data/split_vls128_2021-06-14_16_00_00"));
  points_provider_velodyne_->process();


  // This is a blocking operation
  std::function<void(const Points &)> callback =
    std::bind(&Mapora::callback_cloud_surround_out, this, std::placeholders::_1);
  points_provider_velodyne_->process_pcaps_into_clouds(
    callback, index_pcap_start_, count_pcaps_to_process_, count_clouds_to_extract_max_);
  std::cout << "process_pcaps_into_clouds done" << std::endl;

  // For each imu pose, calc the transformation from origin (first pose)
  if (poses_imu_.size() < 2) {
    throw std::runtime_error("Something is wrong, poses_imu_.size() < 2");
  }
  transforms_imu_from_origin_.resize(poses_imu_.size() - 1);

  const auto & pose_origin = poses_imu_.front();
  const auto mat_trans_origin = utils::Utils::pose_to_mat_eigen(pose_origin.pose.pose);
  const auto mat_trans_origin_inv = mat_trans_origin.inverse();
  for (size_t i = 1; i < poses_imu_.size(); ++i) {
    const auto & pose_curr = poses_imu_.at(i);

    const auto mat_trans_curr = utils::Utils::pose_to_mat_eigen(pose_curr.pose.pose);

    Eigen::Affine3d affine_trans(mat_trans_origin_inv * mat_trans_curr);
    //    affine_trans.matrix().topLeftCorner<3, 3>() = mat_trans_curr.topRightCorner<3, 3>();
    //    Eigen::Affine3d affine_trans(mat_trans_curr);

    auto & transform_stamped = transforms_imu_from_origin_.at(i - 1);
    transform_stamped = tf2::eigenToTransform(affine_trans);
    transform_stamped.header.frame_id = "imu_origin";
    transform_stamped.child_frame_id = "imu_pose_" + std::to_string(i);
    transform_stamped.header.stamp = pose_curr.header.stamp;

    Eigen::Vector3d translation(
      pose_curr.pose.pose.position.x,
      pose_curr.pose.pose.position.y,
      pose_curr.pose.pose.position.z);
    //    std::cout << "trans: " << i << ": " << translation.transpose() << std::endl;
    //    std::cout << "trans: " << i << ": " << affine_trans.translation().transpose() <<
    //    std::endl;
  }

  geometry_msgs::msg::PoseArray poses_imu_origin;
  poses_imu_origin.header.frame_id = "map";
  //  poses_imu_origin.header.stamp = get_clock()->now();


  auto transform_to_pose = [](const geometry_msgs::msg::TransformStamped & tr_st) {
      geometry_msgs::msg::Pose pose;
      pose.position.set__x(tr_st.transform.translation.x);
      pose.position.set__y(tr_st.transform.translation.y);
      pose.position.set__z(tr_st.transform.translation.z);
      pose.orientation.set__x(tr_st.transform.rotation.x);
      pose.orientation.set__y(tr_st.transform.rotation.y);
      pose.orientation.set__z(tr_st.transform.rotation.z);
      pose.orientation.set__w(tr_st.transform.rotation.w);
      return pose;
    };
  poses_imu_origin.poses.resize(transforms_imu_from_origin_.size());
  std::transform(
    transforms_imu_from_origin_.begin(),
    transforms_imu_from_origin_.end(),
    poses_imu_origin.poses.begin(),
    transform_to_pose);

  pub_ptr_poses_imu_origin_->publish(poses_imu_origin);


  {
    geometry_msgs::msg::PoseArray poses_imu;
    poses_imu.header.frame_id = "map";
    poses_imu.poses.resize(poses_imu_.size());
    std::transform(
      poses_imu_.cbegin(),
      poses_imu_.cend(),
      poses_imu.poses.begin(),
      [](const geometry_msgs::msg::PoseWithCovarianceStamped & pose_with_covariance_stamped) {
        return pose_with_covariance_stamped.pose.pose;
      });
    pub_ptr_poses_imu_->publish(poses_imu);
  }


  // Wait for slam to finish
  transforms_lidar_slam_.clear();
  transforms_lidar_slam_.reserve(transforms_imu_from_origin_.size());
  size_t count_slam_transforms = 0UL;
  while (true) {
    geometry_msgs::msg::TransformStamped::ConstSharedPtr transform_stamped_ptr_slam;
    while (!queue_slam_transforms_->read(transform_stamped_ptr_slam)) {
      // spin until we get a value
    }
    transforms_lidar_slam_.emplace_back(*transform_stamped_ptr_slam);
    count_slam_transforms++;
    std::cout << "count_slam_transforms: " << count_slam_transforms << std::endl;
    std::cout << "transforms_imu_from_origin_: " << transforms_imu_from_origin_.size() << std::endl;

    if (count_slam_transforms >= transforms_imu_from_origin_.size()) {break;}
  }

  geometry_msgs::msg::PoseArray poses_slam_origin;
  poses_slam_origin.header.frame_id = "slam_origin";
  poses_slam_origin.poses.resize(transforms_lidar_slam_.size());

  std::transform(
    transforms_lidar_slam_.begin(),
    transforms_lidar_slam_.end(),
    poses_slam_origin.poses.begin(),
    transform_to_pose);

  pub_ptr_poses_slam_origin_->publish(poses_slam_origin);

  std::cout << "mapora constructor first poses published." << std::endl;

  // get rotation between 5th transforms
  const size_t ind_sel = 1;
  const auto & trans_stamped_imu = transforms_imu_from_origin_.at(ind_sel);
  const auto & trans_stamped_slam = transforms_lidar_slam_.at(ind_sel);

  // imu * trans = slam
  // trans = imu.inv() * slam
  auto print_for_numpy = [](const Eigen::Matrix4d & mat) {
      std::cout << "[";
      for (int i = 0; i < 4; ++i) {
        std::cout << "[";
        for (int j = 0; j < 4; ++j) {
          std::cout << mat(i, j) << ", ";
        }
        std::cout << "], ";
      }
      std::cout << "]" << std::endl;
    };
  Eigen::Affine3d affine_eigen_imu = tf2::transformToEigen(trans_stamped_imu);
  std::cout << "affine_eigen_imu: " << affine_eigen_imu.matrix().transpose() << std::endl;
  print_for_numpy(affine_eigen_imu.matrix().transpose());
  Eigen::Affine3d affine_eigen_slam = tf2::transformToEigen(trans_stamped_slam);
  std::cout << "affine_eigen_slam: " << affine_eigen_slam.matrix().transpose() << std::endl;
  print_for_numpy(affine_eigen_slam.matrix().transpose());
  Eigen::Affine3d affine_trans = affine_eigen_imu.inverse() * affine_eigen_slam;

  std::vector<geometry_msgs::msg::TransformStamped> transforms_lidar_true;
  transforms_lidar_true.reserve(poses_imu_origin.poses.size());
  {
    geometry_msgs::msg::PoseArray poses_lidar_true_origin;
    poses_lidar_true_origin.header.frame_id = "map";
    poses_lidar_true_origin.poses.resize(poses_imu_origin.poses.size());

    for (size_t i = 0; i < transforms_imu_from_origin_.size(); ++i) {
      const auto & transform_imu = transforms_imu_from_origin_.at(i);
      Eigen::Affine3d affine_eigen_imu = tf2::transformToEigen(transform_imu);
      affine_eigen_imu = affine_trans * affine_eigen_imu;
      geometry_msgs::msg::TransformStamped trans_lidar_true =
        tf2::eigenToTransform(affine_eigen_imu);
      transforms_lidar_true.push_back(trans_lidar_true);
      poses_lidar_true_origin.poses.at(i) = transform_to_pose(trans_lidar_true);
    }

    pub_ptr_poses_lidar_true_origin_->publish(poses_lidar_true_origin);
  }


  assert(transforms_lidar_true.size() == vector_clouds_raw_.size() - 1);

  using Occtree = slam::leo_loam::Occtree;
  Occtree::Ptr occtree_ptr = std::make_shared<Occtree>(0.2f);
  for (size_t i = 0; i < vector_clouds_raw_.size(); ++i) {
    const PointCloud2::SharedPtr & cloud_ptr_raw = vector_clouds_raw_.at(i);
    Eigen::Matrix4d mat_trans_imu;
    if (i == 0) {
      mat_trans_imu = Eigen::Matrix4d::Identity();
    } else {
      const auto & transform = transforms_lidar_true.at(i - 1);
      auto iso_imu = tf2::transformToEigen(transform);
      mat_trans_imu = iso_imu.matrix();
    }

    using CloudView = point_cloud_msg_wrapper::PointCloud2View<point_types::PointXYZI>;
    CloudView cloud_view(*cloud_ptr_raw);

    using CloudModifier = point_cloud_msg_wrapper::PointCloud2Modifier<point_types::PointXYZI>;
    PointCloud2::SharedPtr cloud_ptr_trans = std::make_shared<PointCloud2>();
    CloudModifier cloud_modifier_trans(*cloud_ptr_trans, "map");
    cloud_modifier_trans.resize(cloud_view.size());
    std::transform(
      cloud_view.cbegin(),
      cloud_view.cend(),
      cloud_modifier_trans.begin(),
      [&mat_trans_imu](const point_types::PointXYZI & point) {
        Eigen::Vector4d vec_point(point.x, point.y, point.z, 1.0);
        Eigen::Vector4d vec_point_trans(mat_trans_imu * vec_point);
        return point_types::PointXYZI{
          static_cast<float>(vec_point_trans.x()),
          static_cast<float>(vec_point_trans.y()),
          static_cast<float>(vec_point_trans.z()),
          point.intensity};
      });

    std::for_each(
      cloud_modifier_trans.cbegin(),
      cloud_modifier_trans.cend(),
      [&occtree_ptr](const point_types::PointXYZI & point) {
        PclUtils::PointPcl point_pcl = PclUtils::point_to_point_pcl(point);
        if (!occtree_ptr->octree->isVoxelOccupiedAtPoint(point_pcl)) {
          occtree_ptr->AddPoint(point_pcl);
        }
      });

    cloud_ptr_trans = PclUtils::cloud_pcl_to_pointcloud2(occtree_ptr->cloud, "map");
    pub_ptr_cloud_current_trans_->publish(*cloud_ptr_trans);
  }


  std::cout << "mapora constructor has ended." << std::endl;
}

void Mapora::process()
{
  std::cout << "Mapora begins." << std::endl;

  auto thing_to_cloud = [](const Points & points_bad, const std::string & frame_id) {
      using CloudModifier = point_cloud_msg_wrapper::PointCloud2Modifier<point_types::PointXYZI>;
      PointCloud2::SharedPtr cloud_ptr_current = std::make_shared<PointCloud2>();
      CloudModifier cloud_modifier_current(*cloud_ptr_current, frame_id);
      cloud_modifier_current.resize(points_bad.size());
      std::transform(
        std::execution::par,
        points_bad.cbegin(),
        points_bad.cend(),
        cloud_modifier_current.begin(),
        [](const points_provider::PointsProviderVelodyne::Point & point_bad) {
          return point_types::PointXYZI{
            point_bad.x, point_bad.y, point_bad.z, static_cast<float>(point_bad.intensity)};
        });
      return cloud_ptr_current;
    };

  nav_msgs::msg::Path path_sbg;
  path_sbg.header.frame_id = "map";

  // int counter_path = 0;
  points_provider::PointsProviderVelodyne::Points cloud_all;

  // section 2
  // auto start = high_resolution_clock::now();
  //  for (auto & cloud : clouds) {
  //    //    points_provider_velodyne->downsample_cloud(cloud);
  //
  //    points_provider::PointsProviderVelodyne::Points cloud_trans;
  //    cloud_trans.resize(cloud.size());
  //
  //    /*
  //    for (auto & point : cloud)
  //    {
  //        transform_provider::TransformProviderSbg::Pose pose =
  //                transform_provider_sbg->get_pose_at(
  //                        point.stamp_unix_seconds,
  //                        point.stamp_nanoseconds);
  //
  //        const auto & pose_ori = pose.pose_with_covariance.pose.orientation;
  //        std::cout << pose_ori.w<< pose_ori.x << pose_ori.y << pose_ori.z << std::endl;
  //    }
  //    std::cout << "lmao" << std::endl;*/
  //
  //    std::transform(
  //      std::execution::par,
  //      cloud.cbegin(),
  //      cloud.cend(),
  //      cloud_trans.begin(),
  //      [this, &path_sbg](const points_provider::PointsProviderVelodyne::Point & point) {
  //        points_provider::PointsProviderVelodyne::Point point_trans;
  //        transform_provider::TransformProviderSbg::Pose pose =
  //        transform_provider_sbg->get_pose_at(point.stamp_unix_seconds, point.stamp_nanoseconds);
  //
  //
  //        //        std::cout << "affine: " <<
  //        //                  "point_trans.stamp_unix_seconds: " << point.stamp_unix_seconds <<
  //        //                  "point_trans.stamp_nanoseconds: " << point.stamp_nanoseconds <<
  //        //                  std::endl;
  //
  //        geometry_msgs::msg::PoseStamped pose_stamped;
  //        pose_stamped.pose = pose.pose_with_covariance.pose;
  //        pose_stamped.header.frame_id = path_sbg.header.frame_id;
  //
  //        const auto & pose_ori = pose.pose_with_covariance.pose.orientation;
  //        Eigen::Quaterniond quat(pose_ori.w, pose_ori.x, pose_ori.y, pose_ori.z);
  //        Eigen::Affine3d affine(Eigen::Affine3d::Identity());
  //
  //        affine.matrix().topLeftCorner<3, 3>() =
  //        Eigen::AngleAxisd(utils::Utils::deg_to_rad(this->deg_z), Eigen::Vector3d::UnitZ())
  //        .toRotationMatrix() *
  //        Eigen::AngleAxisd(utils::Utils::deg_to_rad(this->deg_y), Eigen::Vector3d::UnitY())
  //        .toRotationMatrix() *
  //        Eigen::AngleAxisd(utils::Utils::deg_to_rad(this->deg_x), Eigen::Vector3d::UnitX())
  //        .toRotationMatrix() *
  //        quat.toRotationMatrix().inverse();
  //        auto & pose_pos = pose_stamped.pose.position;
  //        pose_pos.x += 750;
  //        pose_pos.y += -280;
  //        pose_pos.z += 57;
  //        affine.matrix().topRightCorner<3, 1>() << pose_pos.x, pose_pos.y, pose_pos.z;
  //        /*
  //            if (counter_path % 600 == 0) {
  //                path_sbg.poses.push_back(pose_stamped);
  //            }
  //            counter_path++;*/
  //        //        affine.rotate(Eigen::AngleAxisd(utils::Utils::deg_to_rad(30),
  //        //        Eigen::Vector3d::UnitZ()));
  //
  //        Eigen::Vector4d vec_point_in(point.x, point.y, point.z, 1.0);
  //        //        std::cout << "translation: " << affine.translation() << std::endl;
  //        //        std::cout << "affine: " <<
  //        //          "pose_pos.x: " << pose_pos.x <<
  //        //          ", pose_pos.y: " << pose_pos.y <<
  //        //          ", pose_pos.z: " << pose_pos.z <<
  //        //          std::endl;
  //
  //
  //        //        Eigen::Affine3d affine_special(Eigen::Affine3d::Identity());
  //        //        affine_special.matrix().topRightCorner<3, 1>() << counter_path, counter_path,
  //        //        counter_path; Eigen::Vector4d vec_point_special(0, 0, 5, 1); std::cout <<
  //        //        "affine_special.matrix(): " << affine_special.matrix() << std::endl;
  //
  //        //        Eigen::Vector4d vec_point_trans = affine_special.matrix() * vec_point_special;
  //        //        std::cout << "vec_point_trans: " << vec_point_trans << std::endl;
  //        Eigen::Vector4d vec_point_trans = affine.matrix() * vec_point_in;
  //        //        Eigen::Vector4d vec_point_trans = vec_point_in;
  //        point_trans.x = static_cast<float>(vec_point_trans(0));
  //        point_trans.y = static_cast<float>(vec_point_trans(1));
  //        point_trans.z = static_cast<float>(vec_point_trans(2));
  //        point_trans.intensity = point.intensity;
  //        return point_trans;
  //      });
  //
  //    cloud_all.insert(cloud_all.end(), cloud_trans.begin(), cloud_trans.end());
  //    //    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  //    //    auto cloud_ptr_current = thing_to_cloud(cloud_trans, "map");
  //    //    pub_ptr_cloud_current_->publish(*cloud_ptr_current);
  //    pub_ptr_path_sbg_->publish(path_sbg);
  //  }
  // auto stop = high_resolution_clock::now();
  // auto duration = duration_cast<milliseconds>(stop - start);
  // std::cout << "section 2 - " << duration.count() << std::endl;


  auto cloud_ptr_current = thing_to_cloud(cloud_all, "map");
  pub_ptr_cloud_current_->publish(*cloud_ptr_current);
  pub_ptr_path_sbg_->publish(path_sbg);

  std::cout << "Mapora is done." << std::endl;
}

void Mapora::callback_cloud_surround_out(const Mapora::Points & points_surround)
{
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_surround = points_to_cloud(points_surround, "map");
  while (!queue_clouds_raw_->write(cloud_surround)) {
    // spin until the queue has room
  }
  vector_clouds_raw_.push_back(cloud_surround);
  transform_provider::TransformProviderSbg::Pose pose = transform_provider_sbg_->get_pose_at(
    cloud_surround->header.stamp.sec, cloud_surround->header.stamp.nanosec);
  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance_stamped;
  pose_with_covariance_stamped.header.frame_id = "map";
  pose_with_covariance_stamped.header.stamp = cloud_surround->header.stamp;
  pose_with_covariance_stamped.pose = pose.pose_with_covariance;
  poses_imu_.push_back(pose_with_covariance_stamped);

  //  std::cout << "cov: " << std::sqrt(pose_with_covariance_stamped.pose.covariance.back()) <<
  //    std::endl;

  //  cloud_surround->header.stamp.sec = 0;
  //  cloud_surround->header.stamp.nanosec = 0;
  pub_ptr_cloud_current_->publish(*cloud_surround);
}

sensor_msgs::msg::PointCloud2::SharedPtr Mapora::points_to_cloud(
  const Mapora::Points & points_bad, const std::string & frame_id)
{
  using CloudModifier = point_cloud_msg_wrapper::PointCloud2Modifier<point_types::PointXYZI>;
  PointCloud2::SharedPtr cloud_ptr_current = std::make_shared<PointCloud2>();
  CloudModifier cloud_modifier_current(*cloud_ptr_current, frame_id);
  cloud_modifier_current.resize(points_bad.size());
  std::transform(
    std::execution::par,
    points_bad.cbegin(),
    points_bad.cend(),
    cloud_modifier_current.begin(),
    [](const points_provider::PointsProviderVelodyne::Point & point_bad) {
      return point_types::PointXYZI{
        point_bad.x, point_bad.y, point_bad.z, static_cast<float>(point_bad.intensity)};
    });
  cloud_ptr_current->header.stamp.sec = static_cast<int32_t>(points_bad.front().stamp_unix_seconds);
  cloud_ptr_current->header.stamp.nanosec = points_bad.front().stamp_nanoseconds;
  return cloud_ptr_current;
}

void Mapora::callback_cloud_velodyne_points_raw(const PointCloud2::ConstSharedPtr msg_in)
{
  //  while (!queue_clouds_raw_->write(msg_in)) {
  //    // spin until the queue has room
  //  }
}

void Mapora::callback_slam_transform(
  const geometry_msgs::msg::TransformStamped::ConstSharedPtr & transform_slam)
{
  while (!queue_slam_transforms_->write(transform_slam)) {
    // spin until the queue has room
  }
}


}  // namespace mapora
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mapora::Mapora)
