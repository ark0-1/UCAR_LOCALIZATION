#ifndef SCAN_TO_MAP_LOCATION_H
#define SCAN_TO_MAP_LOCATION_H

#include <cmath>
#include <algorithm>
#include <vector>
#include <chrono>
#include <iostream>
#include <deque>
#include <iomanip>
#include <limits>
#include <mutex>
#include <numeric>
#include <sstream>
#include <typeinfo>
#include <thread>

// ros
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

// tf2
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"

// pcl
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

class Scan2MapLocation
{
private:
    ros::NodeHandle location_nh;
    ros::NodeHandle private_nh;

    ros::Subscriber laser_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber initial_pose_sub_;

    ros::Publisher removal_pointcloud_publisher_;
    ros::Publisher location_publisher_;
    ros::Publisher localization_state_publisher_;
    ros::Publisher localization_diagnostics_publisher_;
    ros::Publisher debug_predicted_pose_publisher_;
    ros::Publisher debug_final_pose_publisher_;
    ros::Publisher debug_scan_predicted_publisher_;
    ros::Publisher debug_scan_used_publisher_;
    ros::Publisher debug_scan_final_publisher_;
    ros::Timer map_to_odom_publish_timer_;

    geometry_msgs::PoseWithCovarianceStamped location_match; // 定位结果

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    Eigen::Isometry3d base_to_lidar_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d map_to_base_ = Eigen::Isometry3d::Identity();  // map到base的欧式变换矩阵4x4
    Eigen::Isometry3d map_to_lidar_ = Eigen::Isometry3d::Identity(); // map到laser的欧式变换矩阵4x4

    Eigen::Isometry3d match_result_ = Eigen::Isometry3d::Identity();      // icp匹配结果
    Eigen::Isometry3d last_match_result_ = Eigen::Isometry3d::Identity(); // 上一帧icp匹配结果
    Eigen::Isometry3d static_lock_pose_ = Eigen::Isometry3d::Identity();

    // parameters
    bool map_initialized_ = false;
    bool scan_initialized_ = false;
    bool odom_initialized_ = false;

    bool Use_TfTree_Always;
    bool if_debug_;
    bool debug_publish_poses_;
    bool debug_publish_clouds_;
    bool publish_localization_state_ = true;
    bool publish_localization_diagnostics_ = true;
    bool publish_map_to_odom_tf_ = true;
    bool use_tf_initial_pose_ = false;
    bool smooth_map_to_odom_ = true;
    bool enable_initial_global_yaw_search_ = true;
    bool enable_startup_relocalization_ = true;

    std::string odom_frame_;
    std::string base_frame_;
    std::string map_frame_;
    std::string lidar_frame_;
    std::string last_failure_reason_;
    std::string last_icp_risk_flags_;

    // 用于计算匹配结果方差
    Eigen::Vector3d Residual_error_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d Euler_Covariance_;

    std::chrono::steady_clock::time_point start_time_, end_time_;
    std::chrono::steady_clock::time_point tran_start_time_;
    std::chrono::steady_clock::time_point tran_end_time_;
    std::chrono::duration<double> tran_time_used_;

    double match_time_;          // 当前匹配的时间
    double last_match_time_ = 0; // 上一帧匹配的时间
    double scan_time_;           // 当前用于匹配的雷达数据时间
    double pose_jump_risk_threshold_; // 预测位姿与匹配结果偏差超过此值，状态标记为 RISK
    double latest_odom_angular_z_ = 0.0;
    double latest_odom_linear_x_ = 0.0;
    double latest_odom_linear_y_ = 0.0;
    double last_scan_age_ = 0.0;
    double last_odom_front_time_ = 0.0;
    double last_odom_back_time_ = 0.0;
    double last_odom_start_used_time_ = 0.0;
    double last_odom_end_used_time_ = 0.0;
    double last_odom_dt_ = 0.0;
    int last_odom_wait_count_ = 0;
    size_t debug_frame_id_ = 0;
    size_t last_scan_raw_count_ = 0;
    size_t last_scan_valid_count_ = 0;
    size_t last_scan_voxel_count_ = 0;
    size_t last_scan_used_count_ = 0;
    size_t last_removed_count_ = 0;
    double last_icp_score_ = std::numeric_limits<double>::quiet_NaN();
    double last_icp_tran_dist_ = 0.0;
    double last_icp_angle_dist_ = 0.0;
    double last_icp_dx_ = 0.0;
    double last_icp_dy_ = 0.0;
    double last_icp_dyaw_ = 0.0;
    bool last_icp_converged_ = false;
    bool last_icp_low_points_ = false;
    bool last_icp_small_correction_ = false;
    bool last_icp_high_score_ = false;
    bool last_icp_large_angle_ = false;
    double last_odom_start_x_ = 0.0;
    double last_odom_start_y_ = 0.0;
    double last_odom_start_yaw_ = 0.0;
    double last_odom_end_x_ = 0.0;
    double last_odom_end_y_ = 0.0;
    double last_odom_end_yaw_ = 0.0;
    double last_odom_delta_x_ = 0.0;
    double last_odom_delta_y_ = 0.0;
    double last_odom_delta_yaw_ = 0.0;
    bool last_static_lock_applied_ = false;
    bool has_static_lock_pose_ = false;
    bool suppress_static_jitter_ = false;
    bool use_scan_midpoint_time_ = true;
    bool enable_scan_deskew_ = true;
    bool enable_yaw_search_ = true;
    double search_translation_range_ = 0.15;
    double search_translation_step_ = 0.05;
    double search_score_max_distance_ = 0.25;
    double search_score_leaf_size_ = 0.08;
    double local_map_radius_ = 8.0;
    double search_accept_score_threshold_ = 0.34;
    double search_accept_improvement_threshold_ = 0.010;
    double search_accept_improvement_ratio_ = 0.97;
    double covariance_translation_min_ = 0.02;
    double covariance_yaw_min_ = 0.03;
    double adaptive_threshold_initial_ = 0.20;
    double adaptive_threshold_min_motion_ = 0.05;
    double registration_translation_epsilon_ = 1e-3;
    double registration_rotation_epsilon_ = 1e-3;
    double static_linear_speed_threshold_ = 0.03;
    double static_angular_speed_threshold_ = 0.05;
    double static_pose_translation_epsilon_ = 0.02;
    double static_pose_yaw_epsilon_ = 0.03;
    double static_odom_translation_epsilon_ = 0.01;
    double static_odom_yaw_epsilon_ = 0.02;
    double yaw_search_trigger_angular_z_ = 0.12;
    double yaw_search_trigger_odom_yaw_ = 0.015;
    double yaw_search_range_deg_ = 18.0;
    double yaw_search_step_deg_ = 6.0;
    double startup_relocalization_translation_range_ = 0.20;
    double startup_relocalization_translation_step_ = 0.04;
    double startup_relocalization_yaw_range_deg_ = 10.0;
    double startup_relocalization_yaw_step_deg_ = 2.0;
    double startup_relocalization_accept_score_threshold_ = 0.40;
    double startup_relocalization_accept_improvement_threshold_ = 0.002;
    double startup_relocalization_accept_improvement_ratio_ = 0.995;
    double initial_global_yaw_search_range_deg_ = 180.0;
    double initial_global_yaw_search_step_deg_ = 30.0;
    int initial_global_yaw_search_frames_ = 8;
    int startup_relocalization_frames_ = 12;
    int static_lock_warmup_frames_ = 6;
    size_t tracking_frame_count_ = 0;
    bool require_heading_bootstrap_ = false;
    double last_scan_start_time_ = 0.0;
    double last_scan_end_time_ = 0.0;
    double last_scan_duration_ = 0.0;
    double last_scan_point_time_increment_ = 0.0;
    double last_scan_reference_offset_ = 0.0;
    double last_scan_motion_translation_ = 0.0;
    double last_scan_motion_yaw_ = 0.0;
    double last_scan_odom_tail_gap_ = 0.0;
    bool last_scan_odom_coverage_ok_ = false;
    int last_scan_odom_wait_count_ = 0;
    double scan_odom_wait_timeout_ = 0.05;
    bool last_scan_deskew_applied_ = false;
    size_t last_scan_deskew_point_count_ = 0;
    size_t last_scan_deskew_failed_point_count_ = 0;
    bool last_icp_yaw_search_used_ = false;
    size_t last_icp_candidate_count_ = 0;
    double last_icp_selected_yaw_offset_ = 0.0;
    bool last_search_active_ = false;
    bool last_search_applied_ = false;
    size_t last_search_candidate_count_ = 0;
    double last_search_predicted_score_ = std::numeric_limits<double>::quiet_NaN();
    double last_search_dx_ = 0.0;
    double last_search_dy_ = 0.0;
    double last_search_dyaw_ = 0.0;
    double last_search_score_ = std::numeric_limits<double>::quiet_NaN();
    bool last_startup_relocalization_active_ = false;
    double adaptive_model_sse_ = 0.04;
    int adaptive_model_samples_ = 1;
    double last_registration_sigma_ = 0.20;
    double last_registration_max_correspondence_ = 0.60;
    size_t last_registration_correspondence_count_ = 0;

    double ObstacleRemoval_Distance_Max; // 如果雷达点云中点在地图点云最近点大于此值，就认为该点为障碍点

    double VoxelGridRemoval_LeafSize; // 体素滤波的边长
    bool enable_outlier_removal_ = true;
    int outlier_mean_k_ = 16;
    double outlier_stddev_mul_ = 1.5;

    ros::Time current_time_;

    // 用于odom获取坐标变换
    std::mutex odom_lock_;
    std::deque<nav_msgs::Odometry> odom_queue_;
    int odom_queue_length_;
    std::mutex initial_pose_lock_;
    bool has_manual_initial_pose_ = false;
    bool pending_manual_relocalization_ = false;
    Eigen::Isometry3d manual_initial_pose_ = Eigen::Isometry3d::Identity();
    std::mutex map_to_odom_lock_;
    bool has_map_to_odom_estimate_ = false;
    Eigen::Isometry3d map_to_odom_estimate_ = Eigen::Isometry3d::Identity();
    ros::Time last_map_to_odom_stamp_;
    double map_to_odom_publish_rate_ = 40.0;
    double map_to_odom_stationary_alpha_ = 0.12;
    double map_to_odom_moving_alpha_ = 0.85;
    double map_to_odom_moving_linear_threshold_ = 0.08;
    double map_to_odom_moving_angular_threshold_ = 0.15;
    double map_to_odom_stationary_translation_deadband_ = 0.008;
    double map_to_odom_stationary_yaw_deadband_ = 0.010;

    // icp
    double ANGLE_SPEED_THRESHOLD_;    // 角速度阈值，大于此值不发布结果
    double AGE_THRESHOLD_;            // scan与匹配的最大时间间隔
    double ANGLE_UPPER_THRESHOLD_;    // 最大变换角度
    double ANGLE_THRESHOLD_;          // 最小变换角度
    double DIST_THRESHOLD_;           // 最小变换距离
    double SCORE_THRESHOLD_MAX_;      // 达到最大迭代次数或者到达差分阈值后后，代价仍高于此值，认为无法收敛
    double Point_Quantity_THRESHOLD_; // 点云数阈值
    double Maximum_Iterations_;       // ICP中的最大迭代次数

    double Variance_X; // 协方差
    double Variance_Y;
    double Variance_Yaw;

    double Scan_Range_Max; // 最大雷达数据距离
    double Scan_Range_Min; // 最小雷达数据距离

    // pcl
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

    PointCloudT::Ptr cloud_map_;
    PointCloudT::Ptr cloud_scan_;
    PointCloudT::Ptr cloud_scan_reference_;
    PointCloudT::Ptr cached_local_map_;
    pcl::KdTreeFLANN<PointT> map_kdtree_;
    bool map_kdtree_initialized_ = false;
    bool has_cached_local_map_ = false;
    Eigen::Isometry3d cached_local_map_pose_ = Eigen::Isometry3d::Identity();
    double local_map_cache_translation_threshold_ = 0.30;
    double local_map_cache_yaw_threshold_ = 0.10;

    void InitParams();

    // scan to map匹配
    bool ScanMatchWithICP(Eigen::Isometry3d &trans, PointCloudT::Ptr &cloud_scan_msg, PointCloudT::Ptr &cloud_map_msg);
    double ComputeAdaptiveThreshold() const;
    void UpdateAdaptiveThreshold(const Eigen::Isometry3d &model_deviation);
    static Eigen::Isometry3d ExpSE2(double dx, double dy, double dtheta);

    void PointCloudOutlierRemoval(PointCloudT::Ptr &cloud_msg);
    void PointCloudObstacleRemoval(PointCloudT::Ptr &cloud_map_msg, PointCloudT::Ptr &cloud_msg, double Distance_Threshold);
    void PointCloudVoxelGridRemoval(PointCloudT::Ptr &cloud_msg, double leafSize);

    // 数据格式转换
    void OccupancyGridToPointCloud(const nav_msgs::OccupancyGrid::ConstPtr &map_msg, PointCloudT::Ptr &cloud_msg);
    void ScanToPointCloudInReference(const sensor_msgs::LaserScan::ConstPtr &scan_msg, PointCloudT::Ptr &cloud_msg);
    void TransformPointCloudToMap(const PointCloudT::Ptr &source_cloud, const Eigen::Isometry3d &map_to_lidar,
                                  const ros::Time &stamp, PointCloudT::Ptr &target_cloud) const;
    void ExtractLocalMap(const Eigen::Isometry3d &map_to_base, PointCloudT::Ptr &local_map, double radius = -1.0) const;
    bool SearchPoseCandidate(const PointCloudT::Ptr &scan_cloud_reference,
                             const Eigen::Isometry3d &predicted_map_to_base,
                             Eigen::Isometry3d &searched_map_to_base);
    double ScorePoseCandidate(const PointCloudT::Ptr &scan_cloud_reference,
                              const Eigen::Isometry3d &candidate_map_to_lidar,
                              const pcl::KdTreeFLANN<PointT> &score_kdtree) const;
    void UpdateLocationCovariance();
    bool ComputeMapToOdomTransform(const Eigen::Isometry3d &map_to_base,
                                   const ros::Time &stamp,
                                   Eigen::Isometry3d &map_to_odom);
    void PublishMapToOdomTransform(const Eigen::Isometry3d &map_to_base,
                                   const ros::Time &stamp);
    void mapToOdomPublishTimerCallback(const ros::TimerEvent &event);

    // 坐标变换
    bool GetTransform(Eigen::Isometry3d &trans, const std::string parent_frame,
                      const std::string child_frame, const ros::Time stamp);
    bool GetOdomTransform(Eigen::Isometry3d &trans, double start_stamp, double end_stamp);
    bool LookupOdomPose(Eigen::Isometry3d &pose, double stamp, const std::deque<nav_msgs::Odometry> &odom_queue) const;
    double ComputeScanDuration(const sensor_msgs::LaserScan::ConstPtr &scan_msg) const;
    double ComputeScanPointTimeIncrement(const sensor_msgs::LaserScan::ConstPtr &scan_msg) const;
    double ComputeScanReferenceTime(const sensor_msgs::LaserScan::ConstPtr &scan_msg) const;
    void PublishDebugPose(const ros::Publisher &publisher, const Eigen::Isometry3d &pose,
                          const ros::Time &stamp, const std::string &frame_id);
    void PublishLocalizationDebugState(size_t frame_id, const ros::Time &stamp,
                                       const std::string &stage, const std::string &status,
                                       const std::string &reason, const Eigen::Isometry3d *predicted_pose,
                                       const Eigen::Isometry3d *final_pose);
    void LogFrameSummary(size_t frame_id, const std::string &stage, const std::string &status,
                         const std::string &reason, const Eigen::Isometry3d *predicted_pose,
                         const Eigen::Isometry3d *final_pose);
    static void ExtractPose2D(const Eigen::Isometry3d &pose, double &x, double &y, double &yaw);
    static Eigen::Isometry3d BuildPose2D(double x, double y, double yaw);
    static double NormalizeAngle(double angle);

public:
    Scan2MapLocation();
    ~Scan2MapLocation() = default;

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &odometryMsg);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
};

#endif  // SCAN_TO_MAP_LOCATION_H
