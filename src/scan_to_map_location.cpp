#include "scan_to_map_location.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <std_msgs/String.h>
#include <algorithm>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

Scan2MapLocation::Scan2MapLocation() : private_nh("~"), tf_listener_(tfBuffer_)
{
    laser_sub_ = location_nh.subscribe("/scan", 1, &Scan2MapLocation::laserCallback, this, ros::TransportHints().tcpNoDelay());
    map_sub_ = location_nh.subscribe("map", 1, &Scan2MapLocation::mapCallback, this);
    odom_sub_ = location_nh.subscribe("odom", 20, &Scan2MapLocation::odomCallback, this, ros::TransportHints().tcpNoDelay());
    initial_pose_sub_ = location_nh.subscribe("initialpose", 1, &Scan2MapLocation::initialPoseCallback, this);

    removal_pointcloud_publisher_ = location_nh.advertise<sensor_msgs::PointCloud2>("removal_pointcloud", 10);
    location_publisher_ = location_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("location_match", 1);
    localization_state_publisher_ = location_nh.advertise<std_msgs::String>("localization_state", 1, true);
    localization_diagnostics_publisher_ = location_nh.advertise<diagnostic_msgs::DiagnosticArray>("localization_diagnostics", 1, true);
    debug_predicted_pose_publisher_ = location_nh.advertise<geometry_msgs::PoseStamped>("debug_predicted_pose", 1);
    debug_final_pose_publisher_ = location_nh.advertise<geometry_msgs::PoseStamped>("debug_final_pose", 1);
    debug_scan_predicted_publisher_ = location_nh.advertise<sensor_msgs::PointCloud2>("debug_predicted_scan", 1);
    debug_scan_used_publisher_ = location_nh.advertise<sensor_msgs::PointCloud2>("debug_scan_used", 1);
    debug_scan_final_publisher_ = location_nh.advertise<sensor_msgs::PointCloud2>("debug_final_scan", 1);

    cloud_map_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    cloud_scan_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    cloud_scan_reference_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    cached_local_map_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    InitParams();
    map_to_odom_publish_timer_ = location_nh.createTimer(
        ros::Duration(1.0 / std::max(1.0, map_to_odom_publish_rate_)),
        &Scan2MapLocation::mapToOdomPublishTimerCallback, this);
}

void Scan2MapLocation::InitParams()
{
    private_nh.param<bool>("if_debug", if_debug_, false);
    private_nh.param<bool>("debug_publish_poses", debug_publish_poses_, false);
    private_nh.param<bool>("debug_publish_clouds", debug_publish_clouds_, false);
    private_nh.param<bool>("publish_localization_state", publish_localization_state_, true);
    private_nh.param<bool>("publish_localization_diagnostics", publish_localization_diagnostics_, true);
    private_nh.param<bool>("publish_map_to_odom_tf", publish_map_to_odom_tf_, true);
    private_nh.param<bool>("use_tf_initial_pose", use_tf_initial_pose_, false);
    private_nh.param<bool>("smooth_map_to_odom", smooth_map_to_odom_, true);
    private_nh.param<bool>("enable_initial_global_yaw_search", enable_initial_global_yaw_search_, true);
    private_nh.param<bool>("enable_startup_relocalization", enable_startup_relocalization_, true);

    private_nh.param<std::string>("odom_frame", odom_frame_, "odom");
    private_nh.param<std::string>("base_frame", base_frame_, "base_link");
    private_nh.param<std::string>("map_frame", map_frame_, "map");
    private_nh.param<std::string>("lidar_frame", lidar_frame_, "laser_frame");

    private_nh.param<int>("odom_queue_length", odom_queue_length_, 300);

    // ICP匹配相关参数
    private_nh.param<double>("ANGLE_SPEED_THRESHOLD", ANGLE_SPEED_THRESHOLD_, 7);         // 角速度阈值，大于此值不发布结果
    private_nh.param<double>("AGE_THRESHOLD", AGE_THRESHOLD_, 1);                         // scan与匹配的最大时间间隔
    private_nh.param<double>("ANGLE_UPPER_THRESHOLD", ANGLE_UPPER_THRESHOLD_, 10);        // 最大变换角度
    private_nh.param<double>("ANGLE_THRESHOLD", ANGLE_THRESHOLD_, 0.01);                  // 最小变换角度
    private_nh.param<double>("DIST_THRESHOLD", DIST_THRESHOLD_, 0.01);                    // 最小变换距离
    private_nh.param<double>("SCORE_THRESHOLD_MAX", SCORE_THRESHOLD_MAX_, 0.1);           // 达到最大迭代次数或者到达差分阈值后后，代价仍高于此值，认为无法收敛,自适应使用
    private_nh.param<double>("Point_Quantity_THRESHOLD", Point_Quantity_THRESHOLD_, 200); // 点云数阈值,低于此值不匹配
    private_nh.param<double>("Maximum_Iterations", Maximum_Iterations_, 100);             // ICP中的最大迭代次数
    private_nh.param<double>("pose_jump_risk_threshold", pose_jump_risk_threshold_, 0.5);
    private_nh.param<bool>("suppress_static_jitter", suppress_static_jitter_, false);
    private_nh.param<bool>("use_scan_midpoint_time", use_scan_midpoint_time_, true);
    private_nh.param<bool>("enable_scan_deskew", enable_scan_deskew_, true);
    private_nh.param<bool>("enable_yaw_search", enable_yaw_search_, true);
    private_nh.param<double>("search_translation_range", search_translation_range_, 0.15);
    private_nh.param<double>("search_translation_step", search_translation_step_, 0.05);
    private_nh.param<double>("search_score_max_distance", search_score_max_distance_, 0.25);
    private_nh.param<double>("search_score_leaf_size", search_score_leaf_size_, 0.08);
    private_nh.param<double>("local_map_radius", local_map_radius_, 8.0);
    private_nh.param<double>("search_accept_score_threshold", search_accept_score_threshold_, 0.34);
    private_nh.param<double>("search_accept_improvement_threshold", search_accept_improvement_threshold_, 0.010);
    private_nh.param<double>("search_accept_improvement_ratio", search_accept_improvement_ratio_, 0.97);
    private_nh.param<double>("covariance_translation_min", covariance_translation_min_, 0.02);
    private_nh.param<double>("covariance_yaw_min", covariance_yaw_min_, 0.03);
    private_nh.param<double>("adaptive_threshold_initial", adaptive_threshold_initial_, 0.20);
    private_nh.param<double>("adaptive_threshold_min_motion", adaptive_threshold_min_motion_, 0.05);
    private_nh.param<double>("registration_translation_epsilon", registration_translation_epsilon_, 1e-3);
    private_nh.param<double>("registration_rotation_epsilon", registration_rotation_epsilon_, 1e-3);
    private_nh.param<double>("static_linear_speed_threshold", static_linear_speed_threshold_, 0.03);
    private_nh.param<double>("static_angular_speed_threshold", static_angular_speed_threshold_, 0.05);
    private_nh.param<double>("static_pose_translation_epsilon", static_pose_translation_epsilon_, 0.02);
    private_nh.param<double>("static_pose_yaw_epsilon", static_pose_yaw_epsilon_, 0.03);
    private_nh.param<double>("static_odom_translation_epsilon", static_odom_translation_epsilon_, 0.01);
    private_nh.param<double>("static_odom_yaw_epsilon", static_odom_yaw_epsilon_, 0.02);
    private_nh.param<double>("yaw_search_trigger_angular_z", yaw_search_trigger_angular_z_, 0.12);
    private_nh.param<double>("yaw_search_trigger_odom_yaw", yaw_search_trigger_odom_yaw_, 0.015);
    private_nh.param<double>("yaw_search_range_deg", yaw_search_range_deg_, 18.0);
    private_nh.param<double>("yaw_search_step_deg", yaw_search_step_deg_, 6.0);
    private_nh.param<double>("startup_relocalization_translation_range", startup_relocalization_translation_range_, 0.20);
    private_nh.param<double>("startup_relocalization_translation_step", startup_relocalization_translation_step_, 0.04);
    private_nh.param<double>("startup_relocalization_yaw_range_deg", startup_relocalization_yaw_range_deg_, 10.0);
    private_nh.param<double>("startup_relocalization_yaw_step_deg", startup_relocalization_yaw_step_deg_, 2.0);
    private_nh.param<double>("startup_relocalization_accept_score_threshold", startup_relocalization_accept_score_threshold_, 0.40);
    private_nh.param<double>("startup_relocalization_accept_improvement_threshold", startup_relocalization_accept_improvement_threshold_, 0.002);
    private_nh.param<double>("startup_relocalization_accept_improvement_ratio", startup_relocalization_accept_improvement_ratio_, 0.995);
    private_nh.param<double>("initial_global_yaw_search_range_deg", initial_global_yaw_search_range_deg_, 180.0);
    private_nh.param<double>("initial_global_yaw_search_step_deg", initial_global_yaw_search_step_deg_, 30.0);
    private_nh.param<int>("initial_global_yaw_search_frames", initial_global_yaw_search_frames_, 8);
    private_nh.param<int>("startup_relocalization_frames", startup_relocalization_frames_, 12);
    private_nh.param<double>("map_to_odom_publish_rate", map_to_odom_publish_rate_, 40.0);
    private_nh.param<double>("map_to_odom_stationary_alpha", map_to_odom_stationary_alpha_, 0.12);
    private_nh.param<double>("map_to_odom_moving_alpha", map_to_odom_moving_alpha_, 0.85);
    private_nh.param<double>("map_to_odom_moving_linear_threshold", map_to_odom_moving_linear_threshold_, 0.08);
    private_nh.param<double>("map_to_odom_moving_angular_threshold", map_to_odom_moving_angular_threshold_, 0.15);
    private_nh.param<double>("map_to_odom_stationary_translation_deadband", map_to_odom_stationary_translation_deadband_, 0.008);
    private_nh.param<double>("map_to_odom_stationary_yaw_deadband", map_to_odom_stationary_yaw_deadband_, 0.010);
    private_nh.param<double>("local_map_cache_translation_threshold", local_map_cache_translation_threshold_, 0.30);
    private_nh.param<double>("local_map_cache_yaw_threshold", local_map_cache_yaw_threshold_, 0.10);

    // 发布位姿的方差
    private_nh.param<double>("Variance_X", Variance_X, 0.01);     // x方向上方差
    private_nh.param<double>("Variance_Y", Variance_Y, 0.01);     // y方向上方差
    private_nh.param<double>("Variance_Yaw", Variance_Yaw, 0.01); // yaw方向上方差

    private_nh.param<double>("Scan_Range_Max", Scan_Range_Max, 20);  // 雷达数据点最大值
    private_nh.param<double>("Scan_Range_Min", Scan_Range_Min, 0.3); // 雷达数据点最小值

    private_nh.param<bool>("Use_TfTree_Always", Use_TfTree_Always, false); // 是否总是使用tf树读取变换

    // 体素滤波的边长
    private_nh.param<double>("VoxelGridRemoval_LeafSize", VoxelGridRemoval_LeafSize, 0.05);
    private_nh.param<bool>("enable_outlier_removal", enable_outlier_removal_, true);
    private_nh.param<int>("outlier_mean_k", outlier_mean_k_, 16);
    private_nh.param<double>("outlier_stddev_mul", outlier_stddev_mul_, 1.5);
    outlier_mean_k_ = std::max(3, outlier_mean_k_);
    outlier_stddev_mul_ = std::max(0.1, outlier_stddev_mul_);

    // 迭代障碍物去除
    // 如果雷达点云中点在地图点云最近点大于此值，就认为该点为障碍点，有最大和最小值，会随着icp迭代的SCORE值按比例进行更新
    private_nh.param<double>("ObstacleRemoval_Distance_Max", ObstacleRemoval_Distance_Max, 2); // 最大距离

    adaptive_model_sse_ = adaptive_threshold_initial_ * adaptive_threshold_initial_;
    adaptive_model_samples_ = 1;
    last_registration_sigma_ = adaptive_threshold_initial_;
    last_registration_max_correspondence_ = 3.0 * adaptive_threshold_initial_;
}

void Scan2MapLocation::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg)
{
    tf2::Quaternion quat(pose_msg->pose.pose.orientation.x,
                         pose_msg->pose.pose.orientation.y,
                         pose_msg->pose.pose.orientation.z,
                         pose_msg->pose.pose.orientation.w);
    const double yaw = tf2::getYaw(quat);
    std::lock_guard<std::mutex> lock(initial_pose_lock_);
    manual_initial_pose_ = BuildPose2D(pose_msg->pose.pose.position.x,
                                       pose_msg->pose.pose.position.y,
                                       yaw);
    has_manual_initial_pose_ = true;
    pending_manual_relocalization_ = true;
}

void Scan2MapLocation::odomCallback(const nav_msgs::Odometry::ConstPtr &odometry_msg)
{
    std::lock_guard<std::mutex> lock(odom_lock_);
    odom_initialized_ = true;
    latest_odom_linear_x_ = odometry_msg->twist.twist.linear.x;
    latest_odom_linear_y_ = odometry_msg->twist.twist.linear.y;
    latest_odom_angular_z_ = odometry_msg->twist.twist.angular.z;
    odom_queue_.push_back(*odometry_msg);
    if (odom_queue_.size() > odom_queue_length_)
    { // 弹出超过长度的数据
        odom_queue_.pop_front();
    }
}

void Scan2MapLocation::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    map_initialized_ = true;
    OccupancyGridToPointCloud(map_msg, cloud_map_); // map数据转为pointcloud数据
    // PointCloudVoxelGridRemoval(cloud_map_, VoxelGridRemoval_LeafSize);
    if (!cloud_map_->empty())
    {
        map_kdtree_.setInputCloud(cloud_map_);
        map_kdtree_initialized_ = true;
    }
    else
    {
        map_kdtree_initialized_ = false;
    }
}

void Scan2MapLocation::laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    const size_t frame_id = ++debug_frame_id_;
    last_failure_reason_.clear();
    last_icp_risk_flags_.clear();
    last_removed_count_ = 0;
    last_scan_voxel_count_ = 0;
    last_scan_used_count_ = 0;
    last_odom_wait_count_ = 0;
    last_odom_dt_ = 0.0;
    last_static_lock_applied_ = false;
    last_scan_start_time_ = 0.0;
    last_scan_end_time_ = 0.0;
    last_scan_duration_ = 0.0;
    last_scan_point_time_increment_ = 0.0;
    last_scan_reference_offset_ = 0.0;
    last_scan_motion_translation_ = 0.0;
    last_scan_motion_yaw_ = 0.0;
    last_scan_odom_tail_gap_ = 0.0;
    last_scan_odom_coverage_ok_ = false;
    last_scan_odom_wait_count_ = 0;
    last_scan_deskew_applied_ = false;
    last_scan_deskew_point_count_ = 0;
    last_scan_deskew_failed_point_count_ = 0;
    last_icp_yaw_search_used_ = false;
    last_icp_candidate_count_ = 0;
    last_icp_selected_yaw_offset_ = 0.0;
    last_registration_correspondence_count_ = 0;
    last_search_active_ = false;
    last_search_applied_ = false;
    last_search_candidate_count_ = 0;
    last_search_predicted_score_ = std::numeric_limits<double>::quiet_NaN();
    last_search_dx_ = 0.0;
    last_search_dy_ = 0.0;
    last_search_dyaw_ = 0.0;
    last_search_score_ = std::numeric_limits<double>::quiet_NaN();
    last_startup_relocalization_active_ = false;

    if (!map_initialized_ || !odom_initialized_)
    {
        ROS_WARN_THROTTLE(1.0, "[loc-debug] waiting for map/odom initialization ...");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(odom_lock_);
        if (odom_queue_.empty())
        {
            ROS_WARN_THROTTLE(1.0, "waiting for first odom msg ...");
            return;
        }
        last_odom_front_time_ = odom_queue_.front().header.stamp.toSec();
        last_odom_back_time_ = odom_queue_.back().header.stamp.toSec();
    }

    bool reset_for_manual_relocalization = false;
    {
        std::lock_guard<std::mutex> lock(initial_pose_lock_);
        reset_for_manual_relocalization = pending_manual_relocalization_;
    }
    if (reset_for_manual_relocalization)
    {
        ROS_INFO_THROTTLE(1.0, "[loc-debug] apply manual initial pose relocalization");
        scan_initialized_ = false;
        tracking_frame_count_ = 0;
        require_heading_bootstrap_ = false;
        has_static_lock_pose_ = false;
        has_cached_local_map_ = false;
        match_result_ = Eigen::Isometry3d::Identity();
        last_match_result_ = Eigen::Isometry3d::Identity();
        {
            std::lock_guard<std::mutex> lock(map_to_odom_lock_);
            has_map_to_odom_estimate_ = false;
            map_to_odom_estimate_ = Eigen::Isometry3d::Identity();
            last_map_to_odom_stamp_ = ros::Time(0);
        }
    }

    Eigen::Isometry3d predicted_map_to_base = Eigen::Isometry3d::Identity();
    bool has_predicted_pose = false;
    const double scan_reference_time = ComputeScanReferenceTime(scan_msg);
    const ros::Time scan_reference_stamp(scan_reference_time);
    // 未初始化时，需要通过tf变换读取map_to_lidar和map_to_base的坐标变换
    // 初始化函数与坐标变换
    if (!scan_initialized_)
    {

        if (if_debug_)
        {
            std::cout << "initial scancallback " << std::endl;
        }
        scan_time_ = scan_reference_time; // 初始化匹配时间
        match_time_ = scan_time_;
        last_scan_age_ = ros::Time::now().toSec() - scan_time_;

        // 读取base_to_lidar静态变换
        base_to_lidar_ = Eigen::Isometry3d::Identity();
        if (!GetTransform(base_to_lidar_, base_frame_, lidar_frame_, scan_reference_stamp))
        {
            last_failure_reason_ = "missing_tf_base_to_lidar";
            LogFrameSummary(frame_id, "init", "FAIL", last_failure_reason_, nullptr, nullptr);
            return;
        }
        if (if_debug_)
        {
            std::cout << "success get base_to_lidar_" << std::endl;
        }

        map_to_base_ = Eigen::Isometry3d::Identity();
        bool initialized_from_pose = false;

        {
            std::lock_guard<std::mutex> lock(initial_pose_lock_);
            if (has_manual_initial_pose_)
            {
                map_to_base_ = manual_initial_pose_;
                initialized_from_pose = true;
                has_manual_initial_pose_ = false;
                pending_manual_relocalization_ = false;
            }
        }

        if (!initialized_from_pose && use_tf_initial_pose_)
        {
            if (!GetTransform(map_to_base_, map_frame_, base_frame_, scan_reference_stamp))
            {
                last_failure_reason_ = "missing_tf_map_to_base";
                LogFrameSummary(frame_id, "init", "FAIL", last_failure_reason_, nullptr, nullptr);
                return;
            }
            initialized_from_pose = true;
        }

        if (!initialized_from_pose)
        {
            Eigen::Isometry3d odom_to_base = Eigen::Isometry3d::Identity();
            std::deque<nav_msgs::Odometry> odom_queue_snapshot;
            {
                std::lock_guard<std::mutex> lock(odom_lock_);
                odom_queue_snapshot = odom_queue_;
            }
            if (!LookupOdomPose(odom_to_base, scan_reference_time, odom_queue_snapshot))
            {
                last_failure_reason_ = "missing_initial_odom_pose";
                LogFrameSummary(frame_id, "init", "FAIL", last_failure_reason_, nullptr, nullptr);
                return;
            }
            map_to_base_ = odom_to_base;
        }

        {
            double map_x, map_y, map_yaw;
            ExtractPose2D(map_to_base_, map_x, map_y, map_yaw);
            map_to_base_ = BuildPose2D(map_x, map_y, map_yaw);
        }
        require_heading_bootstrap_ = !initialized_from_pose;
        scan_initialized_ = true;

        map_to_lidar_ = map_to_base_ * base_to_lidar_; // 获得map到lidar变换，用于雷达数据转地图数据
        predicted_map_to_base = map_to_base_;
        has_predicted_pose = true;
        static_lock_pose_ = map_to_base_;
        has_static_lock_pose_ = true;
    }
    else
    {
        // 初始化后，通过上一帧数据和odom数据计算得到map_to_lidar和map_to_base的坐标变换
        if (if_debug_)
        {
            std::cout << "scancallback initialized" << std::endl;
        }
        last_match_time_ = match_time_;
        last_match_result_ = match_result_; // 保存上一次匹配结果
        scan_time_ = scan_reference_time;
        last_scan_age_ = ros::Time::now().toSec() - scan_time_;

        // 判断是否超时，如果超时退出
        if (ros::Time::now().toSec() - scan_time_ > AGE_THRESHOLD_)
        {
            last_failure_reason_ = "scan_timeout";
            LogFrameSummary(frame_id, "track", "FAIL", last_failure_reason_, &last_match_result_, nullptr);
            scan_initialized_ = false; // 数据超时，需要重新初始化
            return;
        }
        Eigen::Isometry3d baselast_to_basenow = Eigen::Isometry3d::Identity();
        if (!GetOdomTransform(baselast_to_basenow, last_match_time_, scan_time_))
        {
            last_failure_reason_ = last_failure_reason_.empty() ? "odom_delta_unavailable" : last_failure_reason_;
            LogFrameSummary(frame_id, "track", "FAIL", last_failure_reason_, &last_match_result_, nullptr);
            return;
        }

        map_to_base_ = last_match_result_ * baselast_to_basenow;
        {
            double map_x, map_y, map_yaw;
            ExtractPose2D(map_to_base_, map_x, map_y, map_yaw);
            map_to_base_ = BuildPose2D(map_x, map_y, map_yaw);
        }
        map_to_lidar_ = map_to_base_ * base_to_lidar_; // 获得map到lidar变换，用于雷达数据转地图数据
        predicted_map_to_base = map_to_base_;
        has_predicted_pose = true;
    }

    if (has_predicted_pose && debug_publish_poses_)
    {
        PublishDebugPose(debug_predicted_pose_publisher_, predicted_map_to_base, scan_reference_stamp, map_frame_);
    }

    if (Use_TfTree_Always)
    {
        scan_initialized_ = false; // 每次都使用tf读取结果
    }

    ScanToPointCloudInReference(scan_msg, cloud_scan_reference_);

    if (has_predicted_pose && debug_publish_clouds_)
    {
        PointCloudT::Ptr cloud_predicted(new PointCloudT);
        TransformPointCloudToMap(cloud_scan_reference_, predicted_map_to_base * base_to_lidar_,
                                 scan_reference_stamp, cloud_predicted);
        debug_scan_predicted_publisher_.publish(cloud_predicted);
    }

    Eigen::Isometry3d searched_map_to_base = predicted_map_to_base;
    if (has_predicted_pose)
    {
        SearchPoseCandidate(cloud_scan_reference_, predicted_map_to_base, searched_map_to_base);
    }
    map_to_base_ = searched_map_to_base;
    map_to_lidar_ = map_to_base_ * base_to_lidar_;

    TransformPointCloudToMap(cloud_scan_reference_, map_to_lidar_, scan_reference_stamp, cloud_scan_);

    PointCloudT::Ptr cloud_map_local;
    bool rebuild_local_map = true;
    if (has_cached_local_map_)
    {
        double cached_x, cached_y, cached_yaw;
        double current_x, current_y, current_yaw;
        ExtractPose2D(cached_local_map_pose_, cached_x, cached_y, cached_yaw);
        ExtractPose2D(map_to_base_, current_x, current_y, current_yaw);
        const double translation_delta = std::hypot(current_x - cached_x, current_y - cached_y);
        const double yaw_delta = std::abs(NormalizeAngle(current_yaw - cached_yaw));
        if (translation_delta < local_map_cache_translation_threshold_ &&
            yaw_delta < local_map_cache_yaw_threshold_)
        {
            rebuild_local_map = false;
        }
    }

    if (rebuild_local_map)
    {
        cached_local_map_->clear();
        ExtractLocalMap(map_to_base_, cached_local_map_);
        if (cached_local_map_->empty())
        {
            *cached_local_map_ = *cloud_map_;
        }
        cached_local_map_pose_ = map_to_base_;
        has_cached_local_map_ = true;
    }
    cloud_map_local = cached_local_map_;

    PointCloudVoxelGridRemoval(cloud_scan_, VoxelGridRemoval_LeafSize);
    last_scan_voxel_count_ = cloud_scan_->points.size();
    if (enable_outlier_removal_)
    {
        PointCloudOutlierRemoval(cloud_scan_);
    }
    PointCloudObstacleRemoval(cloud_map_local, cloud_scan_, ObstacleRemoval_Distance_Max);
    last_scan_used_count_ = cloud_scan_->points.size();
    if (debug_publish_clouds_)
    {
        debug_scan_used_publisher_.publish(cloud_scan_);
    }

    match_result_ = Eigen::Isometry3d::Identity();
    if (!ScanMatchWithICP(match_result_, cloud_scan_, cloud_map_local))
    {
        last_failure_reason_ = last_failure_reason_.empty() ? "icp_rejected" : last_failure_reason_;
        LogFrameSummary(frame_id, "icp", "FAIL", last_failure_reason_, has_predicted_pose ? &predicted_map_to_base : nullptr, nullptr);
        scan_initialized_ = false; // 数据错误，需要重新初始化
        return;
    }

    match_result_ = match_result_ * map_to_base_; // 将结果转换到map坐标系
    {
        double map_x, map_y, map_yaw;
        ExtractPose2D(match_result_, map_x, map_y, map_yaw);
        match_result_ = BuildPose2D(map_x, map_y, map_yaw);
    }

    if (has_predicted_pose && suppress_static_jitter_)
    {
        double pred_x, pred_y, pred_yaw;
        double final_x, final_y, final_yaw;
        ExtractPose2D(predicted_map_to_base, pred_x, pred_y, pred_yaw);
        ExtractPose2D(match_result_, final_x, final_y, final_yaw);

        const double linear_speed = std::hypot(latest_odom_linear_x_, latest_odom_linear_y_);
        const double odom_translation_delta = std::hypot(last_odom_delta_x_, last_odom_delta_y_);
        const double pose_translation_delta = std::hypot(final_x - pred_x, final_y - pred_y);
        const double pose_yaw_delta = std::abs(std::atan2(std::sin(final_yaw - pred_yaw), std::cos(final_yaw - pred_yaw)));

        if (linear_speed < static_linear_speed_threshold_ &&
            std::abs(latest_odom_angular_z_) < static_angular_speed_threshold_ &&
            odom_translation_delta < static_odom_translation_epsilon_ &&
            std::abs(last_odom_delta_yaw_) < static_odom_yaw_epsilon_ &&
            pose_translation_delta < static_pose_translation_epsilon_ &&
            pose_yaw_delta < static_pose_yaw_epsilon_)
        {
            if (has_static_lock_pose_ && tracking_frame_count_ >= static_lock_warmup_frames_)
            {
                match_result_ = static_lock_pose_;
            }
            else
            {
                match_result_ = predicted_map_to_base;
            }
            last_static_lock_applied_ = true;
        }
        else
        {
            static_lock_pose_ = match_result_;
            has_static_lock_pose_ = true;
        }
    }
    else
    {
        static_lock_pose_ = match_result_;
        has_static_lock_pose_ = true;
    }
    if (tracking_frame_count_ >= static_cast<size_t>(initial_global_yaw_search_frames_))
    {
        require_heading_bootstrap_ = false;
    }
    tracking_frame_count_++;

    match_time_ = scan_time_;
    if (debug_publish_poses_)
    {
        PublishDebugPose(debug_final_pose_publisher_, match_result_, scan_reference_stamp, map_frame_);
    }
    if (debug_publish_clouds_)
    {
        PointCloudT::Ptr cloud_final(new PointCloudT);
        TransformPointCloudToMap(cloud_scan_reference_, match_result_ * base_to_lidar_, scan_reference_stamp, cloud_final);
        debug_scan_final_publisher_.publish(cloud_final);
    }

    Eigen::Quaterniond q = Eigen::Quaterniond(match_result_.rotation());
    location_match.header.stamp = scan_reference_stamp;
    location_match.header.frame_id = "map";
    location_match.pose.pose.orientation.x = q.x();
    location_match.pose.pose.orientation.y = q.y();
    location_match.pose.pose.orientation.z = q.z();
    location_match.pose.pose.orientation.w = q.w();
    // location_match.pose.orientation = q;
    location_match.pose.pose.position.x = match_result_.translation()(0);
    location_match.pose.pose.position.y = match_result_.translation()(1);
    location_match.pose.pose.position.z = match_result_.translation()(2);
    // x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis
    UpdateLocationCovariance();
    const bool suppress_publish_for_turn =
        ANGLE_SPEED_THRESHOLD_ > 0.0 &&
        std::abs(latest_odom_angular_z_) > ANGLE_SPEED_THRESHOLD_;

    std::string status = "OK";
    std::string reason = "accepted";
    std::vector<std::string> risk_flags;
    if (!last_icp_risk_flags_.empty())
    {
        status = "RISK";
        reason = last_icp_risk_flags_;
    }

    auto append_reason = [&](const std::string &flag)
    {
        status = "RISK";
        if (reason.empty() || reason == "accepted")
        {
            reason = flag;
        }
        else
        {
            reason += "|" + flag;
        }
    };

    if (last_scan_duration_ > 1e-6 && last_scan_point_time_increment_ <= 0.0)
    {
        append_reason("scan_dt_invalid");
    }
    if (last_scan_duration_ > 1e-6 && !last_scan_odom_coverage_ok_)
    {
        append_reason("scan_odom_uncovered");
    }
    if ((std::abs(last_scan_motion_yaw_) > 0.03 || last_scan_motion_translation_ > 0.02) &&
        !last_scan_deskew_applied_)
    {
        append_reason("deskew_missing");
    }
    if (last_scan_deskew_failed_point_count_ > 0)
    {
        append_reason("deskew_partial");
    }

    if (has_predicted_pose)
    {
        double pred_x, pred_y, pred_yaw;
        double final_x, final_y, final_yaw;
        ExtractPose2D(predicted_map_to_base, pred_x, pred_y, pred_yaw);
        ExtractPose2D(match_result_, final_x, final_y, final_yaw);
        const double pose_jump = std::hypot(final_x - pred_x, final_y - pred_y);
        if (pose_jump > pose_jump_risk_threshold_)
        {
            status = "RISK";
            if (!reason.empty() && reason != "accepted")
            {
                reason += "|";
            }
            else
            {
                reason.clear();
            }
            reason += "large_pose_jump";
        }
    }

    if (suppress_publish_for_turn)
    {
        append_reason("high_angular_speed_hold");
    }

    if (!suppress_publish_for_turn)
    {
        PublishMapToOdomTransform(match_result_, scan_reference_stamp);
        location_publisher_.publish(location_match); // 发送匹配结果
    }

    LogFrameSummary(frame_id, "track", status, reason,
                    has_predicted_pose ? &predicted_map_to_base : nullptr, &match_result_);
}

/**
 * 使用ICP进行帧间位姿的计算
 */
bool Scan2MapLocation::ScanMatchWithICP(Eigen::Isometry3d &trans, PointCloudT::Ptr &cloud_scan_msg, PointCloudT::Ptr &cloud_map_msg)
{
    last_icp_converged_ = false;
    last_icp_low_points_ = false;
    last_icp_small_correction_ = false;
    last_icp_high_score_ = false;
    last_icp_large_angle_ = false;
    last_icp_score_ = std::numeric_limits<double>::quiet_NaN();
    last_icp_tran_dist_ = 0.0;
    last_icp_angle_dist_ = 0.0;
    last_icp_dx_ = 0.0;
    last_icp_dy_ = 0.0;
    last_icp_dyaw_ = 0.0;
    last_icp_risk_flags_.clear();
    last_failure_reason_.clear();
    last_icp_yaw_search_used_ = false;
    last_icp_candidate_count_ = 0;
    last_icp_selected_yaw_offset_ = 0.0;

    if (cloud_scan_msg->empty())
    {
        last_failure_reason_ = "empty_scan_cloud";
        return false;
    }
    if (cloud_map_msg->empty())
    {
        last_failure_reason_ = "empty_map_cloud";
        return false;
    }

    const double sigma = ComputeAdaptiveThreshold();
    const double max_correspondence_distance = std::max(0.10, 3.0 * sigma);
    last_registration_sigma_ = sigma;
    last_registration_max_correspondence_ = max_correspondence_distance;

    pcl::KdTreeFLANN<PointT> local_map_kdtree;
    local_map_kdtree.setInputCloud(cloud_map_msg);

    Eigen::Isometry3d estimate = Eigen::Isometry3d::Identity();
    size_t correspondence_count = 0;
    double weighted_residual_sum = 0.0;
    double weighted_residual_weight = 0.0;

    for (int iter = 0; iter < static_cast<int>(Maximum_Iterations_); ++iter)
    {
        Eigen::Matrix3d JTJ = Eigen::Matrix3d::Zero();
        Eigen::Vector3d JTr = Eigen::Vector3d::Zero();
        correspondence_count = 0;
        weighted_residual_sum = 0.0;
        weighted_residual_weight = 0.0;

        const double cos_yaw = estimate.rotation()(0, 0);
        const double sin_yaw = estimate.rotation()(1, 0);

        std::vector<int> point_index(1);
        std::vector<float> point_squared_distance(1);
        for (const auto &source_pt : cloud_scan_msg->points)
        {
            const double x = source_pt.x;
            const double y = source_pt.y;
            const double tx = estimate.translation().x() + cos_yaw * x - sin_yaw * y;
            const double ty = estimate.translation().y() + sin_yaw * x + cos_yaw * y;

            PointT query;
            query.x = static_cast<float>(tx);
            query.y = static_cast<float>(ty);
            query.z = 0.0f;

            if (local_map_kdtree.nearestKSearch(query, 1, point_index, point_squared_distance) <= 0)
            {
                continue;
            }

            const double dist = std::sqrt(static_cast<double>(point_squared_distance[0]));
            if (dist > max_correspondence_distance)
            {
                continue;
            }

            const PointT &target_pt = cloud_map_msg->points[point_index[0]];
            const double rx = tx - static_cast<double>(target_pt.x);
            const double ry = ty - static_cast<double>(target_pt.y);
            const double residual_sq = rx * rx + ry * ry;
            const double denom = sigma * sigma + residual_sq;
            const double weight = (sigma * sigma) / (denom * denom);

            Eigen::Matrix<double, 2, 3> J;
            J << 1.0, 0.0, -sin_yaw * x - cos_yaw * y,
                0.0, 1.0, cos_yaw * x - sin_yaw * y;
            const Eigen::Vector2d residual(rx, ry);

            JTJ += J.transpose() * weight * J;
            JTr += J.transpose() * weight * residual;
            weighted_residual_sum += weight * residual_sq;
            weighted_residual_weight += weight;
            correspondence_count++;
        }

        if (correspondence_count < 10)
        {
            last_failure_reason_ = "insufficient_correspondences";
            return false;
        }

        const Eigen::Vector3d delta = JTJ.ldlt().solve(-JTr);
        if (!delta.allFinite())
        {
            last_failure_reason_ = "registration_solve_failed";
            return false;
        }

        estimate = ExpSE2(delta.x(), delta.y(), delta.z()) * estimate;

        const double delta_translation = std::hypot(delta.x(), delta.y());
        const double delta_rotation = std::abs(delta.z());
        if (delta_translation < registration_translation_epsilon_ &&
            delta_rotation < registration_rotation_epsilon_)
        {
            break;
        }
    }

    const double x = estimate.translation().x();
    const double y = estimate.translation().y();
    const double yaw = std::atan2(estimate.rotation()(1, 0), estimate.rotation()(0, 0));
    const double tranDist = std::sqrt(x * x + y * y);
    const double angleDist = std::abs(yaw);
    last_icp_score_ = weighted_residual_weight > 1e-9 ? (weighted_residual_sum / weighted_residual_weight) : std::numeric_limits<double>::infinity();
    last_icp_tran_dist_ = tranDist;
    last_icp_angle_dist_ = angleDist;
    last_icp_dx_ = x;
    last_icp_dy_ = y;
    last_icp_dyaw_ = yaw;
    last_registration_correspondence_count_ = correspondence_count;
    last_icp_converged_ = true;
    last_icp_low_points_ = cloud_scan_msg->points.size() < Point_Quantity_THRESHOLD_;
    last_icp_small_correction_ = (tranDist < DIST_THRESHOLD_ && angleDist < ANGLE_THRESHOLD_);
    last_icp_high_score_ = (last_icp_score_ > SCORE_THRESHOLD_MAX_);
    last_icp_large_angle_ = (angleDist > ANGLE_UPPER_THRESHOLD_);

    std::vector<std::string> risk_flags;
    if (last_icp_low_points_)
    {
        risk_flags.emplace_back("low_points");
    }
    if (last_icp_high_score_)
    {
        risk_flags.emplace_back("high_score");
    }
    if (last_icp_large_angle_)
    {
        risk_flags.emplace_back("large_yaw_correction");
    }
    if (last_icp_small_correction_)
    {
        risk_flags.emplace_back("small_correction");
    }
    for (size_t i = 0; i < risk_flags.size(); ++i)
    {
        if (i != 0)
        {
            last_icp_risk_flags_ += "|";
        }
        last_icp_risk_flags_ += risk_flags[i];
    }

    if (if_debug_)
    {
        std::cout << "tranDist:" << tranDist << " angleDist: " << angleDist
                  << " score: " << last_icp_score_
                  << " sigma: " << sigma
                  << " corr: " << correspondence_count
                  << " angle speed: " << latest_odom_angular_z_ << std::endl;
        std::cout << "cloud_scan_msg->points.size() : " << cloud_scan_msg->points.size() << std::endl;
        if (!last_icp_risk_flags_.empty())
        {
            std::cout << "icp risk flags: " << last_icp_risk_flags_ << std::endl;
        }
    }

    // 预测已经足够准确时，ICP 修正量接近 0 是正常现象，不能视为跟踪失败。
    if (tranDist < DIST_THRESHOLD_ && angleDist < ANGLE_THRESHOLD_)
    {
        if (if_debug_)
        {
            std::cout << "\033[1;33m" << "ICP correction is small, keep predicted pose" << "\033[0m" << std::endl;
        }
    }

    trans = BuildPose2D(x, y, yaw);
    UpdateAdaptiveThreshold(trans);

    return true;
}

/**
 * 对点云中障碍点进行剔除
 * cloud_map_msg为参考点云，cloud_msg为需要剔除障碍点的点云
 */
void Scan2MapLocation::PointCloudObstacleRemoval(PointCloudT::Ptr &cloud_map_msg, PointCloudT::Ptr &cloud_msg, double Distance_Threshold)
{
    PointCloudT::Ptr cloud_removaled(new PointCloudT);
    ;
    const double distance_threshold_sq = Distance_Threshold * Distance_Threshold;

    pcl::KdTreeFLANN<PointT> kdtree;     // 创建kd_tree对象
    kdtree.setInputCloud(cloud_map_msg); // 设置搜索空间
    int K = 1;                           // k近邻收索

    // for (int i = 0; i < cloud_msg->points.size(); i++)
    int i = 0;
    while (i < cloud_msg->points.size())
    {
        PointT searchPoint = cloud_msg->points[i];

        // k近邻收索
        std::vector<int> pointIdxNKNSearch(K);         // 存储查询点近邻索引
        std::vector<float> pointNKNSquaredDistance(K); // 存储近邻点对应平方距离

        if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            if (pointNKNSquaredDistance[0] > distance_threshold_sq)
            {
                // 大于阈值认为是障碍点，剔除
                cloud_msg->erase(cloud_msg->begin() + i);
                cloud_removaled->push_back(searchPoint); // 从点云最后面插入一点
            }
            else
            {
                i++;
            }
        }
        else
        {
            i++;
        }
    }
    last_removed_count_ = cloud_removaled->points.size();

    cloud_removaled->width = cloud_removaled->points.size();
    cloud_removaled->height = 1;
    cloud_removaled->is_dense = false; // contains nans

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    // header.frame_id = lidar_frame_;
    header.frame_id = "map";
    cloud_removaled->header = pcl_conversions::toPCL(header);
    removal_pointcloud_publisher_.publish(cloud_removaled);
    // std::cout<<"size of clound ObstacleRemovaled : " << cloud_msg->points.size() << std::endl;
}

/**
 * 对点云进行离群点滤波剔除离群点
 */
void Scan2MapLocation::PointCloudOutlierRemoval(PointCloudT::Ptr &cloud_msg)
{
    if (!cloud_msg || static_cast<int>(cloud_msg->points.size()) < outlier_mean_k_)
    {
        return;
    }

    pcl::StatisticalOutlierRemoval<PointT> sor_OutRemove;
    /* 设置输入点云 */
    sor_OutRemove.setInputCloud(cloud_msg);
    /* 设置在进行统计时考虑查询点邻近点数 */
    sor_OutRemove.setMeanK(outlier_mean_k_);
    /* 设置判断是否为离群点 的 阈值  设置为1的 话 表示为：如果一个点的距离超过平均距离一个标准差以上则为离群点 */
    sor_OutRemove.setStddevMulThresh(outlier_stddev_mul_);
    /* 执行滤波 返回 滤波后 的 点云 */
    sor_OutRemove.filter(*cloud_msg);
}

/**
 * 对点云进行体素滤波剔除离群点
 */
void Scan2MapLocation::PointCloudVoxelGridRemoval(PointCloudT::Ptr &cloud_msg, double leafSize)
{
    // 体素滤波
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud_msg);
    voxel_grid.setLeafSize(leafSize, leafSize, leafSize); // 注：体素不一定是正方体，体素可以是长宽高不同的长方体
    voxel_grid.filter(*cloud_msg);
}

/**
 * 将占据栅格数据转为pointcloud点云格式数据
 */
void Scan2MapLocation::OccupancyGridToPointCloud(const nav_msgs::OccupancyGrid::ConstPtr &map_msg, PointCloudT::Ptr &cloud_msg)
{
    if (if_debug_)
    {
        std::cout << "start OccupancyGrid to PointCloud......" << std::endl;
    }

    float ori_posx = map_msg->info.origin.position.x; // 初始坐标
    float ori_posy = map_msg->info.origin.position.y;
    float resolution = map_msg->info.resolution; // 分辨率
    float width = map_msg->info.width;           // 地图长宽
    float height = map_msg->info.height;

    std_msgs::Header header;
    header.stamp = ros::Time(0.0);
    header.frame_id = "map";

    // OccupancyGrid转换为pcl点云
    cloud_msg->height = 1;
    cloud_msg->is_dense = false;
    cloud_msg->header = pcl_conversions::toPCL(header);

    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            //@TODO
            if (map_msg->data[x + y * width] == 100)
            {
                // 首先声明一个 cloud_msg第i个点的 引用
                PointT point_tmp;

                // 获取第点在笛卡尔坐标系下的坐标
                point_tmp.x = (.5f + x) * resolution + ori_posx;
                point_tmp.y = (.5f + y) * resolution + ori_posy;
                point_tmp.z = 0;

                cloud_msg->points.push_back(point_tmp);
            }
        }
    cloud_msg->width = cloud_msg->points.size();
}

/**
 * 将scan数据转为pointcloud点云格式数据
 * 并转换到map坐标系下
 * 需要提前获得map_to_lidar_的变换
 */
void Scan2MapLocation::ScanToPointCloudInReference(const sensor_msgs::LaserScan::ConstPtr &scan_msg, PointCloudT::Ptr &cloud_msg)
{
    last_scan_raw_count_ = scan_msg->ranges.size();
    last_scan_start_time_ = scan_msg->header.stamp.toSec();
    last_scan_duration_ = ComputeScanDuration(scan_msg);
    last_scan_point_time_increment_ = ComputeScanPointTimeIncrement(scan_msg);
    last_scan_end_time_ = last_scan_start_time_ + last_scan_duration_;
    last_scan_reference_offset_ = scan_time_ - scan_msg->header.stamp.toSec();
    // 计算雷达数据点长度
    unsigned int point_len = 0;
    for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
    {
        // 获取scan的第i个点的距离值
        float range = scan_msg->ranges[i];
        // std::cout << "range = " << range << std::endl;

        // 将 inf 与 nan 点 设置为无效点
        if (!std::isfinite(range))
        {
            continue;
        }
        // 有些雷达驱动会将无效点设置成 range_max+1
        // 所以要根据雷达的range_min与range_max进行有效值的判断
        if ((range > scan_msg->range_min) && (range < scan_msg->range_max) && (range > Scan_Range_Min) && (range < Scan_Range_Max))
        {
            point_len += 1;
        }
    }

    // 对容器进行初始化
    unsigned int point_num = 0;
    cloud_msg->points.resize(point_len);
    last_scan_valid_count_ = point_len;

    std::deque<nav_msgs::Odometry> odom_queue_snapshot;
    Eigen::Isometry3d odom_pose_ref = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d odom_pose_start = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d odom_pose_end = Eigen::Isometry3d::Identity();
    bool can_deskew = false;
    const double scan_header_time = last_scan_start_time_;
    if (enable_scan_deskew_ && point_len > 0 && last_scan_duration_ > 1e-6 && last_scan_point_time_increment_ > 0.0)
    {
        const ros::Time wait_deadline = ros::Time::now() + ros::Duration(scan_odom_wait_timeout_);
        while (ros::ok())
        {
            {
                std::lock_guard<std::mutex> lock(odom_lock_);
                odom_queue_snapshot = odom_queue_;
            }
            if (!odom_queue_snapshot.empty() &&
                odom_queue_snapshot.front().header.stamp.toSec() <= scan_header_time &&
                odom_queue_snapshot.back().header.stamp.toSec() >= last_scan_end_time_ &&
                LookupOdomPose(odom_pose_ref, scan_time_, odom_queue_snapshot) &&
                LookupOdomPose(odom_pose_start, scan_header_time, odom_queue_snapshot) &&
                LookupOdomPose(odom_pose_end, last_scan_end_time_, odom_queue_snapshot))
            {
                can_deskew = true;
                const Eigen::Isometry3d scan_motion = odom_pose_start.inverse() * odom_pose_end;
                last_scan_motion_translation_ = scan_motion.translation().head<2>().norm();
                last_scan_motion_yaw_ = std::atan2(scan_motion.rotation()(1, 0), scan_motion.rotation()(0, 0));
                last_scan_odom_coverage_ok_ = true;
                break;
            }
            if (ros::Time::now() >= wait_deadline)
            {
                break;
            }
            last_scan_odom_wait_count_++;
            ros::Duration(0.005).sleep();
        }
    }
    last_scan_deskew_applied_ = can_deskew;
    last_scan_deskew_point_count_ = 0;
    last_scan_deskew_failed_point_count_ = 0;

    tran_start_time_ = std::chrono::steady_clock::now();
    for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
    {
        // 获取scan的第i个点的距离值
        float range = scan_msg->ranges[i];
        // std::cout << "range = " << range << std::endl;

        // 将 inf 与 nan 点 设置为无效点
        if (!std::isfinite(range))
        {
            continue;
        }

        // 有些雷达驱动会将无效点设置成 range_max+1
        // 所以要根据雷达的range_min与range_max进行有效值的判断
        if (range > scan_msg->range_min && range < scan_msg->range_max && range > Scan_Range_Min && range < Scan_Range_Max)
        {
            // 首先声明一个 cloud_msg第i个点的 引用
            PointT &point_tmp = cloud_msg->points[point_num];

            // 获取第i个点对应的角度
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            // 获取第i个点在笛卡尔坐标系下的坐标
            // point_tmp.x = range * cos(angle);
            // point_tmp.y = range * sin(angle);
            // point_tmp.z = 0.0;

            Eigen::Vector3d point_vector(range * cos(angle), range * sin(angle), 0.0);
            if (can_deskew)
            {
                const double point_time = scan_header_time + static_cast<double>(i) * last_scan_point_time_increment_;
                Eigen::Isometry3d odom_pose_point = Eigen::Isometry3d::Identity();
                if (LookupOdomPose(odom_pose_point, point_time, odom_queue_snapshot))
                {
                    const Eigen::Isometry3d lidar_pose_point = odom_pose_point * base_to_lidar_;
                    const Eigen::Isometry3d lidar_pose_ref = odom_pose_ref * base_to_lidar_;
                    const Eigen::Isometry3d point_to_ref = lidar_pose_ref.inverse() * lidar_pose_point;
                    point_vector = point_to_ref * point_vector;
                    last_scan_deskew_point_count_++;
                }
                else
                {
                    last_scan_deskew_failed_point_count_++;
                }
            }

            // 获取参考雷达坐标系下在笛卡尔坐标系下的坐标
            point_tmp.x = point_vector(0);
            point_tmp.y = point_vector(1);
            point_tmp.z = 0.0;

            point_num += 1;
        }
    }

    tran_end_time_ = std::chrono::steady_clock::now();
    tran_time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(tran_end_time_ - tran_start_time_);
    // std::cout << "雷达数据转换后半处理用时: " << tran_time_used_.count() << " 秒。" << std::endl;

    // cloud_msg->width = scan_msg->ranges.size();
    cloud_msg->width = point_len;
    cloud_msg->height = 1;
    cloud_msg->is_dense = false; // contains nans

    std_msgs::Header header;
    header.stamp = ros::Time(scan_time_);
    header.frame_id = lidar_frame_;
    cloud_msg->header = pcl_conversions::toPCL(header);
}

void Scan2MapLocation::TransformPointCloudToMap(const PointCloudT::Ptr &source_cloud, const Eigen::Isometry3d &map_to_lidar,
                                                const ros::Time &stamp, PointCloudT::Ptr &target_cloud) const
{
    if (!target_cloud)
    {
        target_cloud.reset(new PointCloudT);
    }
    pcl::transformPointCloud(*source_cloud, *target_cloud, map_to_lidar.matrix().cast<float>());
    target_cloud->width = target_cloud->points.size();
    target_cloud->height = 1;
    target_cloud->is_dense = false;

    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = map_frame_;
    target_cloud->header = pcl_conversions::toPCL(header);
}

void Scan2MapLocation::ExtractLocalMap(const Eigen::Isometry3d &map_to_base, PointCloudT::Ptr &local_map, double radius) const
{
    if (!local_map)
    {
        local_map.reset(new PointCloudT);
    }
    local_map->clear();
    if (!cloud_map_ || cloud_map_->empty())
    {
        return;
    }

    const double cx = map_to_base.translation().x();
    const double cy = map_to_base.translation().y();
    const double effective_radius = radius > 0.0 ? radius : local_map_radius_;
    const double radius_sq = effective_radius * effective_radius;

    local_map->points.reserve(cloud_map_->points.size());
    for (const auto &pt : cloud_map_->points)
    {
        const double dx = static_cast<double>(pt.x) - cx;
        const double dy = static_cast<double>(pt.y) - cy;
        if (dx * dx + dy * dy <= radius_sq)
        {
            local_map->points.push_back(pt);
        }
    }

    local_map->width = local_map->points.size();
    local_map->height = 1;
    local_map->is_dense = false;

    std_msgs::Header header;
    header.stamp = ros::Time(scan_time_);
    header.frame_id = map_frame_;
    local_map->header = pcl_conversions::toPCL(header);
}

double Scan2MapLocation::ScorePoseCandidate(const PointCloudT::Ptr &scan_cloud_reference,
                                            const Eigen::Isometry3d &candidate_map_to_lidar,
                                            const pcl::KdTreeFLANN<PointT> &score_kdtree) const
{
    if (!scan_cloud_reference || scan_cloud_reference->empty())
    {
        return std::numeric_limits<double>::infinity();
    }

    PointCloudT candidate_cloud;
    pcl::transformPointCloud(*scan_cloud_reference, candidate_cloud, candidate_map_to_lidar.matrix().cast<float>());
    if (candidate_cloud.empty())
    {
        return std::numeric_limits<double>::infinity();
    }

    const double max_dist_sq = search_score_max_distance_ * search_score_max_distance_;
    const double max_dist = search_score_max_distance_;
    std::vector<double> point_distances;
    point_distances.reserve(candidate_cloud.points.size());
    double weighted_distance_sum = 0.0;
    double weighted_count = 0.0;
    double weighted_inlier_sum = 0.0;

    std::vector<int> point_index(1);
    std::vector<float> point_squared_distance(1);
    for (size_t idx = 0; idx < candidate_cloud.points.size(); ++idx)
    {
        const auto &point = candidate_cloud.points[idx];
        const auto &reference_point = scan_cloud_reference->points[idx];
        const double radius = std::hypot(static_cast<double>(reference_point.x),
                                         static_cast<double>(reference_point.y));
        const double range_weight = 0.6 + std::min(radius, 3.0) / 2.0;
        const double yaw_weight = 0.7 + std::min(radius, 2.5);
        const double point_weight = 0.35 * range_weight + 0.65 * yaw_weight;

        double dist = max_dist;
        if (score_kdtree.nearestKSearch(point, 1, point_index, point_squared_distance) > 0)
        {
            dist = std::sqrt(std::min(static_cast<double>(point_squared_distance[0]), max_dist_sq));
        }
        point_distances.push_back(dist);
        weighted_distance_sum += point_weight * dist;
        weighted_count += point_weight;
        if (dist < 0.10)
        {
            weighted_inlier_sum += point_weight;
        }
    }

    if (point_distances.empty() || weighted_count <= 1e-9)
    {
        return std::numeric_limits<double>::infinity();
    }

    std::sort(point_distances.begin(), point_distances.end());
    const size_t p90_index = std::min(point_distances.size() - 1,
                                      static_cast<size_t>(0.90 * static_cast<double>(point_distances.size())));
    const double p90_distance = point_distances[p90_index];
    const double mean_distance = weighted_distance_sum / weighted_count;
    const double weighted_inlier_ratio = weighted_inlier_sum / weighted_count;
    return mean_distance + 0.35 * p90_distance + 0.08 * (1.0 - weighted_inlier_ratio);
}

bool Scan2MapLocation::SearchPoseCandidate(const PointCloudT::Ptr &scan_cloud_reference,
                                           const Eigen::Isometry3d &predicted_map_to_base,
                                           Eigen::Isometry3d &searched_map_to_base)
{
    last_search_active_ = false;
    last_search_applied_ = false;
    last_search_candidate_count_ = 0;
    last_search_predicted_score_ = std::numeric_limits<double>::quiet_NaN();
    last_search_dx_ = 0.0;
    last_search_dy_ = 0.0;
    last_search_dyaw_ = 0.0;
    last_search_score_ = std::numeric_limits<double>::quiet_NaN();
    last_startup_relocalization_active_ = false;

    if (!scan_cloud_reference || scan_cloud_reference->empty() || !map_kdtree_initialized_)
    {
        searched_map_to_base = predicted_map_to_base;
        return false;
    }

    PointCloudT::Ptr score_cloud(new PointCloudT(*scan_cloud_reference));
    PointCloudVoxelGridRemoval(score_cloud, search_score_leaf_size_);
    if (score_cloud->empty())
    {
        searched_map_to_base = predicted_map_to_base;
        return false;
    }

    const bool use_heading_bootstrap = enable_initial_global_yaw_search_ &&
                                       require_heading_bootstrap_ &&
                                       tracking_frame_count_ < static_cast<size_t>(initial_global_yaw_search_frames_);
    const bool use_startup_relocalization = enable_startup_relocalization_ &&
                                            !use_heading_bootstrap &&
                                            tracking_frame_count_ < static_cast<size_t>(startup_relocalization_frames_);
    const bool use_dynamic_yaw_search = enable_yaw_search_ &&
                                        (std::abs(latest_odom_angular_z_) >= yaw_search_trigger_angular_z_ ||
                                         std::abs(last_odom_delta_yaw_) >= yaw_search_trigger_odom_yaw_ ||
                                         tracking_frame_count_ < static_cast<size_t>(static_lock_warmup_frames_) ||
                                         !std::isfinite(last_icp_score_) ||
                                         last_icp_high_score_);
    const bool use_pose_search = use_heading_bootstrap ||
                                 use_startup_relocalization ||
                                 use_dynamic_yaw_search;
    last_search_active_ = use_pose_search;
    last_startup_relocalization_active_ = use_startup_relocalization;

    const double coarse_trans_range = use_startup_relocalization ? startup_relocalization_translation_range_ : search_translation_range_;
    const double coarse_trans_step = std::max(use_startup_relocalization ? 0.01 : 0.02,
                                              use_startup_relocalization ? startup_relocalization_translation_step_ : search_translation_step_);
    const double coarse_yaw_step = use_heading_bootstrap
                                       ? std::max(M_PI / 180.0, initial_global_yaw_search_step_deg_ * M_PI / 180.0)
                                       : std::max(use_startup_relocalization ? (0.5 * M_PI / 180.0) : (1.0 * M_PI / 180.0),
                                                  (use_startup_relocalization ? startup_relocalization_yaw_step_deg_ : yaw_search_step_deg_) * M_PI / 180.0);
    const double coarse_yaw_range = (use_heading_bootstrap
                                         ? initial_global_yaw_search_range_deg_
                                         : (use_startup_relocalization ? startup_relocalization_yaw_range_deg_ : yaw_search_range_deg_)) *
                                    M_PI / 180.0;
    const double accept_score_threshold = use_startup_relocalization ? startup_relocalization_accept_score_threshold_ : search_accept_score_threshold_;
    const double accept_improvement_threshold = use_startup_relocalization ? startup_relocalization_accept_improvement_threshold_ : search_accept_improvement_threshold_;
    const double accept_improvement_ratio = use_startup_relocalization ? startup_relocalization_accept_improvement_ratio_ : search_accept_improvement_ratio_;

    PointCloudT::Ptr score_map(new PointCloudT);
    const double search_map_radius = local_map_radius_ + std::max(1.0, coarse_trans_range + 0.5);
    ExtractLocalMap(predicted_map_to_base, score_map, search_map_radius);

    pcl::KdTreeFLANN<PointT> score_kdtree;
    if (score_map && !score_map->empty())
    {
        score_kdtree.setInputCloud(score_map);
    }
    else if (map_kdtree_initialized_)
    {
        score_kdtree.setInputCloud(cloud_map_);
    }
    else
    {
        searched_map_to_base = predicted_map_to_base;
        return false;
    }

    double pred_x, pred_y, pred_yaw;
    ExtractPose2D(predicted_map_to_base, pred_x, pred_y, pred_yaw);

    const Eigen::Isometry3d predicted_map_to_lidar = predicted_map_to_base * base_to_lidar_;
    const double predicted_score = ScorePoseCandidate(score_cloud, predicted_map_to_lidar, score_kdtree);
    last_search_predicted_score_ = predicted_score;
    last_search_score_ = predicted_score;
    last_search_candidate_count_ = 1;

    if (!use_pose_search)
    {
        searched_map_to_base = predicted_map_to_base;
        return false;
    }

    double best_score = predicted_score;
    Eigen::Isometry3d best_pose = predicted_map_to_base;
    bool found = std::isfinite(predicted_score);
    size_t candidate_count = 1;

    auto evaluate_grid = [&](double center_x, double center_y, double center_yaw,
                             double trans_range, double trans_step,
                             double yaw_range, double yaw_step)
    {
        for (double yaw_offset = -yaw_range; yaw_offset <= yaw_range + 1e-9; yaw_offset += yaw_step)
        {
            for (double dx_local = -trans_range; dx_local <= trans_range + 1e-9; dx_local += trans_step)
            {
                for (double dy_local = -trans_range; dy_local <= trans_range + 1e-9; dy_local += trans_step)
                {
                    const double x = center_x + std::cos(center_yaw) * dx_local - std::sin(center_yaw) * dy_local;
                    const double y = center_y + std::sin(center_yaw) * dx_local + std::cos(center_yaw) * dy_local;
                    const double yaw = center_yaw + yaw_offset;
                    Eigen::Isometry3d candidate_map_to_base = BuildPose2D(x, y, yaw);
                    const Eigen::Isometry3d candidate_map_to_lidar = candidate_map_to_base * base_to_lidar_;
                    const double candidate_score = ScorePoseCandidate(score_cloud, candidate_map_to_lidar, score_kdtree);
                    ++candidate_count;
                    if (!std::isfinite(candidate_score))
                    {
                        continue;
                    }
                    if (!found || candidate_score < best_score)
                    {
                        best_score = candidate_score;
                        best_pose = candidate_map_to_base;
                        found = true;
                    }
                }
            }
        }
    };

    evaluate_grid(pred_x, pred_y, pred_yaw,
                  coarse_trans_range, coarse_trans_step,
                  coarse_yaw_range, coarse_yaw_step);

    if (found)
    {
        double best_x, best_y, best_yaw;
        ExtractPose2D(best_pose, best_x, best_y, best_yaw);
        const double fine_trans_floor = use_startup_relocalization ? 0.01 : 0.02;
        const double fine_yaw_floor = use_startup_relocalization ? (0.5 * M_PI / 180.0) : (1.0 * M_PI / 180.0);
        const double fine_trans_step = std::max(fine_trans_floor, 0.5 * coarse_trans_step);
        const double fine_trans_range = std::max(fine_trans_step, 0.75 * coarse_trans_step);
        const double fine_yaw_step = use_heading_bootstrap
                                         ? std::max(fine_yaw_floor, initial_global_yaw_search_step_deg_ * 0.5 * M_PI / 180.0)
                                         : std::max(fine_yaw_floor, 0.5 * coarse_yaw_step);
        const double fine_yaw_range = use_heading_bootstrap
                                          ? std::max(fine_yaw_step, initial_global_yaw_search_step_deg_ * M_PI / 180.0)
                                          : std::max(fine_yaw_step, 0.75 * coarse_yaw_step);
        evaluate_grid(best_x, best_y, best_yaw,
                      fine_trans_range, fine_trans_step,
                      fine_yaw_range, fine_yaw_step);
    }

    if (found)
    {
        double best_x, best_y, best_yaw;
        ExtractPose2D(best_pose, best_x, best_y, best_yaw);
        const double dx = best_x - pred_x;
        const double dy = best_y - pred_y;
        last_search_dx_ = std::cos(pred_yaw) * dx + std::sin(pred_yaw) * dy;
        last_search_dy_ = -std::sin(pred_yaw) * dx + std::cos(pred_yaw) * dy;
        last_search_dyaw_ = std::atan2(std::sin(best_yaw - pred_yaw), std::cos(best_yaw - pred_yaw));
    }

    const double improvement_abs = predicted_score - best_score;
    const double improvement_ratio = (std::isfinite(predicted_score) && predicted_score > 1e-6)
                                         ? (best_score / predicted_score)
                                         : 1.0;
    const bool accept_search = found &&
                               candidate_count > 1 &&
                               std::isfinite(best_score) &&
                               best_score <= accept_score_threshold &&
                               (improvement_abs >= accept_improvement_threshold ||
                                improvement_ratio <= accept_improvement_ratio);

    last_search_applied_ = accept_search;
    last_search_candidate_count_ = candidate_count;
    last_search_score_ = best_score;
    if (use_heading_bootstrap && accept_search)
    {
        require_heading_bootstrap_ = false;
    }
    searched_map_to_base = accept_search ? best_pose : predicted_map_to_base;
    return accept_search;
}

/**
 * 从odom中读取start_time 到end_time时间段的坐标变换 保存到trans中
 * trans 类型： Eigen::Isometry3d
 */
bool Scan2MapLocation::GetOdomTransform(Eigen::Isometry3d &trans, double start_time, double end_time)
{
    ros::Rate loop_rate(100);
    std::deque<nav_msgs::Odometry> odom_queue;
    int wait_count = 0;
    while (ros::ok())
    {
        {
            std::lock_guard<std::mutex> lock(odom_lock_);
            if (!odom_queue_.empty())
            {
                last_odom_front_time_ = odom_queue_.front().header.stamp.toSec();
                last_odom_back_time_ = odom_queue_.back().header.stamp.toSec();
            }
            if (!odom_queue_.empty() &&
                odom_queue_.back().header.stamp.toSec() >= end_time)
            {
                odom_queue = odom_queue_;
                break;
            }

            // ROS_WARN_THROTTLE(1.0, "wait for Odometry data ...");
            wait_count++;
            if (if_debug_ && !odom_queue_.empty())
            {
                std::cout << "time of back :" << odom_queue_.back().header.stamp.toSec() << std::endl;
                std::cout << "time of end_time :" << end_time << std::endl;
            }
        }
        loop_rate.sleep();
    }
    last_odom_wait_count_ = wait_count;

    if (!ros::ok())
    {
        last_failure_reason_ = "ros_shutdown_while_waiting_odom";
        return false;
    }

    // odom数据队列的头尾的时间戳要在雷达数据的时间段外
    if (odom_queue.empty() ||
        odom_queue.front().header.stamp.toSec() > start_time)
    {
        last_failure_reason_ = "odom_queue_does_not_cover_start_time";
        ROS_WARN("start_time out of Odometry data ...");
        return false;
    }
    if (odom_queue.back().header.stamp.toSec() < end_time)
    {
        last_failure_reason_ = "odom_queue_does_not_cover_end_time";
        ROS_WARN("end_time out of Odometry data ...");
        return false;
    }

    if (if_debug_)
    {
        std::cout << "time of front :" << odom_queue.front().header.stamp.toSec() << std::endl;
        std::cout << "time of back :" << odom_queue.back().header.stamp.toSec() << std::endl;
        std::cout << "time of start_time :" << start_time << std::endl;
        std::cout << "time of end_time :" << end_time << std::endl;
    }

    Eigen::Isometry3d transBegin = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d transEnd = Eigen::Isometry3d::Identity();
    if (!LookupOdomPose(transBegin, start_time, odom_queue))
    {
        last_failure_reason_ = "odom_lookup_start_failed";
        return false;
    }
    if (!LookupOdomPose(transEnd, end_time, odom_queue))
    {
        last_failure_reason_ = "odom_lookup_end_failed";
        return false;
    }

    last_odom_start_used_time_ = start_time;
    last_odom_end_used_time_ = end_time;
    last_odom_dt_ = end_time - start_time;
    last_odom_start_x_ = transBegin.translation().x();
    last_odom_start_y_ = transBegin.translation().y();
    last_odom_start_yaw_ = std::atan2(transBegin.rotation()(1, 0), transBegin.rotation()(0, 0));
    last_odom_end_x_ = transEnd.translation().x();
    last_odom_end_y_ = transEnd.translation().y();
    last_odom_end_yaw_ = std::atan2(transEnd.rotation()(1, 0), transEnd.rotation()(0, 0));

    // 计算上一帧 base_link 到当前帧 base_link 的相对运动
    trans = transBegin.inverse() * transEnd;
    last_odom_delta_x_ = trans.translation().x();
    last_odom_delta_y_ = trans.translation().y();
    last_odom_delta_yaw_ = std::atan2(trans.rotation()(1, 0), trans.rotation()(0, 0));
    trans = BuildPose2D(last_odom_delta_x_, last_odom_delta_y_, last_odom_delta_yaw_);
    if (if_debug_)
    {
        std::cout << "trans matrix =  \n"
                  << trans.matrix() << std::endl;
    }
    return true;
}

bool Scan2MapLocation::LookupOdomPose(Eigen::Isometry3d &pose, double stamp, const std::deque<nav_msgs::Odometry> &odom_queue) const
{
    if (odom_queue.empty())
    {
        return false;
    }

    auto build_pose = [&](const nav_msgs::Odometry &msg)
    {
        tf2::Quaternion quat(msg.pose.pose.orientation.x,
                             msg.pose.pose.orientation.y,
                             msg.pose.pose.orientation.z,
                             msg.pose.pose.orientation.w);
        const double yaw = tf2::getYaw(quat);
        return BuildPose2D(msg.pose.pose.position.x,
                           msg.pose.pose.position.y,
                           yaw);
    };

    if (stamp <= odom_queue.front().header.stamp.toSec())
    {
        pose = build_pose(odom_queue.front());
        return true;
    }
    if (stamp >= odom_queue.back().header.stamp.toSec())
    {
        pose = build_pose(odom_queue.back());
        return true;
    }

    for (size_t i = 1; i < odom_queue.size(); ++i)
    {
        const auto &prev = odom_queue[i - 1];
        const auto &next = odom_queue[i];
        const double prev_time = prev.header.stamp.toSec();
        const double next_time = next.header.stamp.toSec();
        if (stamp > next_time)
        {
            continue;
        }

        const double duration = next_time - prev_time;
        const double ratio = duration > 1e-6 ? (stamp - prev_time) / duration : 0.0;

        tf2::Quaternion quat_prev(prev.pose.pose.orientation.x,
                                  prev.pose.pose.orientation.y,
                                  prev.pose.pose.orientation.z,
                                  prev.pose.pose.orientation.w);
        tf2::Quaternion quat_next(next.pose.pose.orientation.x,
                                  next.pose.pose.orientation.y,
                                  next.pose.pose.orientation.z,
                                  next.pose.pose.orientation.w);
        tf2::Quaternion quat_interp = quat_prev.slerp(quat_next, ratio);
        const double x_interp = prev.pose.pose.position.x +
                                ratio * (next.pose.pose.position.x - prev.pose.pose.position.x);
        const double y_interp = prev.pose.pose.position.y +
                                ratio * (next.pose.pose.position.y - prev.pose.pose.position.y);
        const double yaw_interp = tf2::getYaw(quat_interp);

        pose = BuildPose2D(x_interp, y_interp, yaw_interp);
        return true;
    }

    return false;
}

double Scan2MapLocation::ComputeAdaptiveThreshold() const
{
    return std::sqrt(std::max(1e-6, adaptive_model_sse_ / std::max(1, adaptive_model_samples_)));
}

void Scan2MapLocation::UpdateAdaptiveThreshold(const Eigen::Isometry3d &model_deviation)
{
    const double delta_translation = std::hypot(model_deviation.translation().x(),
                                                model_deviation.translation().y());
    const double delta_yaw = std::abs(std::atan2(model_deviation.rotation()(1, 0),
                                                 model_deviation.rotation()(0, 0)));
    const double delta_rotation = 2.0 * Scan_Range_Max * std::sin(0.5 * delta_yaw);
    const double model_error = delta_translation + delta_rotation;
    if (model_error > adaptive_threshold_min_motion_)
    {
        adaptive_model_sse_ += model_error * model_error;
        adaptive_model_samples_++;
    }
}

Eigen::Isometry3d Scan2MapLocation::ExpSE2(double dx, double dy, double dtheta)
{
    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
    delta.rotate(Eigen::AngleAxisd(dtheta, Eigen::Vector3d::UnitZ()));
    delta.pretranslate(Eigen::Vector3d(dx, dy, 0.0));
    return delta;
}

double Scan2MapLocation::ComputeScanDuration(const sensor_msgs::LaserScan::ConstPtr &scan_msg) const
{
    if (scan_msg->scan_time > 1e-6)
    {
        return scan_msg->scan_time;
    }
    if (scan_msg->time_increment > 1e-6 && scan_msg->ranges.size() > 1)
    {
        return scan_msg->time_increment * static_cast<double>(scan_msg->ranges.size() - 1);
    }
    return 0.0;
}

double Scan2MapLocation::ComputeScanPointTimeIncrement(const sensor_msgs::LaserScan::ConstPtr &scan_msg) const
{
    if (scan_msg->time_increment > 1e-9)
    {
        return scan_msg->time_increment;
    }
    if (scan_msg->ranges.size() > 1)
    {
        const double scan_duration = ComputeScanDuration(scan_msg);
        if (scan_duration > 1e-6)
        {
            return scan_duration / static_cast<double>(scan_msg->ranges.size() - 1);
        }
    }
    return 0.0;
}

double Scan2MapLocation::ComputeScanReferenceTime(const sensor_msgs::LaserScan::ConstPtr &scan_msg) const
{
    const double scan_start_time = scan_msg->header.stamp.toSec();
    const double scan_duration = ComputeScanDuration(scan_msg);
    if (!use_scan_midpoint_time_ || scan_duration <= 1e-6)
    {
        return scan_start_time;
    }
    return scan_start_time + 0.5 * scan_duration;
}

void Scan2MapLocation::PublishDebugPose(const ros::Publisher &publisher, const Eigen::Isometry3d &pose,
                                        const ros::Time &stamp, const std::string &frame_id)
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    Eigen::Quaterniond q(pose.rotation());
    msg.pose.position.x = pose.translation().x();
    msg.pose.position.y = pose.translation().y();
    msg.pose.position.z = pose.translation().z();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    publisher.publish(msg);
}

void Scan2MapLocation::PublishLocalizationDebugState(size_t frame_id, const ros::Time &stamp,
                                                     const std::string &stage, const std::string &status,
                                                     const std::string &reason, const Eigen::Isometry3d *predicted_pose,
                                                     const Eigen::Isometry3d *final_pose)
{
    if (publish_localization_state_)
    {
        std_msgs::String state_msg;
        std::ostringstream summary;
        summary << "frame=" << frame_id
                << " stage=" << stage
                << " status=" << status
                << " reason=" << reason;
        state_msg.data = summary.str();
        localization_state_publisher_.publish(state_msg);
    }

    if (!publish_localization_diagnostics_)
    {
        return;
    }

    diagnostic_msgs::DiagnosticArray diag_array;
    diag_array.header.stamp = stamp;

    diagnostic_msgs::DiagnosticStatus diag_status;
    diag_status.name = ros::this_node::getName() + "/localization";
    diag_status.hardware_id = map_frame_ + "->" + base_frame_;
    if (status == "FAIL")
    {
        diag_status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    }
    else if (status == "RISK")
    {
        diag_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    }
    else
    {
        diag_status.level = diagnostic_msgs::DiagnosticStatus::OK;
    }
    diag_status.message = reason.empty() ? status : (status + "|" + reason);

    auto add_kv = [&](const std::string &key, const std::string &value)
    {
        diagnostic_msgs::KeyValue kv;
        kv.key = key;
        kv.value = value;
        diag_status.values.push_back(kv);
    };

    auto add_double = [&](const std::string &key, double value, int precision = 6)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision) << value;
        add_kv(key, oss.str());
    };

    auto add_size = [&](const std::string &key, size_t value)
    {
        add_kv(key, std::to_string(value));
    };

    add_kv("stage", stage);
    add_kv("status", status);
    add_kv("reason", reason);
    add_kv("map_frame", map_frame_);
    add_kv("odom_frame", odom_frame_);
    add_kv("base_frame", base_frame_);
    add_size("frame_id", frame_id);
    add_double("scan_stamp", scan_time_);
    add_double("scan_age", last_scan_age_, 3);
    add_double("scan_duration", last_scan_duration_, 3);
    add_double("scan_dt", last_scan_point_time_increment_, 6);
    add_double("scan_ref_offset", last_scan_reference_offset_, 3);
    add_double("scan_motion_translation", last_scan_motion_translation_, 3);
    add_double("scan_motion_yaw", last_scan_motion_yaw_, 3);
    add_kv("scan_odom_cover", last_scan_odom_coverage_ok_ ? "1" : "0");
    add_kv("deskew", last_scan_deskew_applied_ ? "1" : "0");
    add_size("deskew_points", last_scan_deskew_point_count_);
    add_size("deskew_failed_points", last_scan_deskew_failed_point_count_);
    add_double("odom_dt", last_odom_dt_, 3);
    add_double("odom_delta_x", last_odom_delta_x_, 3);
    add_double("odom_delta_y", last_odom_delta_y_, 3);
    add_double("odom_delta_yaw", last_odom_delta_yaw_, 3);
    add_size("raw_points", last_scan_raw_count_);
    add_size("valid_points", last_scan_valid_count_);
    add_size("voxel_points", last_scan_voxel_count_);
    add_size("used_points", last_scan_used_count_);
    add_size("removed_points", last_removed_count_);
    add_kv("search_active", last_search_active_ ? "1" : "0");
    add_kv("startup_relocalization", last_startup_relocalization_active_ ? "1" : "0");
    add_kv("search_apply", last_search_applied_ ? "1" : "0");
    add_size("search_candidates", last_search_candidate_count_);
    add_double("search_pred_score", last_search_predicted_score_, 6);
    add_double("search_score", last_search_score_, 6);
    add_double("search_dx", last_search_dx_, 3);
    add_double("search_dy", last_search_dy_, 3);
    add_double("search_dyaw", last_search_dyaw_, 3);
    add_double("icp_score", last_icp_score_, 6);
    add_double("icp_dx", last_icp_dx_, 3);
    add_double("icp_dy", last_icp_dy_, 3);
    add_double("icp_dyaw", last_icp_dyaw_, 3);
    add_double("reg_sigma", last_registration_sigma_, 6);
    add_double("reg_max_corr", last_registration_max_correspondence_, 6);
    add_size("reg_corr", last_registration_correspondence_count_);
    add_kv("static_lock", last_static_lock_applied_ ? "1" : "0");

    if (predicted_pose != nullptr)
    {
        double x, y, yaw;
        ExtractPose2D(*predicted_pose, x, y, yaw);
        add_double("pred_x", x, 3);
        add_double("pred_y", y, 3);
        add_double("pred_yaw", yaw, 3);
    }

    if (final_pose != nullptr)
    {
        double x, y, yaw;
        ExtractPose2D(*final_pose, x, y, yaw);
        add_double("final_x", x, 3);
        add_double("final_y", y, 3);
        add_double("final_yaw", yaw, 3);
    }

    diag_array.status.push_back(diag_status);
    localization_diagnostics_publisher_.publish(diag_array);
}

void Scan2MapLocation::ExtractPose2D(const Eigen::Isometry3d &pose, double &x, double &y, double &yaw)
{
    x = pose.translation().x();
    y = pose.translation().y();
    yaw = std::atan2(pose.rotation()(1, 0), pose.rotation()(0, 0));
}

Eigen::Isometry3d Scan2MapLocation::BuildPose2D(double x, double y, double yaw)
{
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    pose.pretranslate(Eigen::Vector3d(x, y, 0.0));
    return pose;
}

double Scan2MapLocation::NormalizeAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

void Scan2MapLocation::UpdateLocationCovariance()
{
    const double icp_score = std::isfinite(last_icp_score_) ? last_icp_score_ : SCORE_THRESHOLD_MAX_;
    const double normalized_icp = std::max(0.0, std::min(1.0, icp_score / std::max(1e-6, SCORE_THRESHOLD_MAX_)));
    const double search_scale = std::max(1e-6, search_score_max_distance_ * search_score_max_distance_);
    double normalized_search = normalized_icp;
    if (last_search_active_ && last_search_applied_)
    {
        const double search_score = std::isfinite(last_search_score_) ? last_search_score_ : search_scale;
        normalized_search = std::max(0.0, std::min(1.0, search_score / search_scale));
    }
    else if (last_search_active_ && !last_search_applied_)
    {
        normalized_search = 1.0;
    }

    double confidence_penalty = std::max(normalized_icp, 0.60 * normalized_search);
    if (last_icp_high_score_)
    {
        confidence_penalty = std::max(confidence_penalty, 0.85);
    }
    if (last_icp_large_angle_)
    {
        confidence_penalty = std::min(1.0, confidence_penalty + 0.10);
    }
    if (last_scan_used_count_ < static_cast<size_t>(Point_Quantity_THRESHOLD_))
    {
        confidence_penalty = std::min(1.0, confidence_penalty + 0.05);
    }

    const double variance_x_max = std::max(Variance_X, covariance_translation_min_);
    const double variance_y_max = std::max(Variance_Y, covariance_translation_min_);
    const double variance_yaw_max = std::max(Variance_Yaw, covariance_yaw_min_);

    const double variance_x = covariance_translation_min_ +
                              (variance_x_max - covariance_translation_min_) * confidence_penalty;
    const double variance_y = covariance_translation_min_ +
                              (variance_y_max - covariance_translation_min_) * confidence_penalty;
    const double variance_yaw = covariance_yaw_min_ +
                                (variance_yaw_max - covariance_yaw_min_) * confidence_penalty;

    location_match.pose.covariance = {variance_x, 0, 0, 0, 0, 0,
                                      0, variance_y, 0, 0, 0, 0,
                                      0, 0, 1e-9, 0, 0, 0,
                                      0, 0, 0, 1e-9, 0, 0,
                                      0, 0, 0, 0, 1e-9, 0,
                                      0, 0, 0, 0, 0, variance_yaw};
}

bool Scan2MapLocation::ComputeMapToOdomTransform(const Eigen::Isometry3d &map_to_base,
                                                 const ros::Time &stamp,
                                                 Eigen::Isometry3d &map_to_odom)
{
    std::deque<nav_msgs::Odometry> odom_queue_snapshot;
    {
        std::lock_guard<std::mutex> lock(odom_lock_);
        odom_queue_snapshot = odom_queue_;
    }

    Eigen::Isometry3d odom_to_base = Eigen::Isometry3d::Identity();
    if (!LookupOdomPose(odom_to_base, stamp.toSec(), odom_queue_snapshot))
    {
        return false;
    }

    const Eigen::Isometry3d raw_map_to_odom = map_to_base * odom_to_base.inverse();
    double x, y, yaw;
    ExtractPose2D(raw_map_to_odom, x, y, yaw);
    map_to_odom = BuildPose2D(x, y, yaw);
    return true;
}

void Scan2MapLocation::PublishMapToOdomTransform(const Eigen::Isometry3d &map_to_base,
                                                 const ros::Time &stamp)
{
    if (!publish_map_to_odom_tf_)
    {
        return;
    }

    Eigen::Isometry3d measured_map_to_odom = Eigen::Isometry3d::Identity();
    if (!ComputeMapToOdomTransform(map_to_base, stamp, measured_map_to_odom))
    {
        return;
    }

    double measured_x, measured_y, measured_yaw;
    ExtractPose2D(measured_map_to_odom, measured_x, measured_y, measured_yaw);

    const double linear_speed = std::hypot(latest_odom_linear_x_, latest_odom_linear_y_);
    const double angular_speed = std::abs(latest_odom_angular_z_);
    const bool moving = (linear_speed > map_to_odom_moving_linear_threshold_) ||
                        (angular_speed > map_to_odom_moving_angular_threshold_);

    std::lock_guard<std::mutex> lock(map_to_odom_lock_);
    if (!has_map_to_odom_estimate_)
    {
        map_to_odom_estimate_ = measured_map_to_odom;
        has_map_to_odom_estimate_ = true;
    }
    else if (smooth_map_to_odom_)
    {
        double prev_x, prev_y, prev_yaw;
        ExtractPose2D(map_to_odom_estimate_, prev_x, prev_y, prev_yaw);

        const double dx = measured_x - prev_x;
        const double dy = measured_y - prev_y;
        const double dyaw = NormalizeAngle(measured_yaw - prev_yaw);
        const double dtrans = std::hypot(dx, dy);

        double alpha = moving ? map_to_odom_moving_alpha_ : map_to_odom_stationary_alpha_;
        if (!moving &&
            dtrans < map_to_odom_stationary_translation_deadband_ &&
            std::abs(dyaw) < map_to_odom_stationary_yaw_deadband_)
        {
            alpha = 0.0;
        }

        map_to_odom_estimate_ = BuildPose2D(prev_x + alpha * dx,
                                            prev_y + alpha * dy,
                                            prev_yaw + alpha * dyaw);
    }
    else
    {
        map_to_odom_estimate_ = measured_map_to_odom;
    }
    last_map_to_odom_stamp_ = stamp;
}

void Scan2MapLocation::mapToOdomPublishTimerCallback(const ros::TimerEvent &event)
{
    if (!publish_map_to_odom_tf_)
    {
        return;
    }

    Eigen::Isometry3d map_to_odom = Eigen::Isometry3d::Identity();
    {
        std::lock_guard<std::mutex> lock(map_to_odom_lock_);
        if (!has_map_to_odom_estimate_)
        {
            return;
        }
        map_to_odom = map_to_odom_estimate_;
    }

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = event.current_real;
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = odom_frame_;
    tf_msg.transform.translation.x = map_to_odom.translation().x();
    tf_msg.transform.translation.y = map_to_odom.translation().y();
    tf_msg.transform.translation.z = 0.0;
    Eigen::Quaterniond q(map_to_odom.rotation());
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_broadcaster_.sendTransform(tf_msg);
}

void Scan2MapLocation::LogFrameSummary(size_t frame_id, const std::string &stage, const std::string &status,
                                       const std::string &reason, const Eigen::Isometry3d *predicted_pose,
                                       const Eigen::Isometry3d *final_pose)
{
    PublishLocalizationDebugState(frame_id, ros::Time(scan_time_), stage, status, reason, predicted_pose, final_pose);

    if (status == "OK" && !if_debug_)
    {
        return;
    }

    const double odom_tail_gap = last_scan_end_time_ - last_odom_back_time_;
    std::ostringstream oss;
    // oss << std::fixed << std::setprecision(3);
    // oss << "[loc-debug] frame=" << frame_id
    //     << " status=" << status
    //     << " stage=" << stage
    //     << " reason=" << reason
    //     << " scan_stamp=" << scan_time_
    //     << " scan_age=" << last_scan_age_
    //     << " scan_range=[" << last_scan_start_time_ << "," << last_scan_end_time_ << "]"
    //     << " scan_duration=" << last_scan_duration_
    //     << std::setprecision(6)
    //     << " scan_dt=" << last_scan_point_time_increment_
    //     << std::setprecision(3)
    //     << " scan_ref_offset=" << last_scan_reference_offset_
    //     << " scan_motion=(" << last_scan_motion_translation_ << "," << last_scan_motion_yaw_ << ")"
    //     << " scan_tail_gap=" << odom_tail_gap
    //     << " scan_odom_cover=" << (last_scan_odom_coverage_ok_ ? 1 : 0)
    //     << " scan_odom_wait=" << last_scan_odom_wait_count_
    //     << " deskew=" << (last_scan_deskew_applied_ ? 1 : 0)
    //     << " deskew_pts=" << last_scan_deskew_point_count_
    //     << " deskew_fail_pts=" << last_scan_deskew_failed_point_count_
    //     << " queue=[" << last_odom_front_time_ << "," << last_odom_back_time_ << "]"
    //     << " odom_used=[" << last_odom_start_used_time_ << "," << last_odom_end_used_time_ << "]"
    //     << " odom_dt=" << last_odom_dt_
    //     << " odom_wait=" << last_odom_wait_count_
    //     << " raw=" << last_scan_raw_count_
    //     << " valid=" << last_scan_valid_count_
    //     << " voxel=" << last_scan_voxel_count_
    //     << " used=" << last_scan_used_count_
    //     << " removed=" << last_removed_count_
    //     << " odom_start=(" << last_odom_start_x_ << "," << last_odom_start_y_ << "," << last_odom_start_yaw_ << ")"
    //     << " odom_end=(" << last_odom_end_x_ << "," << last_odom_end_y_ << "," << last_odom_end_yaw_ << ")"
    //     << " odom_delta=(" << last_odom_delta_x_ << "," << last_odom_delta_y_ << "," << last_odom_delta_yaw_ << ")"
    //     << " search_active=" << (last_search_active_ ? 1 : 0)
    //     << " startup_reloc=" << (last_startup_relocalization_active_ ? 1 : 0)
    //     << " search_apply=" << (last_search_applied_ ? 1 : 0)
    //     << " search_candidates=" << last_search_candidate_count_
    //     << std::setprecision(6)
    //     << " search_pred_score=" << last_search_predicted_score_
    //     << " search_score=" << last_search_score_
    //     << std::setprecision(3)
    //     << " search_offset=(" << last_search_dx_ << "," << last_search_dy_ << "," << last_search_dyaw_ << ")"
    //     << std::setprecision(6)
    //     << " icp_score=" << last_icp_score_
    //     << std::setprecision(3)
    //     << " icp_delta=(" << last_icp_dx_ << "," << last_icp_dy_ << "," << last_icp_dyaw_ << ")"
    //     << std::setprecision(6)
    //     << " reg_sigma=" << last_registration_sigma_
    //     << " reg_max_corr=" << last_registration_max_correspondence_
    //     << std::setprecision(3)
    //     << " reg_corr=" << last_registration_correspondence_count_
    //     << " static_lock=" << (last_static_lock_applied_ ? 1 : 0);

    if (predicted_pose != nullptr)
    {
        double x, y, yaw;
        ExtractPose2D(*predicted_pose, x, y, yaw);
        // oss << " pred=(" << x << "," << y << "," << yaw << ")";
    }

    if (final_pose != nullptr)
    {
        double x, y, yaw;
        ExtractPose2D(*final_pose, x, y, yaw);
        // oss << " final=(" << x << "," << y << "," << yaw << ")";
    }

    if (status == "FAIL" || status == "RISK")
    {
        // ROS_WARN_STREAM(oss.str());
    }
    else
    {
        // ROS_INFO_STREAM(oss.str());
    }
}

/**
 * 从tf树读取parent_frame到child_frame的坐标变换
 * 保存到trans中
 * trans 类型： Eigen::Isometry3d 虽然称为3D，实质上为4*4矩阵
 */
bool Scan2MapLocation::GetTransform(Eigen::Isometry3d &trans, const std::string parent_frame, const std::string child_frame, const ros::Time stamp)
{
    bool gotTransform = false;
    trans = Eigen::Isometry3d::Identity(); // map到base的欧式变换矩阵4x4
    geometry_msgs::TransformStamped transformStamped;

    // Pos_map = T_{map}^{base_link} * Pos_base_link
    try
    {
        gotTransform = true;
        transformStamped = tfBuffer_.lookupTransform(parent_frame, child_frame, stamp, ros::Duration(1.0));
        // std::cout << "input rostime of Transform:" << stamp <<std::endl;
        // std::cout << "output rostime of Transform:" << transformStamped.header.stamp <<std::endl;
    }
    catch (tf2::TransformException &ex)
    {
        gotTransform = false;
        ROS_WARN("DIDNT GET TRANSFORM %s %s IN B", parent_frame.c_str(), child_frame.c_str());
        ros::Duration(1.0).sleep();
        return false;
    }

    tf2::Quaternion quat(
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z,
        transformStamped.transform.rotation.w);

    double roll, pitch, yaw;                       // 定义存储r\p\y的容器
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 进行转换
    // std::cout << "trans roll, pitch, yaw =  \n" << roll << "  " << pitch << "  " << yaw << std::endl;

    Eigen::Matrix3d point_rotation;
    point_rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    // std::cout << "point_rotation = " << point_rotation <<std::endl;

    trans.rotate(point_rotation);
    trans.pretranslate(Eigen::Vector3d(transformStamped.transform.translation.x,
                                       transformStamped.transform.translation.y,
                                       transformStamped.transform.translation.z));
    return gotTransform;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_to_map_location_node"); // 节点的名字
    Scan2MapLocation scan_to_map;
    ros::MultiThreadedSpinner spinner(2); // Use 2 threads
    spinner.spin();                       // spin() will not return until the node has been shutdown
    return 0;
}
