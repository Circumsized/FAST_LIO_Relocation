// FAST_LIO_Relocation ROS2 (Humble) 移植与重构
// 基于 FAST-LIO2 (GPL-3.0, hku-mars) 的 laserMapping.cpp 模块化重写：
//  - 全部全局变量收拢为节点成员
//  - 状态机 / 位姿门控拆分为可单测模块 (src/localization/)
//  - jsk OverlayText -> visualization_msgs/Marker (TEXT_VIEW_FACING)
//  - 固定大数组 -> 按需 vector；移除 Python/matplotlib 依赖
#include <omp.h>
#include <cmath>
#include <csignal>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <deque>
#include <fstream>
#include <utility>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/io.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "common_lib.h"
#include "preprocess.h"
#include "use-ikfom.hpp"
#include "IMU_Processing.hpp"
#include "ikd-Tree/ikd_Tree.h"
#include "localization/state_machine.hpp"
#include "localization/pose_gate.hpp"
#include "ndt_relocalizer.hpp"

#define INIT_TIME       (0.1)
#define LASER_POINT_COV (0.001)
#define PUBFRAME_PERIOD (20)
#define SKEW_SYM_MATRIX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0

/* FAST_LIO_Relocation 修改：运行模式
   MODE_MAPPING      : 原始 FAST-LIO 建图
   MODE_LOCALIZATION : 基于 PCD 先验地图的重定位 */
enum class RunMode
{
    MODE_MAPPING = 0,
    MODE_LOCALIZATION = 1,
};

namespace fast_lio
{

class FastLioNode : public rclcpp::Node
{
public:
    explicit FastLioNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("laserMapping", options),
          tf_broadcaster_(this)
    {
        read_parameters();
        setup_ros_interfaces();
        setup_localization();
        RCLCPP_INFO(get_logger(), "[FAST_LIO_Relocation] Node started in %s mode.",
                    is_localization_mode() ? "LOCALIZATION" : "MAPPING");
    }

    void run()
    {
        rclcpp::WallRate rate(5000);
        MeasureGroup Measures;
        while (rclcpp::ok())
        {
            rclcpp::spin_some(shared_from_this());
            if (sync_packages(Measures))
            {
                process_one_frame(Measures);
            }
            rate.sleep();
        }
        save_pcd_if_needed();
        if (fout_out_.is_open()) fout_out_.close();
    }

private:
    // ============================ 参数 ============================
    template<typename T>
    T param(const std::string &name, const T &def)
    {
        return declare_parameter<T>(name, def);
    }

    void read_parameters()
    {
        path_en_ = param<bool>("publish.path_en", true);
        path_min_distance_ = param<double>("publish.path_min_distance", 0.05);
        path_max_length_ = std::max(1, param<int>("publish.path_max_length", 10000));
        scan_pub_en_ = param<bool>("publish.scan_publish_en", true);
        dense_pub_en_ = param<bool>("publish.dense_publish_en", true);
        scan_body_pub_en_ = param<bool>("publish.scan_bodyframe_pub_en", true);
        NUM_MAX_ITERATIONS_ = param<int>("max_iteration", 4);
        map_file_path_ = param<std::string>("map_file_path", "");
        lid_topic_ = param<std::string>("common.lid_topic", "/livox/lidar");
        imu_topic_ = param<std::string>("common.imu_topic", "/livox/imu");
        time_sync_en_ = param<bool>("common.time_sync_en", false);
        time_diff_lidar_to_imu_ = param<double>("common.time_offset_lidar_to_imu", 0.0);
        filter_size_corner_min_ = std::max(1e-3, param<double>("filter_size_corner", 0.5));
        filter_size_surf_min_ = std::max(1e-3, param<double>("filter_size_surf", 0.5));
        filter_size_map_min_ = std::max(1e-3, param<double>("filter_size_map", 0.5));
        cube_len_ = std::max(1e-3, param<double>("cube_side_length", 200.0));
        gravity_align_en_ = param<bool>("mapping.gravity_align_en", true);
        det_range_ = param<double>("mapping.det_range", 300.0);
        fov_deg_ = param<double>("mapping.fov_degree", 180.0);
        gyr_cov_ = param<double>("mapping.gyr_cov", 0.1);
        acc_cov_ = param<double>("mapping.acc_cov", 0.1);
        b_gyr_cov_ = param<double>("mapping.b_gyr_cov", 0.0001);
        b_acc_cov_ = param<double>("mapping.b_acc_cov", 0.0001);
        extrinsic_est_en_ = param<bool>("mapping.extrinsic_est_en", true);

        p_pre_->blind = std::max(0.0, param<double>("preprocess.blind", p_pre_->blind));
        p_pre_->lidar_type = param<int>("preprocess.lidar_type", p_pre_->lidar_type);
        p_pre_->N_SCANS = std::max(1, std::min(128, param<int>("preprocess.scan_line", p_pre_->N_SCANS)));
        p_pre_->time_unit = param<int>("preprocess.timestamp_unit", p_pre_->time_unit);
        p_pre_->SCAN_RATE = std::max(1, param<int>("preprocess.scan_rate", p_pre_->SCAN_RATE));
        p_pre_->point_filter_num = std::max(1, param<int>("point_filter_num", p_pre_->point_filter_num));
        p_pre_->feature_enabled = param<bool>("feature_extract_enable", p_pre_->feature_enabled);

        runtime_pos_log_ = param<bool>("runtime_pos_log_enable", false);
        pcd_save_en_ = param<bool>("pcd_save.pcd_save_en", false);
        pcd_save_interval_ = param<int>("pcd_save.interval", -1);

        // 注意：ROS2 不允许空数组作为参数默认值（无法推断类型），故给出零默认
        auto extrinsic_T = param<std::vector<double>>("mapping.extrinsic_T", std::vector<double>{0.0, 0.0, 0.0});
        auto extrinsic_R = param<std::vector<double>>("mapping.extrinsic_R",
                                                      std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
        if (extrinsic_T.size() >= 3) std::copy(extrinsic_T.begin(), extrinsic_T.begin() + 3, extrinT_.data());
        if (extrinsic_R.size() >= 9) std::copy(extrinsic_R.begin(), extrinsic_R.begin() + 9, extrinR_.data());

        // ===== FAST_LIO_Relocation：运行模式 =====
        const int run_mode_i = param<int>("run_mode", 0);
        if (run_mode_i != static_cast<int>(RunMode::MODE_MAPPING) &&
            run_mode_i != static_cast<int>(RunMode::MODE_LOCALIZATION))
        {
            RCLCPP_WARN(get_logger(), "[FAST_LIO_Relocation] Invalid run_mode, fallback to MODE_MAPPING.");
            run_mode_ = RunMode::MODE_MAPPING;
        }
        else
        {
            run_mode_ = static_cast<RunMode>(run_mode_i);
        }
        map_voxel_size_ = std::max(1e-3, param<double>("map_voxel_size", 0.5));

        // ===== 定位状态机 / 门控参数 =====
        TrackingStateMachineConfig smcfg;
        smcfg.min_effective_points_for_tracking = param<int>("localization.min_effective_points_for_tracking", 15);
        smcfg.max_residual_for_tracking = param<double>("localization.max_residual_for_tracking", 0.40);
        smcfg.min_effective_points_for_good = param<int>("localization.min_effective_points_for_good", 30);
        smcfg.max_residual_for_good = param<double>("localization.max_residual_for_good", 0.20);
        smcfg.unlock_to_tracking_streak = param<int>("localization.unlock_to_tracking_streak", 3);
        smcfg.good_match_streak_to_lock = param<int>("localization.good_match_streak_to_lock", 5);
        smcfg.bad_match_streak_to_lost = param<int>("localization.bad_match_streak_to_lost", 5);
        smcfg.min_time_before_lock_sec = param<double>("localization.min_time_before_lock_sec", 2.0);
        smcfg.lost_recovery_en = param<bool>("localization.lost_recovery_en", false);
        smcfg.lost_recovery_streak = param<int>("localization.lost_recovery_streak", 10);
        smcfg.lost_recovery_max_attempts = param<int>("localization.lost_recovery_max_attempts", 3);
        smcfg.lost_recovery_cooldown_sec = param<double>("localization.lost_recovery_cooldown_sec", 5.0);
        state_machine_ = std::make_unique<TrackingStateMachine>(smcfg);
        max_residual_for_good_ = smcfg.max_residual_for_good;

        PoseGateConfig pgcfg;
        pgcfg.max_position_jump_for_update_locked =
            param<double>("localization.max_position_jump_for_update_locked", 1.0);
        pgcfg.adaptive_gating_en = param<bool>("localization.adaptive_gating_en", false);
        pgcfg.adaptive_scale_max = param<double>("localization.adaptive_scale_max", 3.0);
        pose_gate_ = std::make_unique<PoseGate>(pgcfg);

        prior_map_pub_interval_ = param<int>("localization.prior_map_pub_interval", 50);

        // ===== NDT 重定位后端参数（默认关闭，保持既有行为）=====
        relocalization_enabled_ = param<bool>("relocalization.enable", false);
        relocalization_trigger_interval_ = std::max(1, param<int>("relocalization.trigger_interval_frames", 5));
        relocalization_submap_frames_ = std::max(1, param<int>("relocalization.local_submap_frames", 5));
        relocalization_local_map_radius_ = std::max(1.0, param<double>("relocalization.local_map_radius", 30.0));
        relocalization_cov_inflation_ = std::max(1.0, param<double>("relocalization.cov_inflation", 10.0));

        NdtRelocalizerConfig ndt_cfg;
        ndt_cfg.ndt_resolution = std::max(1e-3, param<double>("relocalization.ndt_resolution", 1.0));
        ndt_cfg.ndt_step_size = std::max(1e-3, param<double>("relocalization.ndt_step_size", 0.1));
        ndt_cfg.ndt_trans_eps = std::max(1e-6, param<double>("relocalization.ndt_trans_eps", 0.01));
        ndt_cfg.ndt_max_iter = std::max(1, param<int>("relocalization.ndt_max_iter", 30));
        ndt_cfg.ndt_num_threads = std::max(0, param<int>("relocalization.ndt_num_threads", 0));
        ndt_cfg.source_voxel_size = std::max(1e-3, param<double>("relocalization.source_voxel_size", 0.5));
        ndt_cfg.target_voxel_size = std::max(1e-3, param<double>("relocalization.target_voxel_size", 0.5));
        ndt_cfg.max_fitness_score = std::max(1e-6, param<double>("relocalization.max_fitness_score", 1.0));
        ndt_cfg.max_translation_delta = std::max(0.0, param<double>("relocalization.max_translation_delta", 8.0));
        ndt_cfg.max_rotation_delta_deg = std::max(0.0, param<double>("relocalization.max_rotation_delta_deg", 20.0));
        ndt_cfg.min_source_points = std::max(1, param<int>("relocalization.min_source_points", 200));
        ndt_cfg.min_target_points = std::max(1, param<int>("relocalization.min_target_points", 1000));
        ndt_relocalizer_ = std::make_unique<NdtRelocalizer>(ndt_cfg);

        // 启动位姿初值：仅在提供 initial_pose 时启用。
        // 语义约定：initial_pose = [x,y,z,roll,pitch,yaw(度)] 表示 **IMU 相对 map 的位姿 T_WI**，
        // 启动时直接写入滤波器 s0.pos/s0.rot（与 state 的 T_WI 语义一致），不做外参换算。
        auto initial_pose = param<std::vector<double>>("relocalization.initial_pose", std::vector<double>{});
        if (initial_pose.size() >= 6)
        {
            const double roll = initial_pose[3] * PI_M / 180.0;
            const double pitch = initial_pose[4] * PI_M / 180.0;
            const double yaw = initial_pose[5] * PI_M / 180.0;
            const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
            const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
            const Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
            initial_guess_map_body_.setIdentity();
            initial_guess_map_body_.block<3, 3>(0, 0) = (rz * ry * rx).toRotationMatrix();
            initial_guess_map_body_.block<3, 1>(0, 3) = Eigen::Vector3d(initial_pose[0], initial_pose[1], initial_pose[2]);
            initial_pose_provided_ = true;
        }
        else
        {
            initial_pose_provided_ = false;
        }

        // 世界系 frame（集中管理，替代原版 7 处散落字串；可用参数覆盖）
        const std::string default_frame = run_mode_ == RunMode::MODE_LOCALIZATION ? "map" : "camera_init";
        world_frame_ = param<std::string>("world_frame", default_frame);

        FOV_DEG_ = (fov_deg_ + 10.0) > 179.9 ? 179.9 : (fov_deg_ + 10.0);
        HALF_FOV_COS_ = cos(FOV_DEG_ * 0.5 * PI_M / 180.0);
    }

    // ============================ ROS 接口 ============================
    void setup_ros_interfaces()
    {
        p_imu_->set_gyr_cov(V3D(gyr_cov_, gyr_cov_, gyr_cov_));
        p_imu_->set_acc_cov(V3D(acc_cov_, acc_cov_, acc_cov_));
        p_imu_->set_gyr_bias_cov(V3D(b_gyr_cov_, b_gyr_cov_, b_gyr_cov_));
        p_imu_->set_acc_bias_cov(V3D(b_acc_cov_, b_acc_cov_, b_acc_cov_));
        p_imu_->set_extrinsic(extrinT_, extrinR_);
        p_imu_->set_gravity_align_enable(gravity_align_en_);
        p_imu_->lidar_type = p_pre_->lidar_type;

        double epsi[23] = {0.001};
        kf_.init_dyn_share(get_f, df_dx, df_dw,
                           [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &d)
                           { h_share_model(s, d); },
                           NUM_MAX_ITERATIONS_, epsi);

        downSizeFilterSurf_.setLeafSize(filter_size_surf_min_, filter_size_surf_min_, filter_size_surf_min_);

        if (runtime_pos_log_)
        {
            fout_out_.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
        }

        if (p_pre_->lidar_type == AVIA)
        {
            sub_pcl_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
                lid_topic_, 200000,
                [this](const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) { livox_pcl_cbk(msg); });
        }
        else
        {
            sub_pcl_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                lid_topic_, 200000,
                [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { standard_pcl_cbk(msg); });
        }
        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 200000,
            [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) { imu_cbk(msg); });

        pub_laser_cloud_world_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100000);
        pub_laser_cloud_body_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 100000);
        pub_laser_map_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/Laser_map", rclcpp::QoS(1).transient_local());
        pub_odom_aft_mapped_ = create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100000);
        pub_path_ = create_publisher<nav_msgs::msg::Path>("/path", 100000);
        pub_status_marker_ = create_publisher<visualization_msgs::msg::Marker>(
            "/localization_status_marker", 10);

        path_.header.frame_id = world_frame_;
    }

    void setup_localization()
    {
        if (!is_localization_mode()) return;
        if (!init_localization_map_from_prior())
        {
            RCLCPP_ERROR(get_logger(), "[FAST_LIO_Relocation] Failed to initialize prior map.");
            rclcpp::shutdown();
            return;
        }
        publish_prior_map(pub_laser_map_);
        prior_map_published_once_ = true;
        RCLCPP_INFO(get_logger(),
                    "[FAST_LIO_Relocation] Prior map published as latched topic /Laser_map.");
    }

    // ============================ 缓冲与同步 ============================
    void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
    {
        std::lock_guard<std::mutex> lk(mtx_buffer_);
        const double t = stamp_to_sec(msg->header.stamp);
        if (t < last_timestamp_lidar_)
        {
            RCLCPP_ERROR(get_logger(), "lidar loop back, clear buffer");
            lidar_buffer_.clear();
            time_buffer_.clear();
            lidar_pushed_ = false;
            lidar_end_time_ = 0.0;
        }
        last_timestamp_lidar_ = t;

        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        p_pre_->process(msg, ptr);
        lidar_buffer_.push_back(ptr);
        time_buffer_.push_back(t);
    }

    void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg)
    {
        std::lock_guard<std::mutex> lk(mtx_buffer_);
        last_timestamp_lidar_ = stamp_to_sec(msg->header.stamp);
        if (last_timestamp_lidar_ < last_timestamp_lidar_prev_)
        {
            RCLCPP_ERROR(get_logger(), "lidar loop back, clear buffer");
            lidar_buffer_.clear();
            time_buffer_.clear();
            lidar_pushed_ = false;
            lidar_end_time_ = 0.0;
        }

        if (!time_sync_en_ && std::abs(last_timestamp_imu_ - last_timestamp_lidar_) > 10.0 &&
            !imu_buffer_.empty() && !lidar_buffer_.empty())
        {
            RCLCPP_WARN(get_logger(),
                        "IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf",
                        last_timestamp_imu_, last_timestamp_lidar_);
        }
        last_timestamp_lidar_prev_ = last_timestamp_lidar_;

        if (time_sync_en_ && !timediff_set_flg_ &&
            std::abs(last_timestamp_lidar_ - last_timestamp_imu_) > 1.0 && !imu_buffer_.empty())
        {
            timediff_set_flg_ = true;
            timediff_lidar_wrt_imu_ = last_timestamp_lidar_ + 0.1 - last_timestamp_imu_;
            RCLCPP_WARN(get_logger(), "Self sync IMU and LiDAR, time diff is %.10lf",
                        timediff_lidar_wrt_imu_);
        }

        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        p_pre_->process(msg, ptr);
        lidar_buffer_.push_back(ptr);
        time_buffer_.push_back(last_timestamp_lidar_);
    }

    void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in)
    {
        auto msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);
        double t = stamp_to_sec(msg_in->header.stamp) - time_diff_lidar_to_imu_;
        if (std::abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_en_)
        {
            t = timediff_lidar_wrt_imu_ + stamp_to_sec(msg_in->header.stamp);
        }
        msg->header.stamp = sec_to_stamp(t);

        std::lock_guard<std::mutex> lk(mtx_buffer_);
        if (t < last_timestamp_imu_)
        {
            RCLCPP_WARN(get_logger(), "imu loop back, clear buffer");
            imu_buffer_.clear();
        }
        last_timestamp_imu_ = t;
        imu_buffer_.push_back(msg);
    }

    bool sync_packages(MeasureGroup &meas)
    {
        std::lock_guard<std::mutex> lk(mtx_buffer_);
        if (lidar_buffer_.empty() || imu_buffer_.empty()) return false;

        if (!lidar_pushed_)
        {
            meas.lidar = lidar_buffer_.front();
            meas.lidar_beg_time = time_buffer_.front();
            if (meas.lidar->points.size() <= 1)  // time too little
            {
                lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
                RCLCPP_WARN(get_logger(), "Too few input point cloud!");
            }
            else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime_)
            {
                lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
            }
            else
            {
                scan_num_++;
                lidar_end_time_ = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
                lidar_mean_scantime_ += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime_) / scan_num_;
            }
            if (p_pre_->lidar_type == MARSIM)
            {
                lidar_end_time_ = meas.lidar_beg_time;
            }
            meas.lidar_end_time = lidar_end_time_;
            lidar_pushed_ = true;
        }

        if (last_timestamp_imu_ < lidar_end_time_) return false;

        double imu_time = stamp_to_sec(imu_buffer_.front()->header.stamp);
        meas.imu.clear();
        while (!imu_buffer_.empty())
        {
            imu_time = stamp_to_sec(imu_buffer_.front()->header.stamp);
            if (imu_time > lidar_end_time_) break;
            meas.imu.push_back(imu_buffer_.front());
            imu_buffer_.pop_front();
        }

        lidar_buffer_.pop_front();
        time_buffer_.pop_front();
        lidar_pushed_ = false;
        if (meas.imu.empty())
        {
            RCLCPP_WARN(get_logger(), "Drop LiDAR frame without an IMU sample at or before its end time");
            return false;
        }
        return true;
    }

    // ============================ 几何工具 ============================
    void pointBodyToWorld(const PointType &pi, PointType &po)
    {
        const V3D p_body(pi.x, pi.y, pi.z);
        const V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) + state_point_.pos);
        po.x = p_global(0);
        po.y = p_global(1);
        po.z = p_global(2);
        po.intensity = pi.intensity;
    }

    void refresh_pose_cache_from_state()
    {
        euler_cur_ = SO3ToEuler(state_point_.rot);
        pos_lid_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;
        geoQuat_.x = state_point_.rot.coeffs()[0];
        geoQuat_.y = state_point_.rot.coeffs()[1];
        geoQuat_.z = state_point_.rot.coeffs()[2];
        geoQuat_.w = state_point_.rot.coeffs()[3];
    }

    void set_posestamp(geometry_msgs::msg::Pose &pose) const
    {
        pose.position.x = state_point_.pos(0);
        pose.position.y = state_point_.pos(1);
        pose.position.z = state_point_.pos(2);
        pose.orientation.x = geoQuat_.x;
        pose.orientation.y = geoQuat_.y;
        pose.orientation.z = geoQuat_.z;
        pose.orientation.w = geoQuat_.w;
    }

    // ============================ 先验地图 ============================
    bool load_prior_map_from_pcd(const std::string &pcd_path,
                                 const PointCloudXYZI::Ptr &cloud_raw,
                                 const PointCloudXYZI::Ptr &cloud_ds)
    {
        if (pcd_path.empty())
        {
            RCLCPP_ERROR(get_logger(), "[FAST_LIO_Relocation] map_file_path is empty.");
            return false;
        }
        if (pcl::io::loadPCDFile<PointType>(pcd_path, *cloud_raw) < 0)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "[FAST_LIO_Relocation] Failed to load map: " << pcd_path);
            return false;
        }
        if (cloud_raw->empty())
        {
            RCLCPP_ERROR(get_logger(), "[FAST_LIO_Relocation] Loaded map is empty.");
            return false;
        }
        pcl::VoxelGrid<PointType> prior_filter;
        prior_filter.setLeafSize(map_voxel_size_, map_voxel_size_, map_voxel_size_);
        prior_filter.setInputCloud(cloud_raw);
        prior_filter.filter(*cloud_ds);
        if (cloud_ds->empty())
        {
            RCLCPP_ERROR(get_logger(), "[FAST_LIO_Relocation] Downsampled map is empty.");
            return false;
        }
        RCLCPP_INFO_STREAM(get_logger(),
                           "[FAST_LIO_Relocation] Prior map loaded. Raw size = " << cloud_raw->size()
                           << ", downsampled size = " << cloud_ds->size());
        return true;
    }

    bool init_localization_map_from_prior()
    {
        if (prior_map_tree_built_)
        {
            RCLCPP_WARN(get_logger(), "[FAST_LIO_Relocation] Prior map already initialized.");
            return true;
        }
        if (!load_prior_map_from_pcd(map_file_path_, prior_map_raw_, prior_map_ds_))
        {
            return false;
        }
        ikdtree_.set_downsample_param(map_voxel_size_);
        ikdtree_.Build(prior_map_ds_->points);
        prior_map_loaded_ = true;
        prior_map_tree_built_ = true;
        RCLCPP_INFO(get_logger(), "[FAST_LIO_Relocation] ikdtree initialized from prior map.");
        return true;
    }

    // ============================ NDT 重定位辅助 ============================
    // 从先验地图按中心裁剪局部目标点云（用于降低 NDT 搜索空间）
    PointCloudXYZI::Ptr crop_prior_map_local(const Eigen::Vector3d &center, double radius) const
    {
        PointCloudXYZI::Ptr out(new PointCloudXYZI());
        if (prior_map_ds_ == nullptr || prior_map_ds_->empty())
        {
            return out;
        }
        const double r2 = radius * radius;
        for (const auto &p : prior_map_ds_->points)
        {
            const double dx = p.x - center.x();
            const double dy = p.y - center.y();
            const double dz = p.z - center.z();
            if (dx * dx + dy * dy + dz * dz <= r2)
            {
                out->push_back(p);
            }
        }
        return out;
    }

    // 拼接近期扫描作为 NDT 源子图，并做帧间运动补偿：
    // 以"最新帧 body（LiDAR）系"为统一基准，把各历史帧点云变换到该基准后拼接，消除重影。
    // 变换推导（p_Li 为第 i 帧 LiDAR 系点）：
    //   p_W = R_WI_i*(R_LI_i*p_Li + t_LI_i) + p_WI_i            （第 i 帧 LiDAR→world）
    //   p_Llast = R_LI_last^T * ( R_WI_last^T*(p_W - p_WI_last) - t_LI_last )  （world→最新帧 LiDAR）
    // 最新帧自身变换为单位矩阵（单帧时输出与原状逐点等价）。
    PointCloudXYZI::Ptr make_source_submap() const
    {
        PointCloudXYZI::Ptr out(new PointCloudXYZI());
        if (recent_scans_.empty())
        {
            return out;
        }

        const RecentScan &last = recent_scans_.back();
        const Eigen::Matrix3d R_WI_last_inv = last.R_WI.transpose();
        const Eigen::Matrix3d R_LI_last_inv = last.R_LI.transpose();

        for (const auto &rec : recent_scans_)
        {
            if (!rec.cloud || rec.cloud->empty())
            {
                continue;
            }
            for (const auto &p : rec.cloud->points)
            {
                const Eigen::Vector3d p_Li(p.x, p.y, p.z);
                const Eigen::Vector3d p_W = rec.R_WI * (rec.R_LI * p_Li + rec.t_LI) + rec.p_WI;
                const Eigen::Vector3d p_Llast =
                    R_LI_last_inv * (R_WI_last_inv * (p_W - last.p_WI) - last.t_LI);
                PointType q = p;
                q.x = static_cast<float>(p_Llast.x());
                q.y = static_cast<float>(p_Llast.y());
                q.z = static_cast<float>(p_Llast.z());
                // 坐标已刚性变换，清空不再对应的 normal/curvature，避免下游误用（本子图仅供 NDT 取 xyz）
                q.normal_x = 0.0f;
                q.normal_y = 0.0f;
                q.normal_z = 0.0f;
                q.curvature = 0.0f;
                out->push_back(q);
            }
        }
        return out;
    }

    // 恢复后膨胀协方差：只放大与"外部重定位直接改变的量"对应块——pos(0..2) 与 rot(3..5)，
    // 其余块（外参/速度/零偏/重力）保持不变，避免无谓放大非位姿不确定度。
    // state_ikfom 块序：pos(0) rot(3) offset_R_L_I(6) offset_T_L_I(9) vel(12) bg(15) ba(18) grav(21)。
    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov
    inflate_covariance_for_relocalization(
        const esekfom::esekf<state_ikfom, 12, input_ikfom>::cov &P) const
    {
        esekfom::esekf<state_ikfom, 12, input_ikfom>::cov out = P;
        const double s = relocalization_cov_inflation_;
        out.template block<3, 3>(0, 0) = P.template block<3, 3>(0, 0) * s;   // pos
        out.template block<3, 3>(3, 3) = P.template block<3, 3>(3, 3) * s;   // rot
        return out;
    }

    // 执行一次 NDT 重定位尝试；成功则注入 EKF、刷新回滚基准、发布，并返回 true。
    // 失败（NDT 未通过 / 状态机拒绝）返回 false，并消耗一次恢复预算防止无限重试。
    bool try_relocalize_and_recover()
    {
        if (!ndt_relocalizer_ || !prior_map_tree_built_)
        {
            return false;
        }

        // 初值基准与源子图基准必须一致：make_source_submap() 把历史帧统一到"最新帧 body 系"，
        // 故此处也以 recent_scans_ 最新帧的 T_WI/外参构造初值（而非 last_locked_state_）。
        // 源点云是 LiDAR 系，NDT 初值需为 T_WL（map←lidar）。
        // 由 p_W = R_WI*(R_LI*p_L + t_LI) + p_WI 得：R_WL = R_WI*R_LI，p_WL = R_WI*t_LI + p_WI。
        if (recent_scans_.empty() || !recent_scans_.back().cloud ||
            recent_scans_.back().cloud->empty())
        {
            return false;   // 无源子图基准帧，无法构造初值
        }
        const RecentScan &base = recent_scans_.back();
        const Eigen::Matrix3d R_WI0 = base.R_WI;
        const Eigen::Vector3d p_WI0 = base.p_WI;
        const Eigen::Matrix3d R_LI0 = base.R_LI;
        const Eigen::Vector3d t_LI0 = base.t_LI;
        Eigen::Matrix4d init_guess = Eigen::Matrix4d::Identity();
        init_guess.block<3, 3>(0, 0) = R_WI0 * R_LI0;
        init_guess.block<3, 1>(0, 3) = R_WI0 * t_LI0 + p_WI0;

        // 目标：以最新帧（源子图基准帧）的 IMU 位置为中心裁剪局部先验地图
        // （裁剪半径以米计，IMU 与 LiDAR 原点差异为分米级，对球心选择影响可忽略）
        PointCloudXYZI::Ptr target = crop_prior_map_local(p_WI0, relocalization_local_map_radius_);
        // 源：近期扫描拼接
        PointCloudXYZI::Ptr source = make_source_submap();
        if (source->empty() || target->empty())
        {
            state_machine_->note_external_recovery_attempt(now().seconds());
            return false;
        }

        const NdtResult r = ndt_relocalizer_->align(source, target, init_guess);
        if (!r.success)
        {
            state_machine_->note_external_recovery_attempt(now().seconds());
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2.0,
                                 "[FAST_LIO_Relocation] NDT relocalization attempt failed.");
            return false;
        }

        const ExternalRecoveryStatus st =
            state_machine_->on_external_relocalization_success(now().seconds());
        if (st != ExternalRecoveryStatus::kSuccess)
        {
            // NDT 收敛/通过但状态机拒绝（如预算耗尽/冷却中）：同样计入一次尝试，避免下一步无限重试
            state_machine_->note_external_recovery_attempt(now().seconds());
            return false;
        }

        // 以当前状态为基底，仅覆盖 pos/rot（保留 bias/extrinsic/gravity/vel）
        state_ikfom ndt_state = kf_.get_x();
        // 坐标系补偿：NDT 源点云为 LiDAR 系、目标为 map 系，故 r.transform 是 T_WL（map←lidar）；
        // 而 state.rot/pos 语义为 T_WI（map←IMU）。按 p_W = R_WI*(R_LI*p_L + t_LI) + p_WI
        // = (R_WI*R_LI)*p_L + (R_WI*t_LI + p_WI) 得 T_WL；其逆（T_WL→T_WI）为：
        //   R_WI = R_WL * R_LI^T        （R_LI 为 L→I 旋转，其逆为转置）
        //   p_WI = p_WL - R_WI * t_LI
        // 外参必须与构造 init_guess（源子图基准帧）所用的外参一致：T_WL↔T_WI 换算要求同一外参。
        // 故此处用 base.R_LI/base.t_LI（源子图基准帧），而非当前 ndt_state 的外参——消除隐性耦合。
        const Eigen::Matrix3d R_WL = r.transform.block<3, 3>(0, 0);
        const Eigen::Vector3d p_WL = r.transform.block<3, 1>(0, 3);
        const Eigen::Matrix3d R_LI = base.R_LI;
        const Eigen::Vector3d t_LI = base.t_LI;
        const Eigen::Matrix3d R_WI = R_WL * R_LI.transpose();
        const Eigen::Vector3d p_WI = p_WL - R_WI * t_LI;
        ndt_state.pos = p_WI;
        ndt_state.rot = SO3(R_WI);
        // 清零速度：LOST 回滚后 vel 为旧帧值，若保留会在下一帧传播中把刚注入的位姿推走，
        // 抵消重定位效果（速度在 LOST 期间本就不可信）。
        ndt_state.vel = Eigen::Vector3d::Zero();
        kf_.change_x(ndt_state);
        auto inflated_P = inflate_covariance_for_relocalization(kf_.get_P());
        kf_.change_P(inflated_P);
        state_point_ = ndt_state;

        // 刷新回滚基准，使 NDT 位姿成为新的 last_locked
        last_locked_state_ = ndt_state;
        last_locked_covariance_ = inflated_P;
        state_machine_->notify_good_snapshot();

        refresh_pose_cache_from_state();
        publish_odometry();
        if (path_en_) publish_path();
        publish_localization_status_overlay();
        RCLCPP_INFO(get_logger(),
                    "[FAST_LIO_Relocation] NDT relocalization succeeded (fitness=%.4f, d_trans=%.3f).",
                    r.fitness, r.translation_delta);
        return true;
    }

    // 维护近期扫描滚动缓冲（body 系），供 NDT 源子图使用。
    // 注意：必须深拷贝——调用方传入的 feats_down_body_ 是持久成员且内容被原地刷新，
    // 若直接存指针，缓冲内所有元素会别名到同一对象，make_source_submap() 将重复拼接同一帧。
    // 同时记录该帧的 T_WI（map←IMU）与外参，供拼接时做帧间运动补偿。
    void push_recent_scan(const PointCloudXYZI::Ptr &scan_body, const state_ikfom &state)
    {
        if (!scan_body || scan_body->empty())
        {
            return;
        }
        RecentScan rec;
        rec.cloud.reset(new PointCloudXYZI(*scan_body));   // 深拷贝快照
        rec.R_WI = state.rot.toRotationMatrix();
        rec.p_WI = Eigen::Vector3d(state.pos.x(), state.pos.y(), state.pos.z());
        rec.R_LI = state.offset_R_L_I.toRotationMatrix();
        rec.t_LI = Eigen::Vector3d(state.offset_T_L_I.x(), state.offset_T_L_I.y(),
                                   state.offset_T_L_I.z());
        recent_scans_.push_back(rec);
        while (static_cast<int>(recent_scans_.size()) > relocalization_submap_frames_)
        {
            recent_scans_.pop_front();
        }
    }

    bool init_map_from_first_scan()
    {
        if (feats_down_size_ <= 5)
        {
            RCLCPP_WARN(get_logger(), "Too few points to initialize map.");
            return false;
        }
        ikdtree_.set_downsample_param(filter_size_map_min_);
        feats_down_world_->resize(feats_down_size_);
        for (int i = 0; i < feats_down_size_; i++)
        {
            pointBodyToWorld(feats_down_body_->points[i], feats_down_world_->points[i]);
        }
        ikdtree_.Build(feats_down_world_->points);
        RCLCPP_INFO_STREAM(get_logger(),
                           "[FAST_LIO_Relocation] Map initialized from first scan. Size = "
                           << feats_down_world_->size());
        return true;
    }

    void map_incremental()
    {
        PointVector PointToAdd;
        PointVector PointNoNeedDownsample;
        PointToAdd.reserve(feats_down_size_);
        PointNoNeedDownsample.reserve(feats_down_size_);
        for (int i = 0; i < feats_down_size_; i++)
        {
            pointBodyToWorld(feats_down_body_->points[i], feats_down_world_->points[i]);
            if (!Nearest_Points_[i].empty() && flg_EKF_inited_)
            {
                const PointVector &points_near = Nearest_Points_[i];
                bool need_add = true;
                PointType mid_point;
                mid_point.x = std::floor(feats_down_world_->points[i].x / filter_size_map_min_) * filter_size_map_min_ + 0.5 * filter_size_map_min_;
                mid_point.y = std::floor(feats_down_world_->points[i].y / filter_size_map_min_) * filter_size_map_min_ + 0.5 * filter_size_map_min_;
                mid_point.z = std::floor(feats_down_world_->points[i].z / filter_size_map_min_) * filter_size_map_min_ + 0.5 * filter_size_map_min_;
                if (std::fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min_ &&
                    std::fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min_ &&
                    std::fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min_)
                {
                    PointNoNeedDownsample.push_back(feats_down_world_->points[i]);
                    continue;
                }
                for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
                {
                    if (points_near.size() < NUM_MATCH_POINTS) break;
                    if (calc_dist(points_near[readd_i], mid_point) < calc_dist(feats_down_world_->points[i], mid_point))
                    {
                        need_add = false;
                        break;
                    }
                }
                if (need_add) PointToAdd.push_back(feats_down_world_->points[i]);
            }
            else
            {
                PointToAdd.push_back(feats_down_world_->points[i]);
            }
        }
        ikdtree_.Add_Points(PointToAdd, true);
        ikdtree_.Add_Points(PointNoNeedDownsample, false);
    }

    // 建图模式 FOV 滑窗裁剪
    void lasermap_fov_segment()
    {
        cub_needrm_.clear();
        const V3D pos_LiD = pos_lid_;
        if (!Localmap_Initialized_)
        {
            for (int i = 0; i < 3; i++)
            {
                LocalMap_Points_.vertex_min[i] = pos_LiD(i) - cube_len_ / 2.0;
                LocalMap_Points_.vertex_max[i] = pos_LiD(i) + cube_len_ / 2.0;
            }
            Localmap_Initialized_ = true;
            return;
        }
        float dist_to_map_edge[3][2];
        bool need_move = false;
        for (int i = 0; i < 3; i++)
        {
            dist_to_map_edge[i][0] = std::fabs(pos_LiD(i) - LocalMap_Points_.vertex_min[i]);
            dist_to_map_edge[i][1] = std::fabs(pos_LiD(i) - LocalMap_Points_.vertex_max[i]);
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * det_range_ ||
                dist_to_map_edge[i][1] <= MOV_THRESHOLD * det_range_)
                need_move = true;
        }
        if (!need_move) return;
        BoxPointType New_LocalMap_Points, tmp_boxpoints;
        New_LocalMap_Points = LocalMap_Points_;
        const float mov_dist = std::max((cube_len_ - 2.0 * MOV_THRESHOLD * det_range_) * 0.5 * 0.9,
                                        static_cast<float>(det_range_ * (MOV_THRESHOLD - 1)));
        for (int i = 0; i < 3; i++)
        {
            tmp_boxpoints = LocalMap_Points_;
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * det_range_)
            {
                New_LocalMap_Points.vertex_max[i] -= mov_dist;
                New_LocalMap_Points.vertex_min[i] -= mov_dist;
                tmp_boxpoints.vertex_min[i] = LocalMap_Points_.vertex_max[i] - mov_dist;
                cub_needrm_.push_back(tmp_boxpoints);
            }
            else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * det_range_)
            {
                New_LocalMap_Points.vertex_max[i] += mov_dist;
                New_LocalMap_Points.vertex_min[i] += mov_dist;
                tmp_boxpoints.vertex_max[i] = LocalMap_Points_.vertex_min[i] + mov_dist;
                cub_needrm_.push_back(tmp_boxpoints);
            }
        }
        LocalMap_Points_ = New_LocalMap_Points;
        if (!cub_needrm_.empty()) ikdtree_.Delete_Point_Boxes(cub_needrm_);
    }

    // ============================ 发布 ============================
    void publish_frame_world()
    {
        if (!scan_pub_en_ && !pcd_save_en_) return;
        const PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en_ ? feats_undistort_ : feats_down_body_);
        const int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
        for (int i = 0; i < size; i++)
        {
            pointBodyToWorld(laserCloudFullRes->points[i], laserCloudWorld->points[i]);
        }

        if (scan_pub_en_)
        {
            sensor_msgs::msg::PointCloud2 laserCloudmsg;
            pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
            laserCloudmsg.header.stamp = sec_to_stamp(lidar_end_time_);
            laserCloudmsg.header.frame_id = world_frame_;
            pub_laser_cloud_world_->publish(laserCloudmsg);
        }

        if (pcd_save_en_)
        {
            *pcd_wait_save_ += *laserCloudWorld;
            ++pcd_frame_count_;
            if (pcd_save_interval_ > 0 && pcd_frame_count_ >= pcd_save_interval_)
            {
                save_pcd_if_needed();
            }
        }
    }

    void publish_frame_body()
    {
        if (!scan_body_pub_en_) return;
        const int size = feats_undistort_->points.size();
        PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));
        for (int i = 0; i < size; i++)
        {
            const auto &pi = feats_undistort_->points[i];
            const V3D p_body_lidar(pi.x, pi.y, pi.z);
            const V3D p_body_imu(state_point_.offset_R_L_I * p_body_lidar + state_point_.offset_T_L_I);
            laserCloudIMUBody->points[i].x = p_body_imu(0);
            laserCloudIMUBody->points[i].y = p_body_imu(1);
            laserCloudIMUBody->points[i].z = p_body_imu(2);
            laserCloudIMUBody->points[i].intensity = pi.intensity;
        }
        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
        laserCloudmsg.header.stamp = sec_to_stamp(lidar_end_time_);
        laserCloudmsg.header.frame_id = "body";
        pub_laser_cloud_body_->publish(laserCloudmsg);
    }

    void publish_prior_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub)
    {
        if (!prior_map_loaded_ || prior_map_ds_->empty()) return;
        sensor_msgs::msg::PointCloud2 laserCloudMap;
        pcl::toROSMsg(*prior_map_ds_, laserCloudMap);
        laserCloudMap.header.stamp = (lidar_end_time_ > 0.0) ? sec_to_stamp(lidar_end_time_)
                                                             : builtin_interfaces::msg::Time();
        laserCloudMap.header.frame_id = world_frame_;
        pub->publish(laserCloudMap);
    }

    /* 用 RViz Marker (TEXT_VIEW_FACING) 在世界系上方显示定位状态，
       替代 ROS1 jsk_rviz_plugins::OverlayText（ROS2 无对应包） */
    void publish_localization_status_overlay()
    {
        if (!is_localization_mode()) return;
        visualization_msgs::msg::Marker text;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.id = 0;
        text.ns = "fastlio_status";
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.header.frame_id = world_frame_;
        text.header.stamp = now();
        text.pose.position.z = 3.0;
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.3;
        switch (state_machine_->state())
        {
        case LocalizationTrackingState::TRACKING_LOCKED:   text.color.g = 1.0; break;
        case LocalizationTrackingState::TRACKING_TRACKING: text.color.b = 1.0; break;
        case LocalizationTrackingState::TRACKING_UNLOCKED: text.color.r = 1.0; text.color.g = 1.0; break;
        default:                                            text.color.r = 1.0; break;
        }
        text.color.a = 1.0;
        std::ostringstream oss;
        oss << "FAST_LIO_Relocation\n"
            << "Mode: LOCALIZATION\n"
            << "State: " << tracking_state_name(state_machine_->state()) << "\n"
            << "Effective points: " << effct_feat_num_ << "\n"
            << std::fixed << std::setprecision(3)
            << "Mean residual: " << res_mean_last_ << "\n"
            << "Good streak: " << state_machine_->good_streak() << "\n"
            << "Bad streak: " << state_machine_->bad_streak() << "\n"
            << "Acceptable streak: " << state_machine_->acceptable_streak() << "\n"
            << "Update accepted: " << (accept_lidar_update_ ? "YES" : "NO") << "\n"
            << "Has last tracking: " << (state_machine_->has_last_tracking() ? "YES" : "NO") << "\n"
            << "Has last locked: " << (state_machine_->has_last_locked() ? "YES" : "NO");
        text.text = oss.str();
        pub_status_marker_->publish(text);
    }

    void publish_odometry()
    {
        nav_msgs::msg::Odometry odomAftMapped;
        odomAftMapped.header.frame_id = world_frame_;
        odomAftMapped.child_frame_id = "body";
        odomAftMapped.header.stamp = sec_to_stamp(lidar_end_time_);
        set_posestamp(odomAftMapped.pose.pose);
        const auto &P = kf_.get_P();
        for (int i = 0; i < 6; i++)
        {
            const int k = i < 3 ? i + 3 : i - 3;
            odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
            odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
            odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
            odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
            odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
            odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
        }
        pub_odom_aft_mapped_->publish(odomAftMapped);

        geometry_msgs::msg::TransformStamped trans;
        trans.header.stamp = odomAftMapped.header.stamp;
        trans.header.frame_id = world_frame_;
        trans.child_frame_id = "body";
        trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
        trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
        trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
        trans.transform.rotation.x = geoQuat_.x;
        trans.transform.rotation.y = geoQuat_.y;
        trans.transform.rotation.z = geoQuat_.z;
        trans.transform.rotation.w = geoQuat_.w;
        tf_broadcaster_.sendTransform(trans);
    }

    void publish_path()
    {
        set_posestamp(msg_body_pos_.pose);
        msg_body_pos_.header.stamp = sec_to_stamp(lidar_end_time_);
        msg_body_pos_.header.frame_id = world_frame_;
        path_.header.frame_id = world_frame_;
        const auto &p = msg_body_pos_.pose.position;
        bool append = path_.poses.empty();
        if (!append)
        {
            const auto &last = path_.poses.back().pose.position;
            const double dx = p.x - last.x;
            const double dy = p.y - last.y;
            const double dz = p.z - last.z;
            append = std::sqrt(dx * dx + dy * dy + dz * dz) >= path_min_distance_;
        }
        if (append)
        {
            path_.poses.push_back(msg_body_pos_);
            if (path_.poses.size() > path_max_length_)
            {
                path_.poses.erase(path_.poses.begin(),
                                  path_.poses.begin() + (path_.poses.size() - path_max_length_));
            }
        }
        pub_path_->publish(path_);
    }

    // ============================ IESKF ============================
    // esekfom 回调：最近邻搜索 + 平面拟合 + 残差/雅可比组装
    void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
    {
        if (laserCloudOri_->points.size() != static_cast<size_t>(feats_down_size_))
        {
            laserCloudOri_->points.resize(feats_down_size_, PointType());
            laserCloudOri_->width = feats_down_size_;
            laserCloudOri_->height = 1;
            laserCloudOri_->is_dense = true;
        }
        if (corr_normvect_->points.size() != static_cast<size_t>(feats_down_size_))
        {
            corr_normvect_->points.resize(feats_down_size_, PointType());
            corr_normvect_->width = feats_down_size_;
            corr_normvect_->height = 1;
            corr_normvect_->is_dense = true;
        }
        std::fill(laserCloudOri_->points.begin(), laserCloudOri_->points.end(), PointType());
        std::fill(corr_normvect_->points.begin(), corr_normvect_->points.end(), PointType());
        total_residual_ = 0.0;

        /** closest surface search and residual computation **/
#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
        for (int i = 0; i < feats_down_size_; i++)
        {
            PointType &point_body = feats_down_body_->points[i];
            PointType &point_world = feats_down_world_->points[i];

            const V3D p_body(point_body.x, point_body.y, point_body.z);
            const V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
            point_world.x = p_global(0);
            point_world.y = p_global(1);
            point_world.z = p_global(2);
            point_world.intensity = point_body.intensity;

            vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
            auto &points_near = Nearest_Points_[i];

            if (ekfom_data.converge)
            {
                ikdtree_.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
                point_selected_surf_[i] = points_near.size() < NUM_MATCH_POINTS ? false :
                                          pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
            }
            if (!point_selected_surf_[i]) continue;

            VF(4) pabcd;
            point_selected_surf_[i] = false;
            if (esti_plane(pabcd, points_near, 0.1f))
            {
                const float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y +
                                  pabcd(2) * point_world.z + pabcd(3);
                const float s_score = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

                if (s_score > 0.9)
                {
                    point_selected_surf_[i] = true;
                    normvec_->points[i].x = pabcd(0);
                    normvec_->points[i].y = pabcd(1);
                    normvec_->points[i].z = pabcd(2);
                    normvec_->points[i].intensity = pd2;
                    res_last_[i] = fabs(pd2);
                }
            }
        }

        effct_feat_num_ = 0;
        for (int i = 0; i < feats_down_size_; i++)
        {
            if (point_selected_surf_[i])
            {
                laserCloudOri_->points[effct_feat_num_] = feats_down_body_->points[i];
                corr_normvect_->points[effct_feat_num_] = normvec_->points[i];
                total_residual_ += res_last_[i];
                effct_feat_num_++;
            }
        }
        if (effct_feat_num_ < 1)
        {
            ekfom_data.valid = false;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0,
                                 "[FAST_LIO_Relocation] No Effective Points!");
            return;
        }
        res_mean_last_ = total_residual_ / effct_feat_num_;

        ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num_, 12);
        ekfom_data.h.resize(effct_feat_num_);
        for (int i = 0; i < effct_feat_num_; i++)
        {
            const PointType &laser_p = laserCloudOri_->points[i];
            const V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
            M3D point_be_crossmat;
            point_be_crossmat << SKEW_SYM_MATRIX(point_this_be);
            const V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
            M3D point_crossmat;
            point_crossmat << SKEW_SYM_MATRIX(point_this);

            const PointType &norm_p = corr_normvect_->points[i];
            const V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

            const V3D C(s.rot.conjugate() * norm_vec);
            const V3D A(point_crossmat * C);
            if (extrinsic_est_en_)
            {
                const V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
                ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
                    VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
            }
            else
            {
                ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
                    VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            }
            ekfom_data.h(i) = -norm_p.intensity;
        }
    }

    // ============================ 主循环单帧处理 ============================
    void process_one_frame(MeasureGroup &Measures)
    {
        if (flg_first_scan_)
        {
            first_lidar_time_ = Measures.lidar_beg_time;
            p_imu_->first_lidar_time = first_lidar_time_;
            flg_first_scan_ = false;
        }

        state_ikfom frame_entry_state = kf_.get_x();
        auto frame_entry_covariance = kf_.get_P();
        const bool was_lost =
            is_localization_mode() &&
            state_machine_->state() == LocalizationTrackingState::TRACKING_LOST;

        if (was_lost && !state_machine_->is_recovery_enabled())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0, "[FAST_LIO_Relocation] Tracking LOST.");
            if (state_machine_->has_last_locked())
            {
                kf_.change_x(last_locked_state_);
                kf_.change_P(last_locked_covariance_);
                state_point_ = last_locked_state_;
            }
            else
            {
                state_point_ = frame_entry_state;
            }
            refresh_pose_cache_from_state();
            publish_odometry();
            if (path_en_) publish_path();
            publish_localization_status_overlay();
            return;
        }

        const auto restore_frame_entry = [this, &frame_entry_state, &frame_entry_covariance, was_lost]()
        {
            if (!was_lost) return;
            kf_.change_x(frame_entry_state);
            kf_.change_P(frame_entry_covariance);
            state_point_ = frame_entry_state;
        };

        // ===== NDT 重定位尝试（LOST 自动恢复）=====
        // 仅当处于 LOST 且恢复开关开启时；成功则注入位姿并本帧 return，不调用 update()，
        // 避免 update() 首行清 just_recovered_flag_ 及随后 LOST 回滚覆盖注入位姿。
        if (was_lost && state_machine_->is_recovery_enabled() && relocalization_enabled_)
        {
            ++relocalization_frame_counter_;
            if (relocalization_frame_counter_ >= relocalization_trigger_interval_)
            {
                relocalization_frame_counter_ = 0;
                if (try_relocalize_and_recover())
                {
                    // 恢复成功：已在内部发布并冻结本帧（随后 return）。
                    // 注意：本帧不执行 p_imu_->Process，故本帧时间戳不推进、点云不入滚动缓冲，
                    // 属"冻结本帧"的既定取舍。
                    return;
                }
            }
            // 未到触发间隔或尝试失败：落入既有 LOST 冻结逻辑（下方）
        }
        else
        {
            // 非 LOST（未丢失/恢复关闭/重定位关闭）：清零节流计数器，
            // 避免下次刚进入 LOST 时因残留值而首帧即触发。
            relocalization_frame_counter_ = 0;
        }

        p_imu_->Process(Measures, kf_, feats_undistort_);
        state_point_ = kf_.get_x();
        pos_lid_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;

        if (feats_undistort_ == nullptr || feats_undistort_->empty())
        {
            restore_frame_entry();
            RCLCPP_WARN(get_logger(), "No point, skip this scan");
            return;
        }

        flg_EKF_inited_ = (Measures.lidar_beg_time - first_lidar_time_) < INIT_TIME ? false : true;

        // ===== 启动位姿初值（仅当提供 initial_pose 时）=====
        // 如实说明：此处仅把外部初值作为滤波器位姿起点（粗对齐来自 assume-given 的 initial_pose），
        // **不在启动阶段运行 NDT**——NDT 仅用于运行期 LOST 恢复。无 initial_pose 则不启用。
        if (is_localization_mode() && relocalization_enabled_ && initial_pose_provided_ &&
            !initial_relocalization_done_ && flg_EKF_inited_)
        {
            state_ikfom s0 = kf_.get_x();
            s0.pos = initial_guess_map_body_.block<3, 1>(0, 3);
            s0.rot = SO3(initial_guess_map_body_.block<3, 3>(0, 0));
            kf_.change_x(s0);
            auto inflated_P = inflate_covariance_for_relocalization(kf_.get_P());
            kf_.change_P(inflated_P);
            state_point_ = s0;
            initial_relocalization_done_ = true;
            RCLCPP_INFO(get_logger(),
                        "[FAST_LIO_Relocation] Applied initial_pose as startup pose prior (no NDT at startup).");
        }

        // ===== 建图模式：FOV 滑窗（定位模式使用固定先验地图，跳过）=====
        if (is_mapping_mode())
        {
            lasermap_fov_segment();
        }

        // ===== 降采样 =====
        downSizeFilterSurf_.setInputCloud(feats_undistort_);
        downSizeFilterSurf_.filter(*feats_down_body_);
        feats_down_size_ = feats_down_body_->points.size();

        // ===== ikdtree 空则初始化（仅建图模式；定位模式启动时已建）=====
        if (ikdtree_.Root_Node == nullptr)
        {
            if (is_localization_mode())
            {
                RCLCPP_ERROR(get_logger(),
                             "[FAST_LIO_Relocation] ikdtree should already be initialized from prior map.");
            }
            else
            {
                init_map_from_first_scan();
            }
            restore_frame_entry();
            return;
        }

        if (feats_down_size_ < 5)
        {
            restore_frame_entry();
            RCLCPP_WARN(get_logger(), "No point, skip this scan");
            return;
        }

        normvec_->resize(feats_down_size_);
        feats_down_world_->resize(feats_down_size_);
        Nearest_Points_.resize(feats_down_size_);
        point_selected_surf_.assign(feats_down_size_, true);
        res_last_.assign(feats_down_size_, -1000.0f);

        // ===== IESKF 迭代更新（保存更新前状态用于门控回滚）=====
        state_ikfom state_before_update = kf_.get_x();
        auto P_before_update = kf_.get_P();
        double solve_H_time = 0;
        kf_.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
        const state_ikfom state_after_update = kf_.get_x();

        // ===== 定位模式：位姿门控（LOCKED 下大跳变拒绝，可自适应）=====
        bool gate_rejected = false;
        if (is_localization_mode())
        {
            const double res_ratio = (res_mean_last_ > 0 && max_residual_for_good_ > 0)
                                         ? res_mean_last_ / max_residual_for_good_ : 0.0;
            if (!pose_gate_->allow(state_before_update, state_after_update,
                                   state_machine_->state(), res_ratio))
            {
                kf_.change_x(state_before_update);
                kf_.change_P(P_before_update);
                state_point_ = state_before_update;
                accept_lidar_update_ = false;
                gate_rejected = true;
            }
            else
            {
                state_point_ = state_after_update;
                accept_lidar_update_ = true;
            }
        }
        else
        {
            state_point_ = state_after_update;
            accept_lidar_update_ = true;
        }
        refresh_pose_cache_from_state();

        if (is_localization_mode())
        {
            MatchQuality q;
            q.correspondence_valid = (effct_feat_num_ >= 1);
            q.gate_rejected = gate_rejected;
            q.effct_feat_num = effct_feat_num_;
            q.res_mean_last = res_mean_last_;
            const double time_since_start = Measures.lidar_beg_time - first_lidar_time_;
            state_machine_->update(q, time_since_start, now().seconds());

            const bool is_lost =
                state_machine_->state() == LocalizationTrackingState::TRACKING_LOST;
            const bool just_entered_lost = !was_lost && is_lost;

            if (is_lost)
            {
                accept_lidar_update_ = false;
                if (just_entered_lost && state_machine_->has_last_locked())
                {
                    kf_.change_x(last_locked_state_);
                    kf_.change_P(last_locked_covariance_);
                    state_point_ = last_locked_state_;
                }
                else
                {
                    kf_.change_x(frame_entry_state);
                    kf_.change_P(frame_entry_covariance);
                    state_point_ = frame_entry_state;
                }

                refresh_pose_cache_from_state();
                publish_odometry();
                if (path_en_) publish_path();
                publish_localization_status_overlay();
                return;
            }

            if (accept_lidar_update_)
            {
                state_machine_->notify_acceptable_snapshot();
            }
            // 维护近期扫描滚动缓冲（仅 TRACKING/LOCKED 的正常帧），供 LOST 后 NDT 源子图使用
            push_recent_scan(feats_down_body_, state_point_);
            if (state_machine_->state() == LocalizationTrackingState::TRACKING_LOCKED &&
                state_machine_->current_match_good())
            {
                last_locked_state_ = state_point_;
                last_locked_covariance_ = kf_.get_P();
                state_machine_->notify_good_snapshot();
            }

            if (state_machine_->just_recovered_from_lost())
            {
                state_machine_->clear_just_recovered();
            }
        }

        // ===== 建图模式：增量建图 =====
        if (is_mapping_mode())
        {
            map_incremental();
        }

        // ===== 发布 =====
        publish_odometry();
        if (path_en_) publish_path();
        publish_frame_world();
        publish_frame_body();
        publish_localization_status_overlay();

        // 先验地图：transient_local 保证后加入订阅者收到；此参数控制是否周期重发
        if (is_localization_mode())
        {
            prior_map_pub_counter_++;
            if (!prior_map_published_once_ || prior_map_pub_counter_ >= prior_map_pub_interval_)
            {
                publish_prior_map(pub_laser_map_);
                prior_map_published_once_ = true;
                prior_map_pub_counter_ = 0;
            }
        }

        if (runtime_pos_log_)
        {
            euler_cur_ = SO3ToEuler(state_point_.rot);
            fout_out_ << std::setw(20) << Measures.lidar_beg_time - first_lidar_time_ << " "
                      << euler_cur_.transpose() << " " << state_point_.pos.transpose() << " "
                      << state_point_.offset_T_L_I.transpose() << " " << state_point_.vel.transpose() << " "
                      << state_point_.bg.transpose() << " " << state_point_.ba.transpose() << " "
                      << state_point_.grav << " " << feats_undistort_->points.size() << "\n";
        }
    }

    void save_pcd_if_needed()
    {
        if (pcd_wait_save_->empty() || !pcd_save_en_) return;

        const std::filesystem::path pcd_directory =
            std::filesystem::path(ROOT_DIR) / "PCD";
        std::error_code error;
        std::filesystem::create_directories(pcd_directory, error);
        if (error)
        {
            RCLCPP_ERROR(get_logger(), "Failed to create PCD directory '%s': %s",
                         pcd_directory.string().c_str(), error.message().c_str());
            return;
        }

        std::filesystem::path output_path;
        do
        {
            std::ostringstream filename;
            filename << "scans_" << std::setw(6) << std::setfill('0')
                     << pcd_file_index_ << ".pcd";
            output_path = pcd_directory / filename.str();
            if (std::filesystem::exists(output_path)) ++pcd_file_index_;
        }
        while (std::filesystem::exists(output_path));

        pcl::PCDWriter pcd_writer;
        if (pcd_writer.writeBinary(output_path.string(), *pcd_wait_save_) != 0)
        {
            RCLCPP_ERROR(get_logger(), "Failed to save PCD to '%s'",
                         output_path.string().c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "Saved PCD to '%s'", output_path.string().c_str());
        pcd_wait_save_->clear();
        pcd_frame_count_ = 0;
        ++pcd_file_index_;
    }

    bool is_mapping_mode() const { return run_mode_ == RunMode::MODE_MAPPING; }
    bool is_localization_mode() const { return run_mode_ == RunMode::MODE_LOCALIZATION; }

    // ============================ 成员 ============================
    // 参数
    bool path_en_{true}, scan_pub_en_{true}, dense_pub_en_{true}, scan_body_pub_en_{true};
    double path_min_distance_{0.05};
    int path_max_length_{10000};
    bool time_sync_en_{false}, extrinsic_est_en_{true}, gravity_align_en_{true};
    bool runtime_pos_log_{false}, pcd_save_en_{false};
    int NUM_MAX_ITERATIONS_{4}, pcd_save_interval_{-1};
    int pcd_frame_count_{0}, pcd_file_index_{0};
    double time_diff_lidar_to_imu_{0.0};
    double filter_size_corner_min_{0}, filter_size_surf_min_{0}, filter_size_map_min_{0};
    double fov_deg_{0}, det_range_{300.0};
    double gyr_cov_{0.1}, acc_cov_{0.1}, b_gyr_cov_{0.0001}, b_acc_cov_{0.0001};
    double cube_len_{200.0}, map_voxel_size_{0.5};
    double FOV_DEG_{0}, HALF_FOV_COS_{0};
    double max_residual_for_good_{0.20};
    int prior_map_pub_interval_{50};
    std::string lid_topic_, imu_topic_, map_file_path_, world_frame_;
    V3D extrinT_{0, 0, 0};
    M3D extrinR_{Matrix3d::Identity()};
    RunMode run_mode_{RunMode::MODE_MAPPING};

    // 组件
    std::shared_ptr<Preprocess> p_pre_{new Preprocess()};
    std::shared_ptr<ImuProcess> p_imu_{new ImuProcess()};
    std::unique_ptr<TrackingStateMachine> state_machine_;
    std::unique_ptr<PoseGate> pose_gate_;

    // 缓冲
    std::mutex mtx_buffer_;
    std::deque<double> time_buffer_;
    std::deque<PointCloudXYZI::Ptr> lidar_buffer_;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_;
    double last_timestamp_lidar_{0}, last_timestamp_imu_{-1.0}, last_timestamp_lidar_prev_{0};
    double timediff_lidar_wrt_imu_{0.0};
    bool timediff_set_flg_{false}, lidar_pushed_{false};
    int scan_num_{0};
    double lidar_mean_scantime_{0.0}, lidar_end_time_{0};

    // 点云与滤波器
    KD_TREE<PointType> ikdtree_;
    PointCloudXYZI::Ptr feats_undistort_{new PointCloudXYZI()};
    PointCloudXYZI::Ptr feats_down_body_{new PointCloudXYZI()};
    PointCloudXYZI::Ptr feats_down_world_{new PointCloudXYZI()};
    PointCloudXYZI::Ptr normvec_{new PointCloudXYZI(100000, 1)};
    PointCloudXYZI::Ptr laserCloudOri_{new PointCloudXYZI(100000, 1)};
    PointCloudXYZI::Ptr corr_normvect_{new PointCloudXYZI(100000, 1)};
    PointCloudXYZI::Ptr pcd_wait_save_{new PointCloudXYZI()};
    pcl::VoxelGrid<PointType> downSizeFilterSurf_;
    std::vector<PointVector> Nearest_Points_;
    std::vector<uint8_t> point_selected_surf_;
    std::vector<float> res_last_;
    int feats_down_size_{0};

    // EKF
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;
    state_ikfom state_point_;
    state_ikfom last_locked_state_;
    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov last_locked_covariance_{
        esekfom::esekf<state_ikfom, 12, input_ikfom>::cov::Identity()};
    V3D euler_cur_{0, 0, 0};
    V3D pos_lid_{0, 0, 0};
    geometry_msgs::msg::Quaternion geoQuat_;

    // 重定位状态
    bool prior_map_loaded_{false};
    bool prior_map_tree_built_{false};
    bool prior_map_published_once_{false};
    int prior_map_pub_counter_{0};
    PointCloudXYZI::Ptr prior_map_raw_{new PointCloudXYZI()};
    PointCloudXYZI::Ptr prior_map_ds_{new PointCloudXYZI()};
    double total_residual_{0.0};
    double res_mean_last_{0.05};
    int effct_feat_num_{0};
    bool accept_lidar_update_{true};

    // ===== NDT 重定位后端（LOST 恢复 + 启动全局重定位）=====
    bool relocalization_enabled_{false};
    int relocalization_trigger_interval_{5};
    int relocalization_submap_frames_{5};
    double relocalization_local_map_radius_{30.0};
    double relocalization_cov_inflation_{10.0};
    int relocalization_frame_counter_{0};
    std::unique_ptr<NdtRelocalizer> ndt_relocalizer_;
    // 近期扫描滚动缓冲：每帧同时保存其 body 系点云与当时的 T_WI（map←IMU），
    // 供 make_source_submap() 把各历史帧按帧间相对位姿统一到"最新帧 body 系"后拼接。
    struct RecentScan
    {
        PointCloudXYZI::Ptr cloud;   // 该帧下采样后的 body 系（LiDAR 系）点云
        Eigen::Matrix3d R_WI;        // 该帧 T_WI 的旋转
        Eigen::Vector3d p_WI;        // 该帧 T_WI 的平移
        Eigen::Matrix3d R_LI;        // 该帧 LiDAR→IMU 外参旋转
        Eigen::Vector3d t_LI;        // 该帧 LiDAR→IMU 外参平移
    };
    std::deque<RecentScan> recent_scans_;
    // 启动全局重定位：仅在提供 initial_pose 时启用（无初值不做裸 NDT 全局搜索）
    bool initial_pose_provided_{false};
    bool initial_relocalization_done_{false};
    Eigen::Matrix4d initial_guess_map_body_{Eigen::Matrix4d::Identity()};

    // 建图模式局部地图
    BoxPointType LocalMap_Points_;
    bool Localmap_Initialized_{false};
    std::vector<BoxPointType> cub_needrm_;
    static constexpr float MOV_THRESHOLD = 1.5f;

    // 标志
    bool flg_first_scan_{true};
    bool flg_EKF_inited_{false};
    double first_lidar_time_{0.0};

    // 消息缓存
    nav_msgs::msg::Path path_;
    geometry_msgs::msg::PoseStamped msg_body_pos_;

    // ROS 接口
    rclcpp::SubscriptionBase::SharedPtr sub_pcl_;
    rclcpp::SubscriptionBase::SharedPtr sub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_world_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_map_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_aft_mapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_status_marker_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // 日志
    std::ofstream fout_out_;
};

} // namespace fast_lio

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<fast_lio::FastLioNode>();
    std::signal(SIGINT, [](int sig)
    {
        (void)sig;
        rclcpp::shutdown();
    });
    node->run();
    rclcpp::shutdown();
    return 0;
}
