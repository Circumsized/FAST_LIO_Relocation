#ifndef IMU_PROCESSING_HPP
#define IMU_PROCESSING_HPP

#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <memory>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "use-ikfom.hpp"
#include "preprocess.h"

/// *************Preconfiguration
#define MAX_INI_COUNT (10)

/// 点云按 curvature 字段排序
inline bool time_list(const PointType &x, const PointType &y) { return (x.curvature < y.curvature); };

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuProcess();
  ~ImuProcess();
  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void set_gravity_align_enable(bool en) { gravity_align_en_ = en; }
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas,
               esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
               PointCloudXYZI::Ptr pcl_un_);
  std::ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time = 0.0;
  int lidar_type = AVIA;
 private:
  void IMU_init(const MeasureGroup &meas,
                esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                int &N);
  void UndistortPcl(const MeasureGroup &meas,
                    esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                    PointCloudXYZI &pcl_in_out);
  PointCloudXYZI::Ptr cur_pcl_un_;
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> v_imu_;
  std::vector<Pose6D> IMUpose;
  std::vector<M3D>    v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;
  double last_lidar_end_time_;
  int  init_iter_num = 1;
  bool b_first_frame_ = true;
  bool imu_need_init_ = true;
  // ===== gravity alignment switch =====
  // 从 yaml 读取，决定是否启用“虚拟水平重力对齐”
  bool gravity_align_en_ = true;
  // ===== virtual level alignment =====
  // 把“倾斜安装下测得的 IMU / 点云”旋到虚拟水平坐标系
  Eigen::Quaterniond q_align_ = Eigen::Quaterniond::Identity();
  // 是否已经成功求得对齐旋转
  bool has_align_ = false;
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc       = V3D(0.1, 0.1, 0.1);
  cov_gyr       = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
  cov_acc_scale = cov_acc;
  cov_gyr_scale = cov_gyr;
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last   = Zero3d;
  acc_s_last    = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  q_align_ = Eigen::Quaterniond::Identity();
  has_align_ = false;
  last_lidar_end_time_ = 0.0;
  last_imu_ = std::make_shared<sensor_msgs::msg::Imu>();
  gravity_align_en_ = true;
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()
{
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last   = Zero3d;
  acc_s_last    = Zero3d;
  imu_need_init_   = true;
  start_timestamp_ = -1;
  init_iter_num    = 1;
  v_imu_.clear();
  IMUpose.clear();
  last_imu_ = std::make_shared<sensor_msgs::msg::Imu>();
  cur_pcl_un_.reset(new PointCloudXYZI());
  q_align_ = Eigen::Quaterniond::Identity();
  has_align_ = false;
  last_lidar_end_time_ = 0.0;
}

void ImuProcess::Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu)
{
  Reset();
  start_timestamp_ = start_timestamp;
  last_imu_ = lastimu;
  last_lidar_end_time_ = start_timestamp;
}

void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

void ImuProcess::IMU_init(const MeasureGroup &meas,
                          esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                          int &N)
{
  /** 负责：
   * 1. 累计静止阶段 IMU，估计 mean_acc / mean_gyr
   * 2. 如果开关开启，则建立“虚拟水平坐标系”
   * 3. 初始化滤波器状态 */
  V3D cur_acc, cur_gyr;
  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    first_lidar_time = meas.lidar_beg_time;
    last_lidar_end_time_ = meas.lidar_beg_time;
  }
  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;
    cov_acc = cov_acc * (N - 1.0) / N +
              (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N +
              (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);
    N++;
  }
  state_ikfom init_state = kf_state.get_x();
  V3D init_mean_gyr = mean_gyr;
  if (gravity_align_en_)
  {
    // ===== build virtual level frame =====
    Eigen::Vector3d acc0(mean_acc.x(), mean_acc.y(), mean_acc.z());
    if (acc0.norm() > 1e-6)
    {
      acc0.normalize();
      q_align_ = Eigen::Quaterniond::FromTwoVectors(acc0, Eigen::Vector3d::UnitZ());
      has_align_ = true;
      Eigen::Vector3d gyr0(mean_gyr.x(), mean_gyr.y(), mean_gyr.z());
      gyr0 = q_align_ * gyr0;
      init_mean_gyr = V3D(gyr0.x(), gyr0.y(), gyr0.z());
    }
    else
    {
      q_align_ = Eigen::Quaterniond::Identity();
      has_align_ = false;
    }
    // 输入数据已经会被旋到水平系，所以滤波器从 identity 开始
    init_state.rot = Eye3d;
    // 水平系下重力固定朝 -Z
    init_state.grav = S2(V3D(0.0, 0.0, -G_m_s2));
  }
  else
  {
    // ===== 原版逻辑 =====
    q_align_ = Eigen::Quaterniond::Identity();
    has_align_ = false;
    double mean_acc_norm = mean_acc.norm();
    if (mean_acc_norm > 1e-6)
    {
      init_state.grav = S2(- mean_acc / mean_acc_norm * G_m_s2);
    }
    else
    {
      init_state.grav = S2(V3D(0.0, 0.0, -G_m_s2));
    }
  }
  init_state.bg  = init_mean_gyr;
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);
  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
  init_P(21,21) = init_P(22,22) = 0.00001;
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();
}

void ImuProcess::UndistortPcl(const MeasureGroup &meas,
                              esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                              PointCloudXYZI &pcl_out)
{
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  const double imu_end_time = stamp_to_sec(v_imu.back()->header.stamp);
  double pcl_beg_time = meas.lidar_beg_time;
  double pcl_end_time = meas.lidar_end_time;
  if (lidar_type == MARSIM)
  {
    pcl_beg_time = last_lidar_end_time_;
    pcl_end_time = meas.lidar_beg_time;
  }
  pcl_out = *(meas.lidar);
  std::sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last,
                               imu_state.vel, imu_state.pos,
                               imu_state.rot.toRotationMatrix()));
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;
  double dt = 0.0;
  input_ikfom in;
  in.acc = Zero3d;
  in.gyro = Zero3d;
  bool has_valid_input = false;
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);
    if (stamp_to_sec(tail->header.stamp) < last_lidar_end_time_) continue;
    angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                  0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                  0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
               0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
               0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);
    const double mean_acc_norm = mean_acc.norm();
    if (mean_acc_norm > 1.0)
    {
      acc_avr = acc_avr * (G_m_s2 / mean_acc_norm);
    }
    // ===== rotate IMU measurements into virtual level frame =====
    if (gravity_align_en_ && has_align_)
    {
      Eigen::Vector3d w(angvel_avr.x(), angvel_avr.y(), angvel_avr.z());
      Eigen::Vector3d a(acc_avr.x(), acc_avr.y(), acc_avr.z());
      w = q_align_ * w;
      a = q_align_ * a;
      angvel_avr = V3D(w.x(), w.y(), w.z());
      acc_avr    = V3D(a.x(), a.y(), a.z());
    }
    if (stamp_to_sec(head->header.stamp) < last_lidar_end_time_)
      dt = stamp_to_sec(tail->header.stamp) - last_lidar_end_time_;
    else
      dt = stamp_to_sec(tail->header.stamp) - stamp_to_sec(head->header.stamp);
    in.acc = acc_avr;
    in.gyro = angvel_avr;
    has_valid_input = true;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    kf_state.predict(dt, Q, in);
    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
    for (int i = 0; i < 3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];
    }
    double &&offs_t = stamp_to_sec(tail->header.stamp) - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last,
                                 imu_state.vel, imu_state.pos,
                                 imu_state.rot.toRotationMatrix()));
  }
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  if (has_valid_input)
  {
    kf_state.predict(dt, Q, in);
    imu_state = kf_state.get_x();
  }
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;
  if (pcl_out.points.begin() == pcl_out.points.end()) return;
  if (lidar_type != MARSIM)
  {
    auto it_pcl = pcl_out.points.end() - 1;
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
    {
      auto head = it_kp - 1;
      auto tail = it_kp;
      R_imu << MAT_FROM_ARRAY(head->rot);
      vel_imu << VEC_FROM_ARRAY(head->vel);
      pos_imu << VEC_FROM_ARRAY(head->pos);
      acc_imu << VEC_FROM_ARRAY(tail->acc);
      angvel_avr << VEC_FROM_ARRAY(tail->gyr);
      for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--)
      {
        dt = it_pcl->curvature / double(1000) - head->offset_time;
        M3D R_i(R_imu * Exp(angvel_avr, dt));
        V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
        V3D P_compensate =
            imu_state.offset_R_L_I.conjugate() *
            (imu_state.rot.conjugate() *
                 (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) -
             imu_state.offset_T_L_I);
        // ===== rotate LiDAR point into virtual level frame =====
        if (gravity_align_en_ && has_align_)
        {
          Eigen::Vector3d p(P_compensate(0), P_compensate(1), P_compensate(2));
          p = q_align_ * p;
          P_compensate = V3D(p.x(), p.y(), p.z());
        }
        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);
        if (it_pcl == pcl_out.points.begin()) break;
      }
    }
  }
  else
  {
    if (gravity_align_en_ && has_align_)
    {
      for (auto &pt : pcl_out.points)
      {
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        p = q_align_ * p;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
      }
    }
  }
}

void ImuProcess::Process(const MeasureGroup &meas,
                         esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                         PointCloudXYZI::Ptr cur_pcl_un_)
{
  if (meas.imu.empty()) { return; }
  if (meas.lidar == nullptr || cur_pcl_un_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ImuProcess"), "IMU processing output or lidar measure is null");
    return;
  }
  if (imu_need_init_)
  {
    IMU_init(meas, kf_state, init_iter_num);
    last_imu_ = meas.imu.back();
    if (init_iter_num > MAX_INI_COUNT)
    {
      imu_need_init_ = false;
      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      RCLCPP_INFO(rclcpp::get_logger("ImuProcess"), "IMU Initial Done");
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"), std::ios::out);
    }
    return;
  }
  UndistortPcl(meas, kf_state, *cur_pcl_un_);
}

#endif
