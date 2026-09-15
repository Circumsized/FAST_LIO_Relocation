#ifndef FAST_LIO_NDT_RELOCALIZER_HPP
#define FAST_LIO_NDT_RELOCALIZER_HPP

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "preprocess.h"   // PointType / PointCloudXYZI

namespace fast_lio
{

/* NDT 重定位后端配置。
   - 仅使用点的 xyz 坐标参与配准（source/target 在内部转为 pcl::PointXYZ）。
   - 所有 *_delta 门限均相对“传入的初值 init_guess”。 */
struct NdtRelocalizerConfig
{
    double ndt_resolution = 1.0;          // NDT 体素分辨率（米）
    double ndt_step_size = 0.1;           // 线搜索最大步长
    double ndt_trans_eps = 0.01;          // 收敛判定：变换增量阈值
    int ndt_max_iter = 30;                // 最大迭代次数
    int ndt_num_threads = 0;              // 0 => auto（复用 PCL OpenMP）
    double source_voxel_size = 0.5;       // 源点云下采样体素
    double target_voxel_size = 0.5;       // 目标点云下采样体素
    double max_fitness_score = 1.0;       // 接受门：适应度上限
    double max_translation_delta = 8.0;   // 接受门：相对初值的平移校正上限（米）
    double max_rotation_delta_deg = 20.0; // 接受门：相对初值的旋转校正上限（度）
    int min_source_points = 200;          // 源点云最少有效点
    int min_target_points = 1000;         // 目标点云最少有效点
};

/* NDT 对齐结果。transform 为 T_map_body（源点云位于 body 系时）。 */
struct NdtResult
{
    bool success = false;
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    double fitness = 1e9;
    double translation_delta = 0.0;       // 相对初值的平移校正（米）
    double rotation_delta_deg = 0.0;      // 相对初值的旋转校正（度）
    int source_points = 0;
    int target_points = 0;
};

/* 独立的 NDT 重定位组件（纯几何，不依赖 ROS 节点逻辑，便于单测）。
   使用 PCL 原生 pcl::NormalDistributionsTransform（Humble 自带，OpenMP 多线程）。 */
class NdtRelocalizer
{
public:
    explicit NdtRelocalizer(const NdtRelocalizerConfig &cfg) : cfg_(cfg) {}

    /* 将 source 对齐到 target。
       - source/target：PCL 点云（含额外字段也可，内部仅取 xyz）；
       - init_guess：T_map_body 初值（必须由调用方提供可信初值，本组件不做全局搜索）。
       返回 NdtResult；任何前置条件不满足时 success=false。 */
    NdtResult align(const PointCloudXYZI::Ptr &source,
                    const PointCloudXYZI::Ptr &target,
                    const Eigen::Matrix4d &init_guess) const;

    const NdtRelocalizerConfig &config() const { return cfg_; }

private:
    /* 抽取 xyz 并按 voxel 下采样；返回 false 表示输入为空或无有效输出。 */
    static bool toXYZDownsampled(const PointCloudXYZI::Ptr &in,
                                 double voxel,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr &out);

    NdtRelocalizerConfig cfg_;
};

} // namespace fast_lio

#endif // FAST_LIO_NDT_RELOCALIZER_HPP
