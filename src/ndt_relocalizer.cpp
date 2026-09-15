#include "ndt_relocalizer.hpp"

#include <cmath>
#include <algorithm>

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

namespace fast_lio
{

bool NdtRelocalizer::toXYZDownsampled(const PointCloudXYZI::Ptr &in,
                                      double voxel,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr &out)
{
    out.reset(new pcl::PointCloud<pcl::PointXYZ>());
    if (!in || in->empty())
    {
        return false;
    }

    // 仅取 xyz，剥离 normal/intensity/curvature，避免无关字段影响配准
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>());
    xyz->reserve(in->size());
    for (const auto &p : in->points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        {
            continue;
        }
        pcl::PointXYZ q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        xyz->push_back(q);
    }
    if (xyz->empty())
    {
        return false;
    }

    // 调用方（节点参数读取）已保证 voxel > 0；此处不再二次兜底，避免掩盖配置错误
    const float leaf = static_cast<float>(voxel);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(leaf, leaf, leaf);
    vg.setInputCloud(xyz);
    vg.filter(*out);
    return !out->empty();
}

NdtResult NdtRelocalizer::align(const PointCloudXYZI::Ptr &source,
                                const PointCloudXYZI::Ptr &target,
                                const Eigen::Matrix4d &init_guess) const
{
    NdtResult result;

    pcl::PointCloud<pcl::PointXYZ>::Ptr src;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt;
    if (!toXYZDownsampled(source, cfg_.source_voxel_size, src) ||
        !toXYZDownsampled(target, cfg_.target_voxel_size, tgt))
    {
        return result; // 空输入/无有效点：直接失败，不调用 NDT
    }

    result.source_points = static_cast<int>(src->size());
    result.target_points = static_cast<int>(tgt->size());

    // 退化保护：点数不足直接失败
    if (result.source_points < cfg_.min_source_points ||
        result.target_points < cfg_.min_target_points)
    {
        return result;
    }

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setResolution(static_cast<float>(cfg_.ndt_resolution));
    ndt.setStepSize(cfg_.ndt_step_size);
    ndt.setTransformationEpsilon(cfg_.ndt_trans_eps);
    ndt.setMaximumIterations(cfg_.ndt_max_iter);
    ndt.setNumberOfThreads(static_cast<unsigned int>(cfg_.ndt_num_threads));
    ndt.setInputSource(src);
    ndt.setInputTarget(tgt);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    const Eigen::Matrix4f guess = init_guess.cast<float>();
    ndt.align(aligned, guess);

    if (!ndt.hasConverged())
    {
        return result;
    }

    const Eigen::Matrix4f tf = ndt.getFinalTransformation();
    result.transform = tf.cast<double>();

    // 收敛 + 适应度
    // 注意：getFitnessScore 默认 max_range 为 double::max()（无界），远点离群匹配会无界抬高
    // fitness，使 max_fitness_score 门限失真。故按 NDT 分辨率限定有效匹配距离（约一个体素）。
    result.fitness = ndt.getFitnessScore(cfg_.ndt_resolution);
    if (!std::isfinite(result.fitness) || result.fitness > cfg_.max_fitness_score)
    {
        return result;
    }

    // 相对初值的平移校正量
    result.translation_delta = (result.transform.block<3, 1>(0, 3) -
                                init_guess.block<3, 1>(0, 3)).norm();

    // 相对初值的旋转校正量：两旋转的相对角
    const Eigen::Matrix3d R0 = init_guess.block<3, 3>(0, 0);
    const Eigen::Matrix3d R1 = result.transform.block<3, 3>(0, 0);
    const Eigen::Matrix3d Rdiff = R1 * R0.transpose();
    const double cos_theta = std::min(1.0, std::max(-1.0, (Rdiff.trace() - 1.0) * 0.5));
    result.rotation_delta_deg = std::acos(cos_theta) * 180.0 / M_PI;

    if (result.translation_delta > cfg_.max_translation_delta ||
        result.rotation_delta_deg > cfg_.max_rotation_delta_deg)
    {
        return result;
    }

    result.success = true;
    return result;
}

} // namespace fast_lio
