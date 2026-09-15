#ifndef FAST_LIO_POSE_GATE_HPP
#define FAST_LIO_POSE_GATE_HPP

#include <Eigen/Eigen>
#include "use-ikfom.hpp"
#include "localization/state_machine.hpp"

namespace fast_lio
{

/* 位姿门控参数（对应 yaml localization/max_position_jump_for_update_locked + 新增自适应） */
struct PoseGateConfig
{
    double max_position_jump_for_update_locked = 1.0;  // 米
    // 自适应门控（新增，默认关闭保持原行为）
    bool adaptive_gating_en = false;
    double adaptive_scale_max = 3.0;   // 自适应允许的最大放大倍数
};

/* LOCKED 状态下的位姿跳变门控：
   拒绝 EKF 更新后位置跳变过大的帧，防止先验地图定位被坏匹配拉跑。 */
class PoseGate
{
public:
    explicit PoseGate(const PoseGateConfig &cfg) : cfg_(cfg) {}

    /* state_gate：当前状态机状态；仅 LOCKED 时门控生效（与原版一致） */
    bool allow(const state_ikfom &state_before,
               const state_ikfom &state_after,
               LocalizationTrackingState state_gate,
               double res_ratio_hint = 0.0) const
    {
        if (state_gate != LocalizationTrackingState::TRACKING_LOCKED)
        {
            return true;
        }
        double threshold = cfg_.max_position_jump_for_update_locked;
        if (cfg_.adaptive_gating_en && res_ratio_hint > 0.0)
        {
            const double scale_max = cfg_.adaptive_scale_max >= 1.0 ? cfg_.adaptive_scale_max : 1.0;
            const double scale = 1.0 + std::min(res_ratio_hint, scale_max - 1.0);
            threshold *= (scale >= 1.0 ? scale : 1.0);
        }
        const double delta_pos = (state_after.pos - state_before.pos).norm();
        return delta_pos <= threshold;
    }

private:
    PoseGateConfig cfg_;
};

} // namespace fast_lio

#endif // FAST_LIO_POSE_GATE_HPP
