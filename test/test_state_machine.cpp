// 状态机与位姿门控单元测试（脱离 ROS 运行）
#include <gtest/gtest.h>

#include <cmath>

#include "localization/state_machine.hpp"
#include "localization/pose_gate.hpp"

using fast_lio::LocalizationTrackingState;
using fast_lio::MatchQuality;
using fast_lio::PoseGate;
using fast_lio::PoseGateConfig;
using fast_lio::TrackingStateMachine;
using fast_lio::TrackingStateMachineConfig;
using fast_lio::ExternalRecoveryStatus;

namespace
{

MatchQuality make_quality(int n_pts, double res, bool accepted = true)
{
    MatchQuality q;
    q.correspondence_valid = accepted;
    q.effct_feat_num = n_pts;
    q.res_mean_last = res;
    return q;
}

TrackingStateMachineConfig default_cfg()
{
    TrackingStateMachineConfig cfg;
    cfg.min_effective_points_for_tracking = 15;
    cfg.max_residual_for_tracking = 0.40;
    cfg.min_effective_points_for_good = 30;
    cfg.max_residual_for_good = 0.20;
    cfg.unlock_to_tracking_streak = 3;
    cfg.good_match_streak_to_lock = 5;
    cfg.bad_match_streak_to_lost = 5;
    cfg.min_time_before_lock_sec = 2.0;
    cfg.lost_recovery_en = false;
    cfg.lost_recovery_streak = 10;
    cfg.lost_recovery_max_attempts = 3;
    cfg.lost_recovery_cooldown_sec = 5.0;
    return cfg;
}

} // namespace

// ---------- 匹配质量评估边界 ----------
TEST(MatchEvaluate, AcceptableBoundaries)
{
    const auto cfg = default_cfg();
    // 点数不足
    EXPECT_FALSE(fast_lio::is_match_acceptable(make_quality(14, 0.01), cfg));
    EXPECT_TRUE(fast_lio::is_match_acceptable(make_quality(15, 0.01), cfg));
    // 残差过大
    EXPECT_FALSE(fast_lio::is_match_acceptable(make_quality(100, 0.41), cfg));
    EXPECT_TRUE(fast_lio::is_match_acceptable(make_quality(100, 0.40), cfg));
    // 更新被拒绝
    EXPECT_FALSE(fast_lio::is_match_acceptable(make_quality(100, 0.01, false), cfg));
}

TEST(MatchEvaluate, GoodBoundaries)
{
    const auto cfg = default_cfg();
    EXPECT_FALSE(fast_lio::is_match_good(make_quality(29, 0.01), cfg));
    EXPECT_TRUE(fast_lio::is_match_good(make_quality(30, 0.01), cfg));
    EXPECT_FALSE(fast_lio::is_match_good(make_quality(100, 0.21), cfg));
    EXPECT_TRUE(fast_lio::is_match_good(make_quality(100, 0.20), cfg));
    EXPECT_FALSE(fast_lio::is_match_good(make_quality(100, 0.01, false), cfg));
}

// ---------- 状态机转移 ----------
TEST(StateMachine, UnlockedToTrackingNeedsStreak)
{
    auto sm = TrackingStateMachine(default_cfg());
    const auto good_q = make_quality(100, 0.05);
    sm.update(good_q, 0.0);
    sm.update(good_q, 0.1);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_UNLOCKED);  // streak=2 不足
    sm.update(good_q, 0.2);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);  // streak=3 达标
}

TEST(StateMachine, TrackingToLockedNeedsTimeAndGoodStreak)
{
    auto sm = TrackingStateMachine(default_cfg());
    const auto good_q = make_quality(100, 0.05);
    const auto acceptable_q = make_quality(20, 0.30);
    for (int i = 0; i < 3; i++) sm.update(good_q, 0.1 * i);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);
    for (int i = 0; i < 4; i++) sm.update(good_q, 2.0 + 0.1 * i);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);
    sm.update(acceptable_q, 2.5);
    for (int i = 0; i < 4; i++) sm.update(good_q, 3.0 + 0.1 * i);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);
    sm.update(good_q, 3.5);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOCKED);
    EXPECT_TRUE(sm.has_last_locked());
}

TEST(StateMachine, TrackingToLostOnBadStreak)
{
    auto sm = TrackingStateMachine(default_cfg());
    for (int i = 0; i < 3; i++) sm.update(make_quality(100, 0.05), 0.1 * i);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);
    const auto bad_q = make_quality(2, 5.0, false);
    for (int i = 0; i < 4; i++) sm.update(bad_q, 1.0 + 0.1 * i);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);  // bad streak=4
    sm.update(bad_q, 2.0);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
}

TEST(StateMachine, LockedDowngradesToTrackingWhenNotGood)
{
    auto sm = TrackingStateMachine(default_cfg());
    for (int i = 0; i < 3; i++) sm.update(make_quality(100, 0.05), 0.1 * i);
    for (int i = 0; i < 8; i++) sm.update(make_quality(100, 0.05), 3.0 + 0.1 * i);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOCKED);
    // acceptable 但不 good → 降级
    sm.update(make_quality(40, 0.30), 5.0);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);
}

TEST(StateMachine, LostStaysFrozenWhenRecoveryDisabled)
{
    auto sm = TrackingStateMachine(default_cfg());
    const auto bad_q = make_quality(2, 5.0, false);
    for (int i = 0; i < 3; i++) sm.update(make_quality(100, 0.05), 0.1 * i);
    for (int i = 0; i < 5; i++) sm.update(bad_q, 1.0 + i);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
    // 长时间好匹配也无法恢复（默认关闭恢复，与原版一致）
    for (int i = 0; i < 100; i++) sm.update(make_quality(100, 0.05), 10.0 + i * 0.1);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
}

TEST(StateMachine, LostRecoversWhenEnabled)
{
    auto cfg = default_cfg();
    cfg.lost_recovery_en = true;
    cfg.lost_recovery_streak = 10;
    auto sm = TrackingStateMachine(cfg);
    for (int i = 0; i < 3; i++) sm.update(make_quality(100, 0.05), 0.1 * i);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);
    const auto bad_q = make_quality(2, 5.0, false);
    for (int i = 0; i < 5; i++) sm.update(bad_q, 1.0 + 0.1 * i);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
    // 连续 10 帧 acceptable 且不在冷却 → 恢复
    for (int i = 0; i < 10; i++) sm.update(make_quality(100, 0.05), 10.0 + i * 0.1, 100.0);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);
    EXPECT_EQ(sm.recovery_attempts(), 1);
}

TEST(StateMachine, LostRecoveryRespectsMaxAttempts)
{
    auto cfg = default_cfg();
    cfg.lost_recovery_en = true;
    cfg.lost_recovery_streak = 3;
    cfg.lost_recovery_max_attempts = 2;
    cfg.lost_recovery_cooldown_sec = 0.0;
    auto sm = TrackingStateMachine(cfg);
    const auto bad_q = make_quality(2, 5.0, false);
    for (int i = 0; i < 3; i++) sm.update(make_quality(100, 0.05), 0.1 * i);
    for (int i = 0; i < 5; i++) sm.update(bad_q, 1.0);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
    for (int i = 0; i < 3; i++) sm.update(make_quality(100, 0.05), 10.0, 100.0);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);
    for (int i = 0; i < 5; i++) sm.update(bad_q, 11.0);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
    for (int i = 0; i < 3; i++) sm.update(make_quality(100, 0.05), 20.0, 200.0);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);
    EXPECT_EQ(sm.recovery_attempts(), 2);
    for (int i = 0; i < 5; i++) sm.update(bad_q, 21.0);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
    for (int i = 0; i < 20; i++) sm.update(make_quality(100, 0.05), 30.0, 300.0);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
    EXPECT_EQ(sm.recovery_attempts(), 2);
}

TEST(MatchEvaluate, GateRejectionDoesNotInvalidateGeometry)
{
    const auto cfg = default_cfg();
    auto q = make_quality(100, 0.05);
    q.gate_rejected = true;
    EXPECT_TRUE(fast_lio::is_match_acceptable(q, cfg));
    EXPECT_TRUE(fast_lio::is_match_good(q, cfg));
}

TEST(StateMachine, RecoveryFlagLastsOneUpdate)
{
    auto cfg = default_cfg();
    cfg.lost_recovery_en = true;
    cfg.lost_recovery_streak = 1;
    cfg.lost_recovery_cooldown_sec = 0.0;
    auto sm = TrackingStateMachine(cfg);
    const auto good_q = make_quality(100, 0.05);
    const auto bad_q = make_quality(2, 5.0, false);
    for (int i = 0; i < 3; ++i) sm.update(good_q, 0.1 * i);
    for (int i = 0; i < 5; ++i) sm.update(bad_q, 1.0);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
    sm.update(good_q, 2.0, 10.0);
    EXPECT_TRUE(sm.just_recovered_from_lost());
    sm.update(good_q, 2.1, 10.1);
    EXPECT_FALSE(sm.just_recovered_from_lost());
}

// ---------- 外部重定位恢复（NDT）----------

TEST(ExternalRecovery, RequiresLostState)
{
    auto cfg = default_cfg();
    cfg.lost_recovery_en = true;
    auto sm = TrackingStateMachine(cfg);
    // 处于 UNLOCKED：不是 LOST，应返回 kNotLost
    EXPECT_EQ(sm.on_external_relocalization_success(1.0), ExternalRecoveryStatus::kNotLost);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_UNLOCKED);
}

TEST(ExternalRecovery, DisabledWhenLostRecoveryOff)
{
    auto cfg = default_cfg();
    cfg.lost_recovery_en = false;   // 关闭总开关 → 外部恢复也必须被拒（守护不变量）
    auto sm = TrackingStateMachine(cfg);
    const auto bad_q = make_quality(2, 5.0, false);
    for (int i = 0; i < 3; i++) sm.update(make_quality(100, 0.05), 0.1 * i);
    for (int i = 0; i < 5; i++) sm.update(bad_q, 1.0 + i);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
    EXPECT_EQ(sm.on_external_relocalization_success(100.0),
              ExternalRecoveryStatus::kDisabledOrExhausted);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
}

TEST(ExternalRecovery, SuccessReturnsToTrackingAndResetsStreaks)
{
    auto cfg = default_cfg();
    cfg.lost_recovery_en = true;
    auto sm = TrackingStateMachine(cfg);
    const auto bad_q = make_quality(2, 5.0, false);
    for (int i = 0; i < 3; i++) sm.update(make_quality(100, 0.05), 0.1 * i);
    for (int i = 0; i < 5; i++) sm.update(bad_q, 1.0 + i);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
    EXPECT_EQ(sm.on_external_relocalization_success(100.0), ExternalRecoveryStatus::kSuccess);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);
    EXPECT_EQ(sm.good_streak(), 0);
    EXPECT_EQ(sm.bad_streak(), 0);
    EXPECT_EQ(sm.recovery_attempts(), 1);
}

TEST(ExternalRecovery, SharesBudgetWithSoftRecovery)
{
    auto cfg = default_cfg();
    cfg.lost_recovery_en = true;
    cfg.lost_recovery_max_attempts = 1;   // 仅允许一次恢复
    cfg.lost_recovery_streak = 1;
    cfg.lost_recovery_cooldown_sec = 0.0;
    auto sm = TrackingStateMachine(cfg);
    const auto bad_q = make_quality(2, 5.0, false);
    for (int i = 0; i < 3; i++) sm.update(make_quality(100, 0.05), 0.1 * i);
    for (int i = 0; i < 5; i++) sm.update(bad_q, 1.0 + i);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
    // 外部恢复消耗唯一一次预算
    EXPECT_EQ(sm.on_external_relocalization_success(100.0), ExternalRecoveryStatus::kSuccess);
    EXPECT_EQ(sm.recovery_attempts(), 1);
    // 再次进入 LOST 后，内部软恢复也因预算耗尽无法恢复（共用预算）
    for (int i = 0; i < 5; i++) sm.update(bad_q, 110.0 + i);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
    for (int i = 0; i < 20; i++) sm.update(make_quality(100, 0.05), 120.0, 300.0);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
    EXPECT_EQ(sm.recovery_attempts(), 1);
}

TEST(ExternalRecovery, NoteAttemptConsumesBudgetAndCooldown)
{
    auto cfg = default_cfg();
    cfg.lost_recovery_en = true;
    cfg.lost_recovery_max_attempts = 3;
    cfg.lost_recovery_cooldown_sec = 5.0;
    auto sm = TrackingStateMachine(cfg);
    const auto bad_q = make_quality(2, 5.0, false);
    for (int i = 0; i < 3; i++) sm.update(make_quality(100, 0.05), 0.1 * i);
    for (int i = 0; i < 5; i++) sm.update(bad_q, 1.0 + i);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
    // 首次失败尝试：消耗一次并进入冷却
    EXPECT_TRUE(sm.note_external_recovery_attempt(100.0));
    EXPECT_EQ(sm.recovery_attempts(), 1);
    // 冷却期内再次尝试：拒绝（不消耗）
    EXPECT_FALSE(sm.note_external_recovery_attempt(101.0));
    EXPECT_EQ(sm.recovery_attempts(), 1);
    // 冷却期过后：可再次消耗
    EXPECT_TRUE(sm.note_external_recovery_attempt(200.0));
    EXPECT_EQ(sm.recovery_attempts(), 2);
}

// 复现节点侧控制流（D2）：NDT 通过但状态机 on_external... 被拒（此处因冷却），
// 节点随后调用 note_external_recovery_attempt；只要过了冷却，尝试应被计入（不无限重试）。
TEST(ExternalRecovery, RejectedSuccessStillCountsAttemptAfterCooldown)
{
    auto cfg = default_cfg();
    cfg.lost_recovery_en = true;
    cfg.lost_recovery_max_attempts = 3;
    cfg.lost_recovery_cooldown_sec = 5.0;
    auto sm = TrackingStateMachine(cfg);
    const auto bad_q = make_quality(2, 5.0, false);
    for (int i = 0; i < 3; i++) sm.update(make_quality(100, 0.05), 0.1 * i);
    for (int i = 0; i < 5; i++) sm.update(bad_q, 1.0 + i);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);

    // 先经历一次失败尝试 → 进入冷却（attempts=1）
    EXPECT_TRUE(sm.note_external_recovery_attempt(100.0));
    EXPECT_EQ(sm.recovery_attempts(), 1);

    // 冷却期内：on_external... 被拒（非 Success），节点补记 note_... 也不消耗
    EXPECT_NE(sm.on_external_relocalization_success(101.0), ExternalRecoveryStatus::kSuccess);
    EXPECT_FALSE(sm.note_external_recovery_attempt(101.0));
    EXPECT_EQ(sm.recovery_attempts(), 1);

    // 冷却过后：on_external... 成功恢复，计入一次（attempts=2）
    EXPECT_EQ(sm.on_external_relocalization_success(200.0), ExternalRecoveryStatus::kSuccess);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);
    EXPECT_EQ(sm.recovery_attempts(), 2);
}

// 预算耗尽边界：max_attempts=1，用尽后 on_external... / note_... 均返回失败/未消耗
TEST(ExternalRecovery, ExhaustedBudgetRejectsBothApi)
{
    auto cfg = default_cfg();
    cfg.lost_recovery_en = true;
    cfg.lost_recovery_max_attempts = 1;
    cfg.lost_recovery_cooldown_sec = 0.0;   // 排除冷却干扰，只测预算
    auto sm = TrackingStateMachine(cfg);
    const auto bad_q = make_quality(2, 5.0, false);
    for (int i = 0; i < 3; i++) sm.update(make_quality(100, 0.05), 0.1 * i);
    for (int i = 0; i < 5; i++) sm.update(bad_q, 1.0 + i);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);

    // 用掉唯一一次预算
    EXPECT_TRUE(sm.note_external_recovery_attempt(100.0));
    EXPECT_EQ(sm.recovery_attempts(), 1);

    // 预算耗尽：on_external... 返回 kDisabledOrExhausted，note_... 返回 false（不再消耗）
    EXPECT_EQ(sm.on_external_relocalization_success(200.0),
              ExternalRecoveryStatus::kDisabledOrExhausted);
    EXPECT_FALSE(sm.note_external_recovery_attempt(200.0));
    EXPECT_EQ(sm.recovery_attempts(), 1);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
}

TEST(StateMachine, RecoveryWaitsForCooldown)
{
    auto cfg = default_cfg();
    cfg.lost_recovery_en = true;
    cfg.lost_recovery_streak = 2;
    cfg.lost_recovery_cooldown_sec = 5.0;
    auto sm = TrackingStateMachine(cfg);
    const auto good_q = make_quality(100, 0.05);
    const auto bad_q = make_quality(2, 5.0, false);
    for (int i = 0; i < 3; ++i) sm.update(good_q, 0.1 * i);
    for (int i = 0; i < 5; ++i) sm.update(bad_q, 1.0);
    for (int i = 0; i < 2; ++i) sm.update(good_q, 2.0, 10.0);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);
    for (int i = 0; i < 5; ++i) sm.update(bad_q, 3.0, 11.0);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
    for (int i = 0; i < 2; ++i) sm.update(good_q, 4.0, 12.0);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
    EXPECT_TRUE(sm.in_cooldown(12.0));
    sm.update(good_q, 5.0, 15.0);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);
}

// ---------- 位姿门控 ----------
TEST(PoseGate, OnlyGatesWhenLocked)
{
    PoseGateConfig cfg;
    cfg.max_position_jump_for_update_locked = 1.0;
    PoseGate gate(cfg);

    state_ikfom before, after;
    before.pos = Eigen::Vector3d(0, 0, 0);
    after.pos = Eigen::Vector3d(10, 0, 0);  // 巨大跳变

    // 非 LOCKED 状态一律放行
    EXPECT_TRUE(gate.allow(before, after, LocalizationTrackingState::TRACKING_UNLOCKED));
    EXPECT_TRUE(gate.allow(before, after, LocalizationTrackingState::TRACKING_TRACKING));
    EXPECT_TRUE(gate.allow(before, after, LocalizationTrackingState::TRACKING_LOST));
    // LOCKED 且超阈值 → 拒绝
    EXPECT_FALSE(gate.allow(before, after, LocalizationTrackingState::TRACKING_LOCKED));
}

TEST(PoseGate, ThresholdBoundary)
{
    PoseGateConfig cfg;
    cfg.max_position_jump_for_update_locked = 1.0;
    PoseGate gate(cfg);

    state_ikfom before, after;
    before.pos = Eigen::Vector3d(0, 0, 0);
    after.pos = Eigen::Vector3d(0.99, 0, 0);
    EXPECT_TRUE(gate.allow(before, after, LocalizationTrackingState::TRACKING_LOCKED));
    after.pos = Eigen::Vector3d(1.01, 0, 0);
    EXPECT_FALSE(gate.allow(before, after, LocalizationTrackingState::TRACKING_LOCKED));
}

TEST(PoseGate, AdaptiveGatingScalesThreshold)
{
    PoseGateConfig cfg;
    cfg.max_position_jump_for_update_locked = 1.0;
    cfg.adaptive_gating_en = true;
    cfg.adaptive_scale_max = 3.0;
    PoseGate gate(cfg);

    state_ikfom before, after;
    before.pos = Eigen::Vector3d(0, 0, 0);
    after.pos = Eigen::Vector3d(1.5, 0, 0);  // 超基础阈值

    // 关闭自适应时拒绝
    EXPECT_FALSE(gate.allow(before, after, LocalizationTrackingState::TRACKING_LOCKED, 0.0));
    // 残差比率 1.0 → 阈值放大 2 倍 → 放行
    EXPECT_TRUE(gate.allow(before, after, LocalizationTrackingState::TRACKING_LOCKED, 1.0));
    // 跳变 3.5 超过最大放大（3 倍 = 3.0m）→ 拒绝
    after.pos = Eigen::Vector3d(3.5, 0, 0);
    EXPECT_FALSE(gate.allow(before, after, LocalizationTrackingState::TRACKING_LOCKED, 100.0));
}

TEST(StateMachine, RecoveryResetsGoodStreak)
{
    auto cfg = default_cfg();
    cfg.lost_recovery_en = true;
    cfg.lost_recovery_streak = 5;
    cfg.good_match_streak_to_lock = 5;
    cfg.lost_recovery_cooldown_sec = 0.0;
    cfg.min_time_before_lock_sec = 2.0;
    auto sm = TrackingStateMachine(cfg);
    const auto good_q = make_quality(100, 0.05);
    const auto bad_q = make_quality(2, 5.0, false);

    for (int i = 0; i < 3; i++) sm.update(good_q, 0.1 * i);      // → TRACKING
    for (int i = 0; i < 5; i++) sm.update(bad_q, 1.0);            // → LOST
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOST);
    for (int i = 0; i < 5; i++) sm.update(good_q, 2.0 + 0.1 * i, 10.0 + i);  // 恢复
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);
    sm.update(good_q, 3.0, 20.0);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);
}

TEST(StateMachine, UnlockResetsGoodStreak)
{
    auto cfg = default_cfg();
    cfg.unlock_to_tracking_streak = 3;
    cfg.good_match_streak_to_lock = 5;
    cfg.min_time_before_lock_sec = 0.0;
    auto sm = TrackingStateMachine(cfg);
    const auto good_q = make_quality(100, 0.05);

    for (int i = 0; i < 3; i++) sm.update(good_q, 0.1 * i);
    ASSERT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);
    for (int i = 0; i < 4; i++) sm.update(good_q, 0.2 * i);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_TRACKING);
    sm.update(good_q, 1.0);
    EXPECT_EQ(sm.state(), LocalizationTrackingState::TRACKING_LOCKED);
}
