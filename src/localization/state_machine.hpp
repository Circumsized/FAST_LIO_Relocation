#ifndef FAST_LIO_TRACKING_STATE_MACHINE_HPP
#define FAST_LIO_TRACKING_STATE_MACHINE_HPP

namespace fast_lio
{

enum class LocalizationTrackingState
{
    TRACKING_UNLOCKED = 0,
    TRACKING_TRACKING = 1,
    TRACKING_LOCKED = 2,
    TRACKING_LOST = 3,
};

inline const char *tracking_state_name(LocalizationTrackingState s)
{
    switch (s)
    {
    case LocalizationTrackingState::TRACKING_UNLOCKED: return "UNLOCKED";
    case LocalizationTrackingState::TRACKING_TRACKING: return "TRACKING";
    case LocalizationTrackingState::TRACKING_LOCKED: return "LOCKED";
    case LocalizationTrackingState::TRACKING_LOST: return "LOST";
    }
    return "UNKNOWN";
}

/* 外部重定位（如 NDT）恢复的结果状态。
   用于让节点区分“未处于 LOST / 预算或冷却受限 / 成功”，避免与内部软恢复混淆。 */
enum class ExternalRecoveryStatus
{
    kSuccess = 0,
    kNotLost = 1,
    kDisabledOrExhausted = 2,
};

struct MatchQuality
{
    bool correspondence_valid = false;
    bool gate_rejected = false;
    int effct_feat_num = 0;
    double res_mean_last = 1e9;
};

struct TrackingStateMachineConfig
{
    int min_effective_points_for_tracking = 15;
    double max_residual_for_tracking = 0.40;
    int min_effective_points_for_good = 30;
    double max_residual_for_good = 0.20;
    int unlock_to_tracking_streak = 3;
    int good_match_streak_to_lock = 5;
    int bad_match_streak_to_lost = 5;
    double min_time_before_lock_sec = 2.0;
    bool lost_recovery_en = false;
    int lost_recovery_streak = 10;
    int lost_recovery_max_attempts = 3;
    double lost_recovery_cooldown_sec = 5.0;
};

inline bool is_match_acceptable(const MatchQuality &q, const TrackingStateMachineConfig &cfg)
{
    if (!q.correspondence_valid) return false;
    if (q.effct_feat_num < cfg.min_effective_points_for_tracking) return false;
    if (q.res_mean_last > cfg.max_residual_for_tracking) return false;
    return true;
}

inline bool is_match_good(const MatchQuality &q, const TrackingStateMachineConfig &cfg)
{
    if (!q.correspondence_valid) return false;
    if (q.effct_feat_num < cfg.min_effective_points_for_good) return false;
    if (q.res_mean_last > cfg.max_residual_for_good) return false;
    return true;
}

class TrackingStateMachine
{
public:
    explicit TrackingStateMachine(const TrackingStateMachineConfig &cfg) : cfg_(cfg)
    {
        if (cfg_.min_effective_points_for_tracking < 1) cfg_.min_effective_points_for_tracking = 1;
        if (cfg_.min_effective_points_for_good < cfg_.min_effective_points_for_tracking)
            cfg_.min_effective_points_for_good = cfg_.min_effective_points_for_tracking;
        if (cfg_.max_residual_for_tracking < 0.0) cfg_.max_residual_for_tracking = 0.0;
        if (cfg_.max_residual_for_good < 0.0) cfg_.max_residual_for_good = 0.0;
        if (cfg_.max_residual_for_good > cfg_.max_residual_for_tracking)
            cfg_.max_residual_for_good = cfg_.max_residual_for_tracking;
        if (cfg_.unlock_to_tracking_streak < 1) cfg_.unlock_to_tracking_streak = 1;
        if (cfg_.good_match_streak_to_lock < 1) cfg_.good_match_streak_to_lock = 1;
        if (cfg_.bad_match_streak_to_lost < 1) cfg_.bad_match_streak_to_lost = 1;
        if (cfg_.lost_recovery_streak < 1) cfg_.lost_recovery_streak = 1;
        if (cfg_.lost_recovery_max_attempts < 0) cfg_.lost_recovery_max_attempts = 0;
        if (cfg_.min_time_before_lock_sec < 0.0) cfg_.min_time_before_lock_sec = 0.0;
        if (cfg_.lost_recovery_cooldown_sec < 0.0) cfg_.lost_recovery_cooldown_sec = 0.0;
    }

    LocalizationTrackingState state() const { return state_; }
    int acceptable_streak() const { return acceptable_streak_; }
    int good_streak() const { return good_streak_; }
    int bad_streak() const { return bad_streak_; }
    int recovery_attempts() const { return recovery_attempts_; }
    bool in_cooldown(double now_sec) const
    {
        return cfg_.lost_recovery_en && now_sec < recovery_cooldown_until_;
    }
    bool is_recovery_enabled() const { return cfg_.lost_recovery_en; }
    bool just_recovered_from_lost() const { return just_recovered_flag_; }
    void clear_just_recovered() { just_recovered_flag_ = false; }

    /* 外部重定位（如 NDT）成功时调用：把状态从 LOST 拉回 TRACKING。
       与内部软恢复共用同一恢复预算（recovery_attempts_）与冷却，
       并受 lost_recovery_en 总开关约束，避免绕过预算或破坏“关闭时永久冻结”。
       不设置 just_recovered_flag_（update() 首行会清零）。 */
    ExternalRecoveryStatus on_external_relocalization_success(double now_sec)
    {
        if (!cfg_.lost_recovery_en)
        {
            return ExternalRecoveryStatus::kDisabledOrExhausted;
        }
        if (state_ != LocalizationTrackingState::TRACKING_LOST)
        {
            return ExternalRecoveryStatus::kNotLost;
        }
        if (recovery_attempts_ >= cfg_.lost_recovery_max_attempts)
        {
            return ExternalRecoveryStatus::kDisabledOrExhausted;
        }
        if (in_cooldown(now_sec))
        {
            return ExternalRecoveryStatus::kDisabledOrExhausted;
        }

        ++recovery_attempts_;
        recovery_cooldown_until_ = now_sec + cfg_.lost_recovery_cooldown_sec;
        acceptable_streak_ = 0;
        good_streak_ = 0;
        bad_streak_ = 0;
        current_match_good_ = false;
        has_last_tracking_ = true;
        state_ = LocalizationTrackingState::TRACKING_TRACKING;
        return ExternalRecoveryStatus::kSuccess;
    }

    /* 外部重定位尝试失败时调用：同样消耗一次恢复预算并进入冷却，
       防止在触发间隔内无限重试。返回 false 表示预算已耗尽或处于冷却（未消耗）。 */
    bool note_external_recovery_attempt(double now_sec)
    {
        if (!cfg_.lost_recovery_en)
        {
            return false;
        }
        if (state_ != LocalizationTrackingState::TRACKING_LOST)
        {
            return false;
        }
        if (recovery_attempts_ >= cfg_.lost_recovery_max_attempts)
        {
            return false;
        }
        if (in_cooldown(now_sec))
        {
            return false;
        }

        ++recovery_attempts_;
        recovery_cooldown_until_ = now_sec + cfg_.lost_recovery_cooldown_sec;
        return true;
    }

    void update(const MatchQuality &q, double time_since_start, double now_sec = 0.0)
    {
        just_recovered_flag_ = false;

        const bool match_acceptable = is_match_acceptable(q, cfg_);
        const bool match_good = is_match_good(q, cfg_);
        current_match_good_ = match_good;

        if (match_acceptable)
        {
            ++acceptable_streak_;
            bad_streak_ = 0;
        }
        else
        {
            acceptable_streak_ = 0;
            ++bad_streak_;
        }
        if (match_good)
        {
            ++good_streak_;
        }
        else
        {
            good_streak_ = 0;
        }

        switch (state_)
        {
        case LocalizationTrackingState::TRACKING_UNLOCKED:
            if (match_acceptable && acceptable_streak_ >= cfg_.unlock_to_tracking_streak)
            {
                transition_to(LocalizationTrackingState::TRACKING_TRACKING);
                on_enter_tracking();
            }
            return;

        case LocalizationTrackingState::TRACKING_TRACKING:
            if (time_since_start >= cfg_.min_time_before_lock_sec &&
                match_good && good_streak_ >= cfg_.good_match_streak_to_lock)
            {
                transition_to(LocalizationTrackingState::TRACKING_LOCKED);
                on_enter_locked();
                return;
            }
            if (!match_acceptable && bad_streak_ >= cfg_.bad_match_streak_to_lost)
            {
                transition_to(LocalizationTrackingState::TRACKING_LOST);
                return;
            }
            return;

        case LocalizationTrackingState::TRACKING_LOCKED:
            if (!match_good)
            {
                transition_to(LocalizationTrackingState::TRACKING_TRACKING);
                good_streak_ = 0;
            }
            return;

        case LocalizationTrackingState::TRACKING_LOST:
            if (cfg_.lost_recovery_en &&
                match_acceptable &&
                acceptable_streak_ >= cfg_.lost_recovery_streak &&
                recovery_attempts_ < cfg_.lost_recovery_max_attempts &&
                !in_cooldown(now_sec))
            {
                ++recovery_attempts_;
                recovery_cooldown_until_ = now_sec + cfg_.lost_recovery_cooldown_sec;
                acceptable_streak_ = 0;
                bad_streak_ = 0;
                transition_to(LocalizationTrackingState::TRACKING_TRACKING);
                on_enter_tracking();
                just_recovered_flag_ = true;
            }
            return;
        }
    }

    bool current_match_good() const { return current_match_good_; }
    void notify_acceptable_snapshot() { has_last_tracking_ = true; }
    void notify_good_snapshot() { has_last_locked_ = true; }
    bool has_last_tracking() const { return has_last_tracking_; }
    bool has_last_locked() const { return has_last_locked_; }

    void reset_for_localization_start()
    {
        state_ = LocalizationTrackingState::TRACKING_UNLOCKED;
        acceptable_streak_ = 0;
        good_streak_ = 0;
        bad_streak_ = 0;
        recovery_attempts_ = 0;
        recovery_cooldown_until_ = 0.0;
        has_last_tracking_ = false;
        has_last_locked_ = false;
        current_match_good_ = false;
        just_recovered_flag_ = false;
    }

private:
    void transition_to(LocalizationTrackingState s) { state_ = s; }
    void on_enter_tracking()
    {
        has_last_tracking_ = true;
        good_streak_ = 0;
    }
    void on_enter_locked() { has_last_locked_ = true; }

    TrackingStateMachineConfig cfg_;
    LocalizationTrackingState state_ = LocalizationTrackingState::TRACKING_UNLOCKED;
    int acceptable_streak_ = 0;
    int good_streak_ = 0;
    int bad_streak_ = 0;
    int recovery_attempts_ = 0;
    double recovery_cooldown_until_ = 0.0;
    bool current_match_good_ = false;
    bool has_last_tracking_ = false;
    bool has_last_locked_ = false;
    bool just_recovered_flag_ = false;
};

} // namespace fast_lio

#endif // FAST_LIO_TRACKING_STATE_MACHINE_HPP
