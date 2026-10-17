#pragma once
#include <cmath>
#include <cstdint>
#include <limits>

namespace mobile_openarm_shared_control
{
struct PreferenceSnapshot
{
  enum Phase {AUTONOMOUS, ASSIST, TRANSITION};
  Phase phase = AUTONOMOUS;
  double linear = 0.0;
  double angular = 0.0;

  double cost(double v, double w, double sv, double sw) const
  {
    return phase == AUTONOMOUS ? 0.0 : sv * std::abs(linear - v) + sw * std::abs(angular - w);
  }
};

// All time arguments are monotonic seconds, independent of /clock.
class Preference
{
public:
  Preference(double timeout, double release_delay)
  : timeout_(timeout), release_delay_(release_delay) {}

  void receive(uint64_t epoch, bool permitted, bool active, double v, double w, double now)
  {
    if (epoch != epoch_ || now - received_ > timeout_) {clear();}
    epoch_ = epoch;
    received_ = now;
    if (!permitted || !std::isfinite(v) || !std::isfinite(w)) {
      clear();
      return;
    }
    if (active) {
      preference_ = {PreferenceSnapshot::ASSIST, v, w};
    } else if (preference_.phase == PreferenceSnapshot::ASSIST) {
      // Do not replay a command chosen before a long rotation-shim/recovery interval.
      if (now - selected_at_ <= timeout_ && release_delay_ > 0.0) {
        preference_ = {PreferenceSnapshot::TRANSITION, selected_linear_, 0.0};
        release_until_ = now + release_delay_;
      } else {
        clear();
      }
    }
  }

  PreferenceSnapshot snapshot(double now)
  {
    if (now - received_ > timeout_ ||
      (preference_.phase == PreferenceSnapshot::TRANSITION && now >= release_until_))
    {
      clear();
    }
    return preference_;
  }

  void selected(double v, double now)
  {
    selected_linear_ = v;
    selected_at_ = now;
  }

private:
  void clear()
  {
    preference_ = {};
    selected_at_ = -std::numeric_limits<double>::infinity();
    release_until_ = 0.0;
  }
  double timeout_, release_delay_;
  uint64_t epoch_ = 0;
  double received_ = -std::numeric_limits<double>::infinity();
  double selected_at_ = -std::numeric_limits<double>::infinity();
  double selected_linear_ = 0.0, release_until_ = 0.0;
  PreferenceSnapshot preference_;
};
}  // namespace mobile_openarm_shared_control
