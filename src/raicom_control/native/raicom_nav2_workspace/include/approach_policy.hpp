#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace raicom_nav2_workspace {
constexpr double pi = 3.14159265358979323846;
inline double wrap(double v) { return std::remainder(v, 2.0 * pi); }
struct Limits {
  double xy{0.02}, yaw{pi / 18.}, far{0.25}, near{0.12}, fine{0.03};
  double near_distance{0.25}, fine_distance{0.06}, validated_cap{0.25};
  double turn{0.35}, accel{0.5}, angular_accel{0.7};
  void validate() const {
    for (double v : {xy,yaw,far,near,fine,fine_distance,validated_cap,turn,accel,angular_accel})
      if (!std::isfinite(v) || v <= 0.) throw std::invalid_argument("invalid approach limits");
    if (std::abs(xy-.02)>1e-9 || std::abs(yaw-pi/18)>1e-9 || far>2. || near>2. || fine>2. ||
        !std::isfinite(near_distance) || near_distance<0. || validated_cap>2.0 || turn>2.0)
      throw std::invalid_argument("unsupported approach envelope");
  }
  double speed(double distance) const {
    if (distance<=xy) return 0.;
    const double stage = distance<=std::min(fine_distance,near_distance) ? fine : distance<=near_distance ? near : far;
    // Keep a bounded fine command outside acceptance; do not asymptotically
    // decay below chassis deadband while still outside the position window.
    return std::min({stage,validated_cap,std::max(fine,std::sqrt(2.*accel*(distance-xy)))});
  }
  double braking_horizon(double linear,double angular) const {
    // Controller (10 Hz), smoother and guard (20 Hz) reaction budget plus
    // complete stopping time, instead of truncating high-speed braking at 1 s.
    return .20+std::max(std::abs(linear)/accel,std::abs(angular)/angular_accel);
  }
};
enum class Phase { complete, path_turn, tracking, fine_position, final_yaw };
// Keep one goal active while the smoother, chassis and localization settle.
// Leaving either pose tolerance immediately invalidates the entire hold.
struct SettledArrival {
  double since{-1.};
  void reset() {since=-1.;}
  bool reached(double distance,double angle,double v,double w,double now) {
    if (!std::isfinite(distance) || !std::isfinite(angle) || !std::isfinite(v) ||
        !std::isfinite(w) || !std::isfinite(now) || distance>.02 ||
        std::abs(wrap(angle))>pi/18. || std::abs(v)>.01 || std::abs(w)>.02) {
      reset(); return false;
    }
    if (since<0. || now<since) since=now;
    return now-since>=.5;
  }
};
// A blocked braking prediction first requests zero within the same action.
// A current body collision is never eligible. The deadline does not slide
// forward as velocity falls, and a replan cannot restart this allowance.
struct BrakingHold {
  double until{-1.};
  void reset() {until=-1.;}
  bool wait(double now,double horizon) {
    if (!std::isfinite(now) || !std::isfinite(horizon) || horizon<0.) return false;
    if (until<0.) until=now+std::min(3.5,std::max(.5,horizon+.3));
    return now<until;
  }
};
inline Phase phase(double distance, double final_error, double path_error, const Limits& l) {
  if (distance<=l.xy) return std::abs(wrap(final_error))<=l.yaw ? Phase::complete : Phase::final_yaw;
  if (std::abs(wrap(path_error))>.15) return Phase::path_turn;
  return distance<=l.near_distance ? Phase::fine_position : Phase::tracking;
}
// Initial alignment is strict; small tracking corrections must not repeatedly
// switch between translation and a full stop. Replans of the same goal retain it.
inline Phase stable_phase(double distance, double final_error, double path_error,
    const Limits &l, bool &aligning) {
  if (distance<=l.xy) {
    aligning=true;
    return std::abs(wrap(final_error))<=l.yaw ? Phase::complete : Phase::final_yaw;
  }
  const double error=std::abs(wrap(path_error));
  if (aligning && error<=.08) aligning=false;
  if (!aligning && error>.45) aligning=true;
  if (aligning) return Phase::path_turn;
  return distance<=l.near_distance ? Phase::fine_position : Phase::tracking;
}
// Keep the full footprint/braking check at each candidate speed. A clear slower
// command is preferable to abort/recovery; no valid nonzero command still fails.
template<class Check> double safe_scale(double v, double w, Check clear) {
  for (double scale : {1., .75, .5, .25, .125, .0625}) {
    if (scale<1. && std::abs(v*scale)<.01 && std::abs(w*scale)<.03) break;
    if (clear(v*scale,w*scale)) return scale;
  }
  return 0.;
}
inline double turn_rate(double error, double tolerance, const Limits& l) {
  const double remaining = std::max(0., std::abs(wrap(error))-tolerance);
  if (remaining==0.) return 0.;
  return std::copysign(std::min(l.turn, std::max(.06,std::sqrt(2.*l.angular_accel*remaining))), wrap(error));
}
}  // namespace raicom_nav2_workspace
