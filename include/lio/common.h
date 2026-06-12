#pragma once

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <lio/time.h>

namespace lio {

enum class CostType : std::uint8_t {
  PriorPose = 0,
  RelativePose,
  ControlPointPosition,
  ImuErrorWithGravity,
  PriorSpeedAndBias,
};

inline std::vector<uint32_t> makePower10() {
  std::vector<uint32_t> pow10(10);
  pow10[0] = 1;
  for (int i = 1; i < static_cast<int>(pow10.size()); ++i) {
    pow10[i] = pow10[i - 1] * 10;
  }
  return pow10;
}

std::pair<uint32_t, uint32_t> parseTime(const std::string& time);
void testParseTime();

struct StampedPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Time time;
  Eigen::Vector3d r;
  Eigen::Quaterniond q;

  StampedPose() = default;
  explicit StampedPose(const Time& time_in) : time(time_in) {}
  StampedPose(const Time& time_in, const Eigen::Vector3d& r_in, const Eigen::Quaterniond& q_in)
      : time(time_in), r(r_in), q(q_in) {}

  std::pair<Eigen::Vector3d, Eigen::Quaterniond> pose() const { return std::make_pair(r, q); }
  void setPose(const std::pair<Eigen::Vector3d, Eigen::Quaterniond>& pose_in) {
    r = pose_in.first;
    q = pose_in.second;
  }

  Eigen::Isometry3d betweenPose(const StampedPose& y) const {
    const Eigen::Quaterniond q_rel = q.conjugate() * y.q;
    const Eigen::Vector3d t_rel = q.conjugate() * (y.r - r);
    Eigen::Isometry3d T_rel = Eigen::Isometry3d::Identity();
    T_rel.linear() = q_rel.toRotationMatrix();
    T_rel.translation() = t_rel;
    return T_rel;
  }

  Eigen::Isometry3f isof() const {
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.linear() = q.toRotationMatrix().cast<float>();
    T.translation() = r.cast<float>();
    return T;
  }

  std::string toString() const;

  friend bool operator<(const StampedPose& c1, const StampedPose& c2);
  friend std::ostream& operator<<(std::ostream& os, const StampedPose& p);
};

inline bool operator<(const StampedPose& c1, const StampedPose& c2) {
  return c1.time < c2.time;
}

struct IncrementMotion {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d pij;
  Eigen::Quaterniond qij;
  Eigen::Vector3d vij;

  Eigen::Vector3d ratio(const IncrementMotion& ref) const;
  void multiply(const Eigen::Vector3d& coeffs);
};

struct StampedState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Time time;
  Eigen::Vector3d r;
  Eigen::Quaterniond q;
  Eigen::Vector3d v;
  Eigen::Vector3d bg;
  Eigen::Vector3d ba;
  Eigen::Vector3d gW;

  StampedState() = default;
  StampedState(const Time& t, const Eigen::Vector3d& r_in, const Eigen::Quaterniond& q_in)
      : time(t), r(r_in), q(q_in) {
    v.setZero();
    bg.setZero();
    ba.setZero();
    gW = Eigen::Vector3d(0, 0, -9.81);
  }

  StampedState leftTransform(const Eigen::Quaterniond& q_left) const {
    StampedState result(*this);
    result.r = q_left * result.r;
    result.q = q_left * result.q;
    result.v = q_left * result.v;
    result.gW = q_left * result.gW;
    return result;
  }

  void updatePose(const Eigen::Matrix3d& R, const Eigen::Vector3d& p) {
    r = p;
    q = Eigen::Quaterniond(R);
  }

  void updatePose(const Eigen::Quaterniond& q_in, const Eigen::Vector3d& p) {
    r = p;
    q = q_in;
  }

  void setGravity(const Eigen::Vector3d& gW_in) { gW = gW_in; }
  Eigen::Quaterniond quat() const { return q; }
  Eigen::Vector3d trans() const { return r; }

  Eigen::Isometry3d toIsometry3d() const {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = r;
    T.linear() = q.toRotationMatrix();
    return T;
  }

  Eigen::Isometry3f toIsometry3f() const {
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.translation() = r.cast<float>();
    T.linear() = q.toRotationMatrix().cast<float>();
    return T;
  }

  IncrementMotion between(const StampedState& sj) const;
  Eigen::Isometry3d betweenPose(const StampedState& sj) const;
  void retract(const IncrementMotion& im);

  bool operator<(const StampedState& rhs) const { return time < rhs.time; }

  std::string toString() const;
  friend std::ostream& operator<<(std::ostream& os, const StampedState& s);
  friend std::istream& operator>>(std::istream& is, StampedState& s);
};

struct StampedStateWithCov : public StampedState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr int kCovDim = 23;
  Eigen::Vector3d offset_T_L_I;
  Eigen::Quaterniond offset_R_L_I;
  Eigen::Matrix<double, kCovDim, 1> cov_diag;

  StampedStateWithCov() {
    offset_T_L_I.setZero();
    offset_R_L_I.setIdentity();
    cov_diag.setZero();
  }

  friend std::istream& operator>>(std::istream& is, StampedStateWithCov& s);
};

inline double median_of(std::vector<double> v) {
  const size_t n = v.size();
  auto mid = v.begin() + n / 2;
  std::nth_element(v.begin(), mid, v.end());
  double m = *mid;
  if ((n & 1) == 0) {
    auto midm1 = v.begin() + (n / 2 - 1);
    std::nth_element(v.begin(), midm1, v.end());
    m = 0.5 * (m + *midm1);
  }
  return m;
}

inline double quantile_of(std::vector<double>& v, double q) {
  if (v.empty()) return 0.0;
  q = std::min(std::max(q, 0.0), 1.0);
  const size_t k = static_cast<size_t>(q * (v.size() - 1));
  std::nth_element(v.begin(), v.begin() + k, v.end());
  return v[k];
}

struct ImuData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Time time;
  Eigen::Vector3d g;
  Eigen::Vector3d a;
  friend std::ostream& operator<<(std::ostream& os, const ImuData& d);
  friend std::istream& operator>>(std::istream& is, ImuData& d);
};

using StampedPoseVector = std::vector<StampedPose, Eigen::aligned_allocator<StampedPose>>;
using StampedStateVector = std::vector<StampedState, Eigen::aligned_allocator<StampedState>>;
using ImuDataVector = std::vector<ImuData, Eigen::aligned_allocator<ImuData>>;

size_t loadStates(const std::string& stateFile, StampedStateVector* states);
size_t saveStates(const StampedStateVector& states, const std::string& stateFile);

double approxAreaFromAabbAndNormal(const Eigen::Vector3d& mn,
                                   const Eigen::Vector3d& mx,
                                   const Eigen::Vector3d& n);

StampedState applyIncrement(const StampedState& si, const IncrementMotion& inc);
StampedState interp(const StampedState& a, const StampedState& b, double alpha);
StampedState interpAtMonotonic(const std::vector<StampedState>& traj, const Time& t, size_t& idx);

}  // namespace lio

namespace cba {
using ::lio::CostType;
using ::lio::Duration;
using ::lio::ImuData;
using ::lio::ImuDataVector;
using ::lio::IncrementMotion;
using ::lio::StampedPose;
using ::lio::StampedPoseVector;
using ::lio::StampedState;
using ::lio::StampedStateVector;
using ::lio::Time;
using ::lio::applyIncrement;
using ::lio::approxAreaFromAabbAndNormal;
using ::lio::interp;
using ::lio::interpAtMonotonic;
using ::lio::loadStates;
using ::lio::makePower10;
using ::lio::median_of;
using ::lio::parseTime;
using ::lio::quantile_of;
using ::lio::saveStates;
using ::lio::testParseTime;
}  // namespace cba
