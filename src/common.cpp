#include <lio/common.h>

#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace lio {
namespace {
int mysign(double x) { return (x > 0) - (x < 0); }
const std::vector<uint32_t> power10 = makePower10();
}  // namespace

std::ostream& operator<<(std::ostream& os, const StampedPose& p) {
  os << p.time << " ";
  const std::streamsize oldPrecision = os.precision();
  os << std::fixed << std::setprecision(7)
     << p.r[0] << " " << p.r[1] << " " << p.r[2] << " ";
  os << std::setprecision(9)
     << p.q.x() << " " << p.q.y() << " " << p.q.z() << " " << p.q.w();
  os.precision(oldPrecision);
  return os;
}

std::string StampedPose::toString() const {
  std::stringstream ss;
  const std::string delimiter(" ");
  ss << time << delimiter << std::setprecision(10) << r.x() << delimiter << r.y() << delimiter << r.z()
     << delimiter << q.x() << delimiter << q.y() << delimiter << q.z()
     << delimiter << q.w();
  return ss.str();
}

Eigen::Vector3d IncrementMotion::ratio(const IncrementMotion& ref) const {
  Eigen::Vector3d rqv;
  rqv[0] = ref.pij.norm() * mysign(pij.dot(ref.pij)) / pij.norm();
  Eigen::AngleAxisd aar(ref.qij);
  Eigen::AngleAxisd aa(qij);
  rqv[1] = std::fabs(aar.angle() / aa.angle());
  rqv[2] = ref.vij.norm() * mysign(vij.dot(ref.vij)) / vij.norm();
  return rqv;
}

void IncrementMotion::multiply(const Eigen::Vector3d& coeffs) {
  pij *= coeffs[0];
  Eigen::AngleAxisd aa(qij);
  Eigen::Vector3d log = aa.axis() * aa.angle() * coeffs[1];
  Eigen::AngleAxisd saa(log.norm(), log.normalized());
  qij = saa;
  vij *= coeffs[2];
}

IncrementMotion StampedState::between(const StampedState& sj) const {
  IncrementMotion im;
  im.pij = q.conjugate() * (sj.r - r);
  im.qij = q.conjugate() * sj.q;
  im.vij = q.conjugate() * (sj.v - v);
  return im;
}

Eigen::Isometry3d StampedState::betweenPose(const StampedState& sj) const {
  Eigen::Isometry3d dT = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond dq = q.conjugate() * sj.q;
  dT.linear() = dq.toRotationMatrix();
  Eigen::Vector3d dr = sj.r - r;
  dT.translation() = q.conjugate() * dr;
  return dT;
}

std::string StampedState::toString() const {
  const std::string delimiter = " ";
  std::stringstream os;
  os << time << delimiter << std::fixed << std::setprecision(6) << r[0] << delimiter << r[1] << delimiter << r[2];
  os << delimiter << std::fixed << std::setprecision(9) << q.x() << delimiter << q.y() << delimiter << q.z() << delimiter << q.w();
  os << delimiter << std::fixed << std::setprecision(6) << v[0] << delimiter << v[1] << delimiter << v[2];
  os << delimiter << std::fixed << std::setprecision(6) << bg[0] << delimiter << bg[1] << delimiter << bg[2];
  os << delimiter << std::fixed << std::setprecision(6) << ba[0] << delimiter << ba[1] << delimiter << ba[2];
  os << delimiter << std::fixed << std::setprecision(6) << gW[0] << delimiter << gW[1] << delimiter << gW[2];
  return os.str();
}

void StampedState::retract(const IncrementMotion& im) {
  r += q * im.pij;
  v += q * im.vij;
  q = q * im.qij;
}

std::ostream& operator<<(std::ostream& os, const StampedState& s) {
  os << s.toString();
  return os;
}

std::istream& operator>>(std::istream& is, StampedState& s) {
  std::string tstr;
  double qx, qy, qz, qw;

  if (!(is >> tstr
           >> s.r.x() >> s.r.y() >> s.r.z()
           >> qx >> qy >> qz >> qw
           >> s.v.x() >> s.v.y() >> s.v.z()
           >> s.bg.x() >> s.bg.y() >> s.bg.z()
           >> s.ba.x() >> s.ba.y() >> s.ba.z()
           >> s.gW.x() >> s.gW.y() >> s.gW.z())) {
    return is;
  }

  const auto ptime = parseTime(tstr);
  s.time = Time(ptime.first, ptime.second);
  s.q = Eigen::Quaterniond(qw, qx, qy, qz);
  s.q.normalize();
  return is;
}

std::istream& operator>>(std::istream& is, StampedStateWithCov& s) {
  std::string time_str;
  if (!(is >> time_str)) {
    return is;
  }
  const auto ptime = parseTime(time_str);
  s.time = Time(ptime.first, ptime.second);

  double px, py, pz;
  double qx, qy, qz, qw;
  double vx, vy, vz;
  double bgx, bgy, bgz;
  double bax, bay, baz;
  double gwx, gwy, gwz;
  double otx, oty, otz;
  double orx, ory, orz, orw;
  if (!(is >> px >> py >> pz >> qx >> qy >> qz >> qw
        >> vx >> vy >> vz
        >> bgx >> bgy >> bgz
        >> bax >> bay >> baz
        >> gwx >> gwy >> gwz
        >> otx >> oty >> otz
        >> orx >> ory >> orz >> orw)) {
    return is;
  }

  s.r = Eigen::Vector3d(px, py, pz);
  s.q = Eigen::Quaterniond(qw, qx, qy, qz);
  s.q.normalize();
  s.v = Eigen::Vector3d(vx, vy, vz);
  s.bg = Eigen::Vector3d(bgx, bgy, bgz);
  s.ba = Eigen::Vector3d(bax, bay, baz);
  s.gW = Eigen::Vector3d(gwx, gwy, gwz);
  s.offset_T_L_I = Eigen::Vector3d(otx, oty, otz);
  s.offset_R_L_I = Eigen::Quaterniond(orw, orx, ory, orz);
  s.offset_R_L_I.normalize();

  for (int i = 0; i < StampedStateWithCov::kCovDim; ++i) {
    if (!(is >> s.cov_diag(i))) {
      return is;
    }
  }
  return is;
}

StampedState applyIncrement(const StampedState& si, const IncrementMotion& inc) {
  StampedState s = si;
  s.r = si.r + si.q * inc.pij;
  s.q = (si.q * inc.qij).normalized();
  s.v = si.v + si.q * inc.vij;
  return s;
}

StampedState interp(const StampedState& a, const StampedState& b, double alpha) {
  alpha = std::clamp(alpha, 0.0, 1.0);
  IncrementMotion im = a.between(b);
  im.multiply(Eigen::Vector3d(alpha, alpha, alpha));
  StampedState out = applyIncrement(a, im);
  out.time = a.time + Duration::fromSec((b.time - a.time).toSec() * alpha);
  out.bg = (1 - alpha) * a.bg + alpha * b.bg;
  out.ba = (1 - alpha) * a.ba + alpha * b.ba;
  out.gW = (1 - alpha) * a.gW + alpha * b.gW;
  return out;
}

StampedState interpAtMonotonic(const std::vector<StampedState>& traj, const Time& t, size_t& idx) {
  if (traj.empty()) return {};
  while (idx + 1 < traj.size() && traj[idx + 1].time < t) idx++;
  if (idx + 1 >= traj.size()) return traj.back();
  if (t <= traj.front().time) return traj.front();

  const auto& a = traj[idx];
  const auto& b = traj[idx + 1];
  const double dt = (b.time - a.time).toSec();
  const double alpha = (dt > 0) ? (t - a.time).toSec() / dt : 0.0;
  return interp(a, b, alpha);
}

size_t loadStates(const std::string& stateFile, StampedStateVector* states) {
  std::ifstream inFile(stateFile);
  if (!inFile.is_open()) {
    printf("Open state file %s failed.\n", stateFile.c_str());
    return 0;
  }

  size_t lineCount = 0;
  {
    std::string tmp;
    while (std::getline(inFile, tmp)) {
      if (!tmp.empty() && tmp[0] != '#') lineCount++;
    }
    inFile.clear();
    inFile.seekg(0);
  }
  states->reserve(lineCount);

  std::string lineStr;
  while (std::getline(inFile, lineStr)) {
    if (lineStr.empty() || lineStr[0] == '#') {
      continue;
    }
    std::stringstream ss(lineStr);
    StampedState s;
    ss >> s;
    states->push_back(s);
  }

  std::cout << "#Poses: " << states->size() << std::endl;
  if (!states->empty()) {
    std::cout << "First state: " << states->front() << std::endl;
    std::cout << "Last state: " << states->back() << std::endl;
  }
  return states->size();
}

size_t saveStates(const StampedStateVector& states, const std::string& stateFile) {
  std::ofstream os(stateFile, std::ofstream::out);
  if (!os.is_open()) {
    std::cout << "Failed to open output state file " << stateFile << "." << std::endl;
    return 0;
  }
  for (const auto& s : states) {
    os << s.toString() << "\n";
  }
  return states.size();
}

std::ostream& operator<<(std::ostream& os, const ImuData& d) {
  os << d.time << ", g: " << d.g.transpose() << ", a: " << d.a.transpose();
  return os;
}

std::istream& operator>>(std::istream& is, ImuData& d) {
  std::string tstr;
  if (!(is >> tstr >> d.g.x() >> d.g.y() >> d.g.z() >> d.a.x() >> d.a.y() >> d.a.z())) {
    return is;
  }
  const auto ptime = parseTime(tstr);
  d.time = Time(ptime.first, ptime.second);
  return is;
}

std::pair<uint32_t, uint32_t> parseTime(const std::string& time) {
  size_t pos = time.find(".");
  size_t pos1;
  size_t pad = 0;
  size_t pow = 1;
  if (pos == std::string::npos) {
    pos = time.length() - 9;
    pos1 = pos;
  } else {
    pos1 = pos + 1;
    pad = 9 - (time.length() - pos1);
    pow = power10[pad];
  }
  std::string trunk = time.substr(0, pos);
  uint32_t sec = 0;
  uint32_t nsec = 0;
  std::istringstream ss1(trunk);
  ss1 >> sec;

  std::string residuals = time.substr(pos1);
  std::istringstream ss2(residuals);
  ss2 >> nsec;
  nsec *= pow;
  return std::make_pair(sec, nsec);
}

void testParseTime() {
  auto p1 = parseTime("100000.100001");
  Time t1(p1.first, p1.second);
  auto p2 = parseTime("100000100001000");
  Time t2(p2.first, p2.second);
  std::cout << "0 == " << t1 - t2 << std::endl;
}

double approxAreaFromAabbAndNormal(const Eigen::Vector3d& mn,
                                   const Eigen::Vector3d& mx,
                                   const Eigen::Vector3d& n) {
  Eigen::Vector3d t = (std::abs(n.x()) < 0.9) ? Eigen::Vector3d::UnitX()
                                              : Eigen::Vector3d::UnitY();
  Eigen::Vector3d u = (t - t.dot(n) * n).normalized();
  Eigen::Vector3d v = n.cross(u);

  Eigen::Vector3d c[8] = {
      {mn.x(), mn.y(), mn.z()}, {mx.x(), mn.y(), mn.z()},
      {mn.x(), mx.y(), mn.z()}, {mx.x(), mx.y(), mn.z()},
      {mn.x(), mn.y(), mx.z()}, {mx.x(), mn.y(), mx.z()},
      {mn.x(), mx.y(), mx.z()}, {mx.x(), mx.y(), mx.z()}};

  double umin = std::numeric_limits<double>::infinity();
  double vmin = umin;
  double umax = -umin;
  double vmax = -umin;

  for (const auto& corner : c) {
    const double cu = corner.dot(u);
    const double cv = corner.dot(v);
    umin = std::min(umin, cu);
    umax = std::max(umax, cu);
    vmin = std::min(vmin, cv);
    vmax = std::max(vmax, cv);
  }
  return std::max(0.0, umax - umin) * std::max(0.0, vmax - vmin);
}

}  // namespace lio
