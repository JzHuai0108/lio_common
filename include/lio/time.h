#pragma once

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <ostream>
#include <type_traits>
#include <utility>

namespace lio {
namespace detail {
template <typename T, typename = void>
struct HasToNSec : std::false_type {};

template <typename T>
struct HasToNSec<T, std::void_t<decltype(std::declval<const T&>().toNSec())>>
    : std::true_type {};

template <typename T, typename = void>
struct HasFromNSec : std::false_type {};

template <typename T>
struct HasFromNSec<T, std::void_t<decltype(std::declval<T&>().fromNSec(std::declval<uint64_t>()))>>
    : std::true_type {};
}  // namespace detail

struct Duration {
  int64_t ns = 0;

  Duration() = default;
  explicit constexpr Duration(int64_t nanoseconds) : ns(nanoseconds) {}
  explicit Duration(double seconds) : ns(static_cast<int64_t>(std::llround(seconds * 1e9))) {}

  template <typename DurationLike,
            typename = std::enable_if_t<detail::HasToNSec<DurationLike>::value>>
  Duration(const DurationLike& d) : ns(static_cast<int64_t>(d.toNSec())) {}

  Duration(int64_t sec, int64_t nsec) {
    sec += nsec / 1000000000LL;
    nsec = nsec % 1000000000LL;
    if (nsec < 0) {
      nsec += 1000000000LL;
      sec -= 1;
    }
    ns = sec * 1000000000LL + nsec;
  }

  int64_t sec() const { return ns / 1000000000LL; }
  int64_t nsec() const {
    int64_t r = ns % 1000000000LL;
    if (r < 0) r += 1000000000LL;
    return r;
  }

  static constexpr Duration fromNSec(int64_t nanoseconds) { return Duration(nanoseconds); }
  static Duration fromSec(double seconds) { return Duration(seconds); }

  int64_t toNSec() const { return ns; }
  double toSec() const { return static_cast<double>(ns) * 1e-9; }

  template <typename DurationLike,
            typename = std::enable_if_t<detail::HasFromNSec<DurationLike>::value>>
  operator DurationLike() const {
    DurationLike out;
    out.fromNSec(static_cast<int64_t>(ns));
    return out;
  }

  friend constexpr bool operator<(Duration a, Duration b) { return a.ns < b.ns; }
  friend constexpr bool operator>(Duration a, Duration b) { return a.ns > b.ns; }
  friend constexpr bool operator<=(Duration a, Duration b) { return a.ns <= b.ns; }
  friend constexpr bool operator>=(Duration a, Duration b) { return a.ns >= b.ns; }
  friend constexpr bool operator==(Duration a, Duration b) { return a.ns == b.ns; }
  friend constexpr bool operator!=(Duration a, Duration b) { return a.ns != b.ns; }
};

struct Time {
  int64_t ns = 0;

  Time() = default;
  explicit constexpr Time(int64_t nanoseconds) : ns(nanoseconds) {}
  explicit Time(double seconds) {
    const int64_t sec = static_cast<int64_t>(seconds);
    ns = sec * 1000000000LL +
         static_cast<int64_t>(std::llround((seconds - static_cast<double>(sec)) * 1000000000.0));
  }

  template <typename TimeLike,
            typename = std::enable_if_t<detail::HasToNSec<TimeLike>::value>>
  Time(const TimeLike& t) : ns(static_cast<int64_t>(t.toNSec())) {}

  Time(int64_t sec, int64_t nsec) {
    sec += nsec / 1000000000LL;
    nsec = nsec % 1000000000LL;
    if (nsec < 0) {
      nsec += 1000000000LL;
      sec -= 1;
    }
    ns = sec * 1000000000LL + nsec;
  }

  template <typename TimeLike,
            typename = std::enable_if_t<detail::HasToNSec<TimeLike>::value>>
  Time& operator=(const TimeLike& t) {
    ns = static_cast<int64_t>(t.toNSec());
    return *this;
  }

  template <typename TimeLike,
            typename = std::enable_if_t<detail::HasFromNSec<TimeLike>::value>>
  operator TimeLike() const {
    TimeLike out;
    out.fromNSec(static_cast<uint64_t>(ns));
    return out;
  }

  static constexpr Time fromNSec(int64_t nanoseconds) { return Time(nanoseconds); }
  static Time fromSec(double seconds) { return Time(seconds); }

  int64_t toNSec() const { return ns; }
  double toSec() const { return static_cast<double>(ns) * 1e-9; }

  int64_t sec() const { return ns / 1000000000LL; }
  int64_t nsec() const {
    int64_t r = ns % 1000000000LL;
    if (r < 0) r += 1000000000LL;
    return r;
  }

  friend constexpr bool operator<(Time a, Time b) { return a.ns < b.ns; }
  friend constexpr bool operator>(Time a, Time b) { return a.ns > b.ns; }
  friend constexpr bool operator<=(Time a, Time b) { return a.ns <= b.ns; }
  friend constexpr bool operator>=(Time a, Time b) { return a.ns >= b.ns; }
  friend constexpr bool operator==(Time a, Time b) { return a.ns == b.ns; }
  friend constexpr bool operator!=(Time a, Time b) { return a.ns != b.ns; }
};

inline std::ostream& operator<<(std::ostream& os, const Time& t) {
  os << t.sec() << '.' << std::setw(9) << std::setfill('0') << t.nsec();
  os << std::setfill(' ');
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const Duration& t) {
  os << t.sec() << '.' << std::setw(9) << std::setfill('0') << t.nsec();
  os << std::setfill(' ');
  return os;
}

inline constexpr Duration operator-(Time a, Time b) { return Duration(a.ns - b.ns); }
inline constexpr Time operator+(Time t, Duration d) { return Time(t.ns + d.ns); }
inline constexpr Time operator-(Time t, Duration d) { return Time(t.ns - d.ns); }
inline constexpr Time& operator+=(Time& t, Duration d) {
  t.ns += d.ns;
  return t;
}
inline constexpr Time& operator-=(Time& t, Duration d) {
  t.ns -= d.ns;
  return t;
}

inline double stampToSec(const Time& t) { return t.toSec(); }
inline double stampToSec(uint64_t pcl_stamp_us) { return static_cast<double>(pcl_stamp_us) * 1e-6; }
inline double stampSec(const Time& t) { return t.toSec(); }

template <typename TimeLike,
          typename = std::enable_if_t<detail::HasToNSec<TimeLike>::value &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Time> &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Duration>>>
inline bool operator<(const TimeLike& a, Time b) {
  return static_cast<int64_t>(a.toNSec()) < b.toNSec();
}

template <typename TimeLike,
          typename = std::enable_if_t<detail::HasToNSec<TimeLike>::value &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Time> &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Duration>>>
inline bool operator>(const TimeLike& a, Time b) {
  return static_cast<int64_t>(a.toNSec()) > b.toNSec();
}

template <typename TimeLike,
          typename = std::enable_if_t<detail::HasToNSec<TimeLike>::value &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Time> &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Duration>>>
inline bool operator<=(const TimeLike& a, Time b) {
  return static_cast<int64_t>(a.toNSec()) <= b.toNSec();
}

template <typename TimeLike,
          typename = std::enable_if_t<detail::HasToNSec<TimeLike>::value &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Time> &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Duration>>>
inline bool operator>=(const TimeLike& a, Time b) {
  return static_cast<int64_t>(a.toNSec()) >= b.toNSec();
}

template <typename TimeLike,
          typename = std::enable_if_t<detail::HasToNSec<TimeLike>::value &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Time> &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Duration>>>
inline bool operator==(const TimeLike& a, Time b) {
  return static_cast<int64_t>(a.toNSec()) == b.toNSec();
}

template <typename TimeLike,
          typename = std::enable_if_t<detail::HasToNSec<TimeLike>::value &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Time> &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Duration>>>
inline bool operator!=(const TimeLike& a, Time b) {
  return !(a == b);
}

template <typename TimeLike,
          typename = std::enable_if_t<detail::HasToNSec<TimeLike>::value &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Time> &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Duration>>>
inline bool operator<(Time a, const TimeLike& b) {
  return a.toNSec() < static_cast<int64_t>(b.toNSec());
}

template <typename TimeLike,
          typename = std::enable_if_t<detail::HasToNSec<TimeLike>::value &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Time> &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Duration>>>
inline bool operator>(Time a, const TimeLike& b) {
  return a.toNSec() > static_cast<int64_t>(b.toNSec());
}

template <typename TimeLike,
          typename = std::enable_if_t<detail::HasToNSec<TimeLike>::value &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Time> &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Duration>>>
inline bool operator<=(Time a, const TimeLike& b) {
  return a.toNSec() <= static_cast<int64_t>(b.toNSec());
}

template <typename TimeLike,
          typename = std::enable_if_t<detail::HasToNSec<TimeLike>::value &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Time> &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Duration>>>
inline bool operator>=(Time a, const TimeLike& b) {
  return a.toNSec() >= static_cast<int64_t>(b.toNSec());
}

template <typename TimeLike,
          typename = std::enable_if_t<detail::HasToNSec<TimeLike>::value &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Time> &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Duration>>>
inline bool operator==(Time a, const TimeLike& b) {
  return a.toNSec() == static_cast<int64_t>(b.toNSec());
}

template <typename TimeLike,
          typename = std::enable_if_t<detail::HasToNSec<TimeLike>::value &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Time> &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Duration>>>
inline bool operator!=(Time a, const TimeLike& b) {
  return !(a == b);
}

template <typename TimeLike,
          typename = std::enable_if_t<detail::HasToNSec<TimeLike>::value &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Time> &&
                                      !std::is_same_v<std::decay_t<TimeLike>, Duration>>>
inline Duration operator-(const TimeLike& a, Time b) {
  return Duration(static_cast<int64_t>(a.toNSec()) - b.toNSec());
}

}  // namespace lio
