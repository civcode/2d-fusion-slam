#pragma once

#include <cstdint>
#include <variant>

namespace slam {
using TimeNs = std::int64_t;

struct DurationNs {
  TimeNs value{};
};

// Different sources are not temporally comparable until explicitly mapped.
struct Timestamp {
  TimeNs ns{};
  std::uint32_t clock_domain{};
  std::uint64_t epoch{};
};

enum class TimeError { different_domain, different_epoch, overflow };
using DurationResult = std::variant<DurationNs, TimeError>;
using TimestampResult = std::variant<Timestamp, TimeError>;

[[nodiscard]] DurationResult between(Timestamp later, Timestamp earlier) noexcept;
[[nodiscard]] TimestampResult advance(Timestamp origin, DurationNs delta) noexcept;
[[nodiscard]] double seconds(DurationNs duration) noexcept;
}  // namespace slam
