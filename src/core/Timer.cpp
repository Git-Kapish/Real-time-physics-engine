/// @file Timer.cpp
#include "core/Timer.h"
#include <algorithm>

namespace renderer {

Timer::Timer() {
    reset();
}

void Timer::reset() {
    start_       = Clock::now();
    last_        = start_;
    smoothedDt_  = 1.f / 60.f;
    frameCount_  = 0;
}

float Timer::tick() {
    const auto now = Clock::now();
    const float dt = std::chrono::duration<float>(now - last_).count();
    last_          = now;
    ++frameCount_;

    // Clamp to avoid spiral-of-death on first frame or after a stall
    const float clampedDt = std::min(dt, 0.25f);

    // Exponential moving average: smoothed = alpha*new + (1-alpha)*old
    smoothedDt_ = smoothAlpha_ * clampedDt + (1.f - smoothAlpha_) * smoothedDt_;

    return clampedDt;
}

float Timer::elapsed() const {
    return std::chrono::duration<float>(Clock::now() - start_).count();
}

float Timer::fps() const {
    if (smoothedDt_ < 1e-6f) return 0.f;
    return 1.f / smoothedDt_;
}

} // namespace renderer
