#pragma once
/// @file Timer.h
/// @brief High-resolution frame timer with EMA-smoothed FPS counter.

#include <chrono>
#include <cstdint>

namespace renderer {

/**
 * Frame timer built on std::chrono::high_resolution_clock.
 *
 * Call tick() once per frame; it returns dt (seconds since last tick).
 * fps() gives a smoothed frames-per-second using an exponential moving
 * average so the display stays stable even with occasional spiky frames.
 */
class Timer {
public:
    Timer();

    /// Reset start and last-tick time to now; zero the frame counter.
    void reset();

    /**
     * Record the start of a new frame.
     * @return dt  Seconds elapsed since the previous call to tick().
     *             First call returns 1/60 to avoid a zero dt spike.
     */
    float tick();

    /// Total seconds elapsed since construction or the last reset().
    float elapsed() const;

    /// Smoothed FPS (exponential moving average; avoids raw 1/dt spikes).
    float fps() const;

    /// Total number of tick() calls since construction or last reset().
    uint64_t frameCount() const { return frameCount_; }

private:
    using Clock = std::chrono::high_resolution_clock;

    Clock::time_point start_;
    Clock::time_point last_;
    float             smoothedDt_  = 1.f / 60.f;
    float             smoothAlpha_ = 0.1f;   ///< EMA weight for the newest sample
    uint64_t          frameCount_  = 0;
};

} // namespace renderer
