/// @file Camera.cpp
#include "renderer/Camera.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace renderer {

Camera::Camera(Vec3 target, float distance, float yaw, float pitch)
    : target_(target)
    , distance_(distance)
    , yaw_(yaw)
    , pitch_(pitch)
    , smoothYaw_(yaw)
    , smoothPitch_(pitch)
    , smoothDist_(distance)
    , defaultTarget_(target)
    , defaultDistance_(distance)
    , defaultYaw_(yaw)
    , defaultPitch_(pitch)
{}

Mat4 Camera::viewMatrix() const {
    const Vec3 eye = computePosition();
    return Mat4::lookAt(eye, target_, Vec3(0.f, 1.f, 0.f));
}

Mat4 Camera::projMatrix(float aspect) const {
    const float fovRad = fovY_ * static_cast<float>(M_PI) / 180.f;
    return Mat4::perspective(fovRad, aspect, nearPlane_, farPlane_);
}

void Camera::onMouseDrag(float dx, float dy) {
    yaw_   += dx * sensitivity_;
    pitch_ += dy * sensitivity_;
    pitch_  = std::clamp(pitch_, -89.f, 89.f);
}

void Camera::onScroll(float delta) {
    distance_ -= delta * zoomSpeed_;
    if (distance_ < 1.f)   distance_ = 1.f;
    if (distance_ > 200.f) distance_ = 200.f;
}

void Camera::onResize(int /*w*/, int /*h*/) {
    // Aspect ratio is passed per-frame via projMatrix(); nothing to store here.
}

void Camera::reset() {
    target_      = defaultTarget_;
    distance_    = defaultDistance_;
    yaw_         = defaultYaw_;
    pitch_       = defaultPitch_;
    smoothYaw_   = defaultYaw_;
    smoothPitch_ = defaultPitch_;
    smoothDist_  = defaultDistance_;
}

void Camera::update(float dt) {
    // Exponential-decay smoothing — framerate-independent
    const float a = 1.f - std::exp(-smoothK_ * dt);
    smoothYaw_   += (yaw_      - smoothYaw_)   * a;
    smoothPitch_ += (pitch_    - smoothPitch_) * a;
    smoothDist_  += (distance_ - smoothDist_)  * a;
}

Vec3 Camera::computePosition() const {
    const float pitchRad = smoothPitch_ * static_cast<float>(M_PI) / 180.f;
    const float yawRad   = smoothYaw_   * static_cast<float>(M_PI) / 180.f;
    const float cosPitch = std::cos(pitchRad);
    const float sinPitch = std::sin(pitchRad);
    const float cosYaw   = std::cos(yawRad);
    const float sinYaw   = std::sin(yawRad);
    return target_ + Vec3(
        smoothDist_ * cosPitch * cosYaw,
        smoothDist_ * sinPitch,
        smoothDist_ * cosPitch * sinYaw
    );
}

} // namespace renderer
