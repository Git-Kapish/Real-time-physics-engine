/// @file Window.cpp
#include "core/Window.h"
#include <iostream>
#include <cstring>

namespace renderer {

/* ── Construction / destruction ────────────────────────────────────────── */

Window::Window(int width, int height, const std::string& title)
    : width_(width), height_(height)
{
    if (!glfwInit()) {
        throw std::runtime_error("[Window] glfwInit() failed");
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif
    glfwWindowHint(GLFW_SAMPLES, 4); // 4× MSAA

    window_ = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        throw std::runtime_error("[Window] glfwCreateWindow() failed");
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1); // v-sync

    // Load OpenGL function pointers
    if (!gladLoadGL()) {
        glfwDestroyWindow(window_);
        glfwTerminate();
        throw std::runtime_error("[Window] gladLoadGL() failed — "
                                 "OpenGL 3.3 Core not supported");
    }

    // Store 'this' so callbacks can reach the Window instance
    glfwSetWindowUserPointer(window_, this);

    // Framebuffer size callback
    glfwSetFramebufferSizeCallback(window_, cbFramebufferSize);

    // Key callback
    glfwSetKeyCallback(window_, cbKey);

    // Mouse callbacks
    glfwSetCursorPosCallback(window_, cbCursorPos);
    glfwSetMouseButtonCallback(window_, cbMouseButton);
    glfwSetScrollCallback(window_, cbScroll);

    // Initial viewport
    glViewport(0, 0, width_, height_);
}

Window::~Window() {
    if (window_) glfwDestroyWindow(window_);
    glfwTerminate();
}

/* ── Frame control ──────────────────────────────────────────────────────── */

bool Window::shouldClose() const {
    return glfwWindowShouldClose(window_) != 0;
}

void Window::pollEvents() {
    glfwPollEvents();
}

void Window::swapBuffers() {
    glfwSwapBuffers(window_);
}

void Window::setTitle(const std::string& title) {
    glfwSetWindowTitle(window_, title.c_str());
}

/* ── Keyboard ───────────────────────────────────────────────────────────── */

bool Window::isKeyPressed(int glfwKey) const {
    return glfwGetKey(window_, glfwKey) == GLFW_PRESS;
}

bool Window::isKeyJustPressed(int glfwKey) {
    const auto it = justPressed_.find(glfwKey);
    if (it != justPressed_.end()) {
        justPressed_.erase(it);
        return true;
    }
    return false;
}

/* ── Mouse ──────────────────────────────────────────────────────────────── */

bool Window::isMouseButtonDown(int glfwButton) const {
    return glfwGetMouseButton(window_, glfwButton) == GLFW_PRESS;
}

float Window::mouseDX() {
    const float v = mouseDX_;
    mouseDX_ = 0.f;
    return v;
}

float Window::mouseDY() {
    const float v = mouseDY_;
    mouseDY_ = 0.f;
    return v;
}

float Window::scrollDY() {
    const float v = scrollDY_;
    scrollDY_ = 0.f;
    return v;
}

/* ── GLFW callbacks ─────────────────────────────────────────────────────── */

void Window::cbFramebufferSize(GLFWwindow* win, int w, int h) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(win));
    self->width_  = w;
    self->height_ = h;
    glViewport(0, 0, w, h);
}

void Window::cbKey(GLFWwindow* win, int key, int /*scancode*/, int action, int /*mods*/) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(win));
    if (action == GLFW_PRESS) {
        self->justPressed_.insert(key);
    }
}

void Window::cbCursorPos(GLFWwindow* win, double xpos, double ypos) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(win));

    const float fx = static_cast<float>(xpos);
    const float fy = static_cast<float>(ypos);

    if (self->firstMouse_) {
        self->lastMouseX_ = fx;
        self->lastMouseY_ = fy;
        self->firstMouse_ = false;
        return;
    }

    const float dx = fx - self->lastMouseX_;
    const float dy = fy - self->lastMouseY_;
    self->lastMouseX_ = fx;
    self->lastMouseY_ = fy;

    if (self->mouseDown_) {
        self->mouseDX_ += dx;
        self->mouseDY_ += dy;
    }
}

void Window::cbMouseButton(GLFWwindow* win, int button, int action, int /*mods*/) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(win));
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        self->mouseDown_ = (action == GLFW_PRESS);
        if (action == GLFW_PRESS) {
            // Reset first-mouse so drag starts cleanly from current position
            self->firstMouse_ = true;
        }
    }
}

void Window::cbScroll(GLFWwindow* win, double /*xoffset*/, double yoffset) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(win));
    self->scrollDY_ += static_cast<float>(yoffset);
}

} // namespace renderer
