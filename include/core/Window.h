#pragma once
/// @file Window.h
/// @brief GLFW window with an OpenGL 3.3 Core context.

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <string>
#include <unordered_set>
#include <stdexcept>

namespace renderer {

/**
 * RAII wrapper around a GLFW window + OpenGL 3.3 Core context.
 *
 * Initialises GLFW, creates the window, loads OpenGL functions via
 * gladLoadGL(), and installs the input callbacks.
 *
 * Keyboard events:
 *   isKeyPressed(key)      — true while the key is held.
 *   isKeyJustPressed(key)  — true only on the first frame the key is held;
 *                            resets automatically after being read once.
 *
 * Mouse deltas (mouse-drag orbit):
 *   mouseDX() / mouseDY()  — pixel delta since last pollEvents(); reading
 *                            resets the accumulator.
 *   scrollDY()             — vertical scroll delta; reading resets it.
 *   isMouseButtonDown(btn) — true while the mouse button is held.
 *
 * @throws std::runtime_error if GLFW initialisation or context creation fails.
 */
class Window {
public:
    Window(int width, int height, const std::string& title);
    ~Window();

    Window(const Window&)            = delete;
    Window& operator=(const Window&) = delete;

    // ── Frame control ─────────────────────────────────────────────────────

    bool shouldClose() const;
    void pollEvents();
    void swapBuffers();
    void setTitle(const std::string& title);

    // ── Dimensions ────────────────────────────────────────────────────────

    int   width()  const { return width_; }
    int   height() const { return height_; }
    float aspect() const {
        return (height_ > 0) ? static_cast<float>(width_) / static_cast<float>(height_) : 1.f;
    }

    // ── Keyboard ──────────────────────────────────────────────────────────

    /// True while the key is physically held down.
    bool isKeyPressed(int glfwKey) const;

    /**
     * True only on the first poll after a key-press event.
     * Reading consumes the flag (returns false on subsequent calls
     * for the same key until it is released and pressed again).
     */
    bool isKeyJustPressed(int glfwKey);

    // ── Mouse ─────────────────────────────────────────────────────────────

    bool  isMouseButtonDown(int glfwButton) const;

    /// Horizontal cursor delta since last pollEvents(); resets on read.
    float mouseDX();
    /// Vertical cursor delta since last pollEvents(); resets on read.
    float mouseDY();
    /// Vertical scroll delta since last pollEvents(); resets on read.
    float scrollDY();

    // ── Native handle ─────────────────────────────────────────────────────

    void* nativeHandle() const { return window_; }

private:
    GLFWwindow* window_  = nullptr;
    int         width_   = 0;
    int         height_  = 0;

    // Mouse state
    float mouseDX_   = 0.f;
    float mouseDY_   = 0.f;
    float scrollDY_  = 0.f;
    float lastMouseX_ = 0.f;
    float lastMouseY_ = 0.f;
    bool  mouseDown_ = false;
    bool  firstMouse_ = true;

    // Keyboard "just pressed" set
    std::unordered_set<int> justPressed_;

    // ── GLFW static callbacks ─────────────────────────────────────────────
    static void cbFramebufferSize(GLFWwindow*, int w, int h);
    static void cbKey           (GLFWwindow*, int key, int scancode, int action, int mods);
    static void cbCursorPos     (GLFWwindow*, double xpos, double ypos);
    static void cbMouseButton   (GLFWwindow*, int button, int action, int mods);
    static void cbScroll        (GLFWwindow*, double xoffset, double yoffset);
};

} // namespace renderer
