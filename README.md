# Real-Time Physics Engine

A from-scratch rigid body physics engine written in C++17, built across six incremental phases — from bare-metal vector math through to a live OpenGL 3.3 debug renderer.

---

## Features

### Phase 1 — Math Library
Header-only linear algebra with no external dependencies.

| Type | Highlights |
|------|-----------|
| `Vec3` | Dot, cross, length, normalize, linear interpolation |
| `Mat3` | Determinant, inverse, transpose, outer product |
| `Mat4` | TRS construction, perspective/look-at projection, row-major storage |
| `Quat` | SLERP, axis-angle, `toMat3`, `fromAxisAngle`, `identity` |

### Phase 2 — Rigid Body Integration
- `RigidBody` supporting **Sphere**, **Box**, and **Plane** shape types
- **Static**, **Dynamic**, and **Kinematic** body types
- Semi-implicit Euler integration with linear and angular damping
- Per-body restitution and friction coefficients
- `PhysicsWorld` with fixed-timestep accumulator (`1/60 s` default)

### Phase 3 — Collision Detection
- Sphere–sphere, sphere–box, box–box, and half-space (plane) narrow-phase tests
- `ContactManifold` carrying contact point, penetration depth, and contact normal
- `CollisionDetector` decoupled from the world loop for easy testing

### Phase 4 — Impulse Solver
- Sequential impulse resolution with configurable **velocity iterations** (default 8) and **position iterations** (default 3)
- **Baumgarte position correction** to prevent ghost collisions from penetration build-up
- **Coulomb friction** model: tangential impulse clamped to `μ × normal_impulse`
- Configurable penetration slop and restitution threshold
- Body sleep detection based on linear and angular velocity thresholds

### Phase 5 — BVH Broadphase
- Dynamic **AABB Bounding Volume Hierarchy** with **SAH-guided insertion**
- `O(log n)` insertion and removal; AABB union propagated up on update
- Free-list node recycling — zero heap allocations during steady-state simulation
- Generates candidate pairs for the narrow phase, skipping static–static pairs

### Phase 6 — OpenGL Debug Renderer
- **OpenGL 3.3 Core Profile** via a vendored minimal GLAD loader (no external header dependencies)
- **GLFW 3.4** window and input management
- **Animated deep-space gradient background** — top colour slowly cycles through hue
- **Per-body unique wireframe colours** using golden-ratio HSV cycling; static bodies red, kinematic cyan
- **Ghost fill + crisp wireframe** on boxes and spheres (two-pass with alpha blend)
- **4-layer glowing contact points** (22 px outer glow → 3 px white-hot core)
- **Speed-tinted velocity arrows** (green → yellow → red) scaled to magnitude
- **World-origin XYZ axis widget** (red / green / blue), always visible
- **Smooth orbit camera** with exponential-decay interpolation (`1 − e^(−kt)`)
- All GLSL embedded as string literals — no shader file I/O, works from any working directory

---

## Demo Scene

The built-in demo spawns 34 bodies inside a walled arena:

- 1 ground plane + 4 wall planes
- 1 static platform box at the centre
- 20 dynamic spheres (random radii 0.3–0.7 m, random restitution) dropped from above
- 8 dynamic boxes (random half-extents 0.3–0.8 m) dropped from higher up

---

## Controls

| Key / Input | Action |
|------------|--------|
| `Mouse drag` | Orbit camera |
| `Scroll wheel` | Zoom in / out |
| `SPACE` | Pause / resume simulation |
| `R` | Reset scene |
| `G` | Toggle gravity on / off |
| `C` | Toggle contact point visualisation |
| `V` | Toggle velocity vector arrows |
| `B` | Toggle AABB overlay |
| `ESC` | Quit |

---

## Project Structure

```
physics_engine/
├── include/
│   ├── math/           # Vec3, Mat3, Mat4, Quat  (header-only)
│   ├── physics/        # RigidBody, PhysicsWorld, CollisionDetector,
│   │                   # ImpulseSolver, BVHTree, AABB, ContactManifold
│   ├── renderer/       # Camera, Shader, Mesh, DebugRenderer
│   └── core/           # Window, Timer
├── src/
│   ├── physics/        # CollisionDetector, ImpulseSolver, BVHTree, PhysicsWorld
│   ├── renderer/       # Shader, Mesh, Camera, DebugRenderer
│   ├── core/           # Window (GLFW), Timer
│   └── main.cpp        # Demo entry point
├── tests/              # GoogleTest suites (125 tests across 5 files)
├── external/
│   └── glad/           # Vendored minimal OpenGL 3.3 loader
└── CMakeLists.txt
```

---

## Building

### Prerequisites

| Tool | Version |
|------|---------|
| CMake | ≥ 3.16 |
| C++ compiler | GCC 11+ / Clang 14+ / MSVC 2022 (C++17 required) |
| GLFW | 3.x (see platform notes below) |
| OpenGL | 3.3 Core Profile driver |

**Windows (MSYS2 / MinGW64) — recommended:**
```bash
pacman -S mingw-w64-x86_64-glfw
```

**Linux:**
```bash
sudo apt install libglfw3-dev libgl-dev
```

**macOS:**
```bash
brew install glfw
```

### Configuration & Build

```bash
# From the physics_engine/ directory
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target physics_engine -j4
```

GoogleTest is fetched automatically via CMake `FetchContent` — no manual download needed.

### Run

```bash
# Windows
./build/physics_engine.exe

# Linux / macOS
./build/physics_engine
```

On Windows the GLFW DLL must be on `PATH` (MSYS2 installs it to `C:\msys64\mingw64\bin`).

### Run Tests

```bash
cmake --build build --target all -j4
ctest --test-dir build --output-on-failure
```

Expected: **125 tests passing** across `test_math`, `test_physics`, `test_collision`, `test_solver`, `test_bvh`.

---

## Architecture

```
PhysicsWorld::update(dt)
  │
  ├─ Integrate velocities (gravity + forces)
  ├─ BVHTree broadphase  →  candidate pairs
  ├─ CollisionDetector narrow phase  →  ContactManifolds
  ├─ ImpulseSolver  →  velocity + position correction
  └─ Integrate positions
```

The renderer is entirely decoupled from the simulation — `DebugRenderer` reads the world state read-only each frame and never writes back.

---

## Dependencies

| Library | How included |
|---------|-------------|
| [GLFW 3](https://www.glfw.org/) | System package (`find_package`) |
| [GLAD](https://glad.dav1d.de/) | Vendored in `external/glad/` |
| [GoogleTest v1.14](https://github.com/google/googletest) | Auto-fetched via `FetchContent` |

No other third-party dependencies. The math library, physics engine, and renderer are all original code.

---

## License

This project is for educational and portfolio purposes.
