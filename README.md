# ⚙️ Real-Time Physics Engine

[![Build & Test](https://github.com/Git-Kapish/Real-time-physics-engine/actions/workflows/ci.yml/badge.svg)](https://github.com/Git-Kapish/Real-time-physics-engine/actions/workflows/ci.yml)
![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
[![License: MIT](https://img.shields.io/badge/license-MIT-yellow.svg)](LICENSE)

> A from-scratch rigid body physics engine in C++17 — custom quaternion integration, SAH-guided BVH broadphase, impulse-based collision resolution with Coulomb friction, and a live OpenGL 3.3 debug renderer. **Zero external physics or math libraries.**

---
## 🏗️ Architecture

```
PhysicsWorld::update(dt)
  │
  ├─ 1. applyGravity()               — accumulate F = m·g on dynamic bodies
  ├─ 2. integrateForces(dt)          — v += (F/m)·dt   (semi-implicit Euler)
  ├─ 3. integrateVelocities(dt)      — x += v·dt + quaternion integration
  ├─ 4. updateInertiaTensors()       — R · I⁻¹_local · Rᵀ after each rotation
  ├─ 5. BVHTree::update()            — refit fat AABBs; reinsert if moved out
  ├─ 6. CollisionDetector            — broadPhase (BVH) → narrowPhase (SAT/GJK)
  ├─ 7. ImpulseSolver::solve()       — velocity resolution + Baumgarte correction
  └─ 8. clearAllForces()

DebugRenderer (read-only — never writes back to world)
  │
  ├─ Gradient background shader
  ├─ Per-body wireframe (ghost fill + crisp outline, two-pass)
  ├─ 4-layer glowing contact points
  ├─ Speed-tinted velocity arrows  (green → yellow → red)
  └─ World-axis widget  (XYZ = R/G/B)
```

---

## 🚀 Features by Phase

### Phase 1 — Math Library *(header-only, zero dependencies)*

| Type | Operations |
|------|------------|
| `Vec3` | Dot, cross, length, normalize, lerp, `cwiseProduct`, indexed access |
| `Mat3` | Multiply, transpose, inverse, determinant, skew-symmetric builder |
| `Mat4` | TRS constructor, perspective, lookAt (column-major for OpenGL) |
| `Quat` | `fromAxisAngle`, `toMat3`, `rotate(Vec3)`, `integrated(ω, dt)`, SLERP |

### Phase 2 — Rigid Body Integration

- `RigidBody` supporting **Sphere**, **Box**, and **Plane** shape primitives
- **Static**, **Dynamic**, and **Kinematic** body types with correct mass/inertia handling
- **Semi-implicit Euler** integration — more stable than explicit, simpler than RK4
- Analytically correct **inertia tensors**: sphere `I = ⅖mr²`, box per-axis formula
- `PhysicsWorld` with a fixed-timestep accumulator (default `1/60 s`)
- Linear and angular **damping** to stabilise resting stacks

### Phase 3 — Collision Detection

Five narrow-phase shape pairs, all returning a `ContactManifold`:

| Pair | Algorithm |
|------|-----------|
| Sphere–Sphere | Distance vs. sum-of-radii |
| Sphere–Plane | Signed distance to half-space |
| Sphere–Box | Closest-point clamp in box-local space; inside-case handled |
| Box–Plane | All 8 corners tested; deepest returned |
| Box–Box | **15-axis SAT** (3+3 face normals + 9 edge cross-products) |

`CollisionDetector` is stateless and decoupled from the world loop for easy unit testing.

### Phase 4 — Impulse Solver

- **Sequential impulse** with configurable velocity iterations (default 8) and position iterations (default 3)
- Normal impulse: `j = -(1+e)·vₙ / effectiveMass`, clamped ≥ 0
- **Coulomb friction** cone: tangential impulse clamped to `|jₜ| ≤ μ·j`
- **Baumgarte position correction** prevents penetration drift/sinking
- Configurable `restitutionThreshold` suppresses bounce on low-speed impacts
- **Body sleep detection** based on linear and angular velocity thresholds

### Phase 5 — BVH Broadphase

| Feature | Detail |
|---------|--------|
| Structure | Dynamic AABB tree — leaves are bodies, internals are unions |
| Insertion | **SAH branch-and-bound** with priority queue — O(log n) |
| Memory | **Free-list recycling** — zero heap allocations in steady state |
| Update | **Fat AABB** (±10 % margin) prevents thrashing on small moves |
| Query | `queryAllPairs()` with self-collision traversal; `queryAABB()` for point queries |

#### BVH Benchmark *(BVH vs O(n²) brute force, 50 steps, no gravity)*

| Bodies (N) | Brute Force (ms/step) | BVH (ms/step) | Speedup |
|:----------:|:---------------------:|:-------------:|:-------:|
| 50         | 0.0114                | 0.0110        | 1.0×    |
| 100        | 0.0262                | 0.0188        | 1.4×    |
| 250        | 0.0775                | 0.0411        | 1.9×    |
| 500        | 0.2055                | 0.0765        | 2.7×    |
| 1 000      | 0.6659                | 0.1236        | 5.4×    |

### Phase 6 — OpenGL Debug Renderer

- **OpenGL 3.3 Core Profile** via a vendored minimal GLAD loader (no external headers)
- All GLSL embedded as string literals — works from any working directory
- **Animated deep-space gradient** background (hue slowly cycles over time)
- **Per-body unique wireframe** using golden-ratio HSV cycling; static = red, kinematic = cyan
- **Ghost fill + crisp wireframe** two-pass rendering with alpha blend
- **4-layer glowing contact points**: 22 px outer glow → 3 px white-hot core
- **Speed-tinted velocity arrows**: green → yellow → red, scaled to magnitude
- **World-origin XYZ axis widget** always visible
- **Smooth orbit camera** with exponential-decay interpolation `1 − e^(−kt)`

---

## 🎮 Demo Scene & Controls

The built-in demo spawns **34 bodies** inside a walled arena:

- 1 ground plane + 4 wall planes
- 1 static platform box at the centre
- 20 dynamic spheres (radii 0.3–0.7 m, random restitution) dropped from above
- 8 dynamic boxes (half-extents 0.3–0.8 m) dropped from higher up

| Key / Input | Action |
|-------------|--------|
| `Mouse drag` | Orbit camera |
| `Scroll wheel` | Zoom in / out |
| `SPACE` | Pause / resume |
| `R` | Reset scene |
| `G` | Toggle gravity |
| `C` | Toggle contact visualisation |
| `V` | Toggle velocity arrows |
| `B` | Toggle AABB overlay |
| `ESC` | Quit |

---

## 🛠️ Building

### Prerequisites

| Tool | Version |
|------|---------|
| CMake | ≥ 3.16 |
| C++ compiler | GCC 11+ / Clang 14+ / MSVC 2022 (C++17 required) |
| GLFW | 3.x |
| OpenGL driver | 3.3 Core Profile |

**Install GLFW:**

```bash
# Windows (MSYS2 / MinGW64)
pacman -S mingw-w64-x86_64-glfw

# Linux (Debian/Ubuntu)
sudo apt install libglfw3-dev libgl-dev

# macOS
brew install glfw
```

### Configure & Build

```bash
# From the repo root
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target physics_engine -j$(nproc)
```

GoogleTest is fetched automatically via `FetchContent` — no manual download needed.

### Run

```bash
# Linux / macOS
./build/physics_engine

# Windows (MSYS2)
./build/physics_engine.exe
# GLFW DLL must be on PATH — MSYS2 installs it to C:\msys64\mingw64\bin
```

### Run Tests

```bash
cmake --build build --target all -j$(nproc)
ctest --test-dir build --output-on-failure
```

Expected: **125 tests passing** across `test_math`, `test_physics`, `test_collision`, `test_solver`, `test_bvh`.

### Running the Benchmark

```bash
cmake --build build --target physics_benchmark -j$(nproc)
./build/physics_benchmark
```

Copy the output table into the [BVH Benchmark](#phase-5--bvh-broadphase) section above.

---

## 📁 Project Structure

```
Real-time-physics-engine/
├── include/
│   ├── math/           # Vec3, Mat3, Mat4, Quat  (header-only)
│   ├── physics/        # RigidBody, PhysicsWorld, CollisionDetector,
│   │                   # ImpulseSolver, BVHTree, AABB, ContactManifold
│   ├── renderer/       # Camera, Shader, Mesh, DebugRenderer
│   └── core/           # Window (GLFW), Timer
├── src/
│   ├── physics/        # CollisionDetector, ImpulseSolver, BVHTree, PhysicsWorld, RigidBody
│   ├── renderer/       # Shader, Mesh, Camera, DebugRenderer
│   ├── core/           # Window, Timer
│   ├── benchmark.cpp   # BVH vs brute-force timing
│   └── main.cpp        # Demo entry point
├── tests/              # 125 GoogleTest cases (5 suites)
├── external/
│   └── glad/           # Vendored minimal OpenGL 3.3 loader
├── shaders/
│   ├── debug.vert
│   └── debug.frag
└── CMakeLists.txt
```

---

## 📦 Dependencies

| Library | How included | Why |
|---------|-------------|-----|
| [GLFW 3](https://www.glfw.org/) | System package (`find_package`) | Window + input |
| [GLAD](https://glad.dav1d.de/) | Vendored in `external/glad/` | OpenGL 3.3 loader |
| [GoogleTest v1.14](https://github.com/google/googletest) | Auto-fetched via `FetchContent` | Unit tests |

**No external physics or math libraries.** Every vector, matrix, quaternion, collision algorithm, constraint solver, and BVH structure is original code.

---

## 📄 License

This project is licensed under the [MIT License](LICENSE).
