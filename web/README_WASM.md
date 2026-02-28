# WebAssembly Build Guide

Port of the real-time physics engine to WebAssembly using Emscripten,
targeting WebGL 2.0 in the browser.

---

## Prerequisites

| Tool | Minimum version | Install |
|------|----------------|---------|
| [Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html) | 3.1.50 | `git clone https://github.com/emscripten-core/emsdk` |
| CMake | 3.16 | System package or [cmake.org](https://cmake.org/download/) |
| Python 3 | 3.8 | Required by `emrun` |

### Activate the SDK (run once per terminal session)

```bash
# Linux / macOS
source ~/emsdk/emsdk_env.sh

# Windows PowerShell
~\emsdk\emsdk_env.ps1
# — or —
~\emsdk\emsdk activate latest
```

---

## Building

All commands are run **from the repo root** (`Real-time-physics-engine/`).

### Release (recommended — optimised for ≥ 60 fps at 500 bodies)

```bash
# 1. Configure with the Emscripten CMake toolchain
emcmake cmake physics_engine \
    -B build_wasm \
    -DCMAKE_BUILD_TYPE=Release

# 2. Build the WASM target (physics core + WebGL renderer + JS bindings)
cmake --build build_wasm --target physics_engine_wasm -j$(nproc)
```

The build produces two files inside `build_wasm/web/`:

```
physics_engine.js    ← Emscripten glue + Embind JS bindings
physics_engine.wasm  ← compiled physics + renderer
```

### Debug (slow — keep for shader / logic debugging)

```bash
emcmake cmake physics_engine \
    -B build_wasm_dbg \
    -DCMAKE_BUILD_TYPE=Debug

cmake --build build_wasm_dbg --target physics_engine_wasm
```

---

## Running locally with `emrun`

Browsers block local `file://` WASM loads.  Use `emrun` (bundled with the
Emscripten SDK) to spin up a tiny HTTP server:

```bash
# Serve on http://localhost:8080/index.html
emrun --port 8080 --no-browser build_wasm/web/index.html

# — or open the browser explicitly —
emrun --port 8080 build_wasm/web/index.html
```

Alternatively, any static HTTP server works:

```bash
# Python 3
cd build_wasm/web && python3 -m http.server 8080
# then open http://localhost:8080

# Node.js serve
npx serve build_wasm/web
```

---

## JavaScript API

The WASM module is loaded as an ES-module factory (named `PhysicsEngine`).
After `await PhysicsEngine()` the following Embind-exported functions are
available on the returned module object:

```js
const M = await PhysicsEngine();

// ── Spawning ──────────────────────────────────────────────────────────────────
M.addBox(x, y, z, hx, hy, hz, mass);   // → body ID (int)
M.addSphere(x, y, z, radius, mass);    // → body ID (int)
M.spawnRandomBox();                    // → body ID
M.spawnRandomSphere();                 // → body ID

// ── Simulation ────────────────────────────────────────────────────────────────
M.setGravity(gx, gy, gz);             // override gravity vector
M.toggleGravity();                    // flip between ±9.81 and 0
M.step(dt);                           // advance exactly one step (seconds)
M.reset();                            // clear all bodies, rebuild default scene
M.setPaused(bool);
M.togglePaused();
M.isPaused();                         // → bool

// ── Visualisation toggles ─────────────────────────────────────────────────────
M.setShowContacts(bool);
M.setShowVelocities(bool);
M.setShowAABBs(bool);

// ── Stats ─────────────────────────────────────────────────────────────────────
M.getFPS();           // → float (rolling 0.5 s average)
M.getBodyCount();     // → int
M.getContactCount();  // → int (contacts from the last step)
M.getStepCount();     // → BigInt (total fixed steps taken)

// ── Data export ───────────────────────────────────────────────────────────────
M.getBodyTransforms();
// Returns a JS Array of objects:
// { id, px, py, pz, qx, qy, qz, qw, shapeType (0=Sphere,1=Box), radius, hx, hy, hz }
```

---

## Performance targets

| Bodies | Expected FPS |
|--------|-------------|
| 100    | 60+ |
| 300    | 60 |
| 500    | 50–60 |
| 1 000  | ~30 |

The BVH broadphase ensures `O(n log n)` collision detection.
For maximum performance make sure to use the **Release** build
(`-DCMAKE_BUILD_TYPE=Release`) which enables `-O3 --closure=1`.

---

## Build system overview

```
physics_engine/
├── CMakeLists.txt            ← native build; adds web/ when EMSCRIPTEN is set
├── src/
│   ├── CMakeLists.txt        ← builds physics_engine_lib (physics core only)
│   └── physics/…             ← UNTOUCHED physics code
├── include/…                 ← UNTOUCHED headers
└── web/
    ├── CMakeLists.txt        ← physics_engine_wasm target
    ├── main_wasm.cpp         ← Emscripten entry point + Embind JS bindings
    ├── WebGLRenderer.h/.cpp  ← WebGL 2.0 wireframe renderer (GLES 3 shaders)
    └── index.html            ← HTML shell (canvas + control panel + stats)
```

The **physics core** (`src/physics/`, `include/`) is compiled unchanged into
`physics_engine_lib`.  Only the entry point and renderer differ between the
native and WASM builds.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `SharedArrayBuffer` warning | Pass `--no-experimental-fetch` to `emrun`, or add COOP/COEP headers |
| Black screen, no 3-D content | Open DevTools; check for shader compile errors in the console |
| `RangeError: Out of memory` | Increase `INITIAL_MEMORY` or use `-sALLOW_MEMORY_GROWTH=1` (already set) |
| `emrun` not found | Run `. ~/emsdk/emsdk_env.sh`; emsdk must be on PATH |
| WASM module fails to load | Must be served over HTTP — `file://` URLs are blocked by browsers |
| Closure compiler errors | Build Debug first to rule out C++ issues, then switch to Release |
