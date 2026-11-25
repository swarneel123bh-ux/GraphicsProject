# `live.c`: City Ray Tracer

`scratch_raytracer/live.c` is a single-file C program (about 1,200 lines including Doxygen comments) that ray traces a small city block on the CPU and shows the result in a window. You can walk around the scene with the keyboard and mouse while it renders. It is the IS F311 Computer Graphics project.

The file contains:

- a CPU ray tracer with boxes, spheres, cylinders and cones, hard shadows, point lamps, a sun, a sky gradient and mirror reflections;
- a 2D grid that speeds up ray traversal;
- a multithreaded row-based renderer using pthreads;
- a window backend that works with either GLFW or FreeGLUT;
- two headless modes: a single-frame snapshot to a BMP file, and a benchmark.

The project's `Makefile` (in the repository root) builds it. The file's own header comment at [live.c:1](live.c:1) lists the same commands by hand.

---

## Building and running

Install GLFW or FreeGLUT, then run `make` from the project root. The Makefile prefers GLFW when both are present.

```bash
make                    # builds build/live-glfw (or build/live-freeglut)
make BACKEND=freeglut   # force a backend
make run-live           # build and run the window
make clean              # delete build/
```

The Makefile compiles with `-std=c11 -O2 -Wall -Wno-deprecated-declarations -pthread`. It defines `USE_GLFW` or `USE_FREEGLUT`. The program stops with a compile error if neither is defined ([live.c:30-36](live.c:30)).

Running the program:

```bash
./build/live-glfw                          # interactive window
./build/live-glfw --snapshot out.bmp       # one 640x360 frame, saved as BMP, no window
./build/live-glfw --bench 60               # 60 frames along a fixed path, prints FPS, no window
```

Replace `live-glfw` with `live-freeglut` if you built the FreeGLUT backend.

### Controls (window mode)

| Input | Action |
|---|---|
| Mouse | Look around (yaw and pitch) |
| W / A / S / D | Walk forward, left, back, right |
| Shift | Run (3x speed) |
| Scroll wheel, `+` / `-` | Zoom (change field of view) |
| `R` | Reset camera to start |
| `1` / `2` / `3` | Render at 320x180 / 480x270 / 640x360 |
| `T` | Pause or resume the day-night cycle |
| `Esc` | Quit |

The window title shows the render size, the average frame time and FPS, and whether the day-night cycle is running.

---

## Command-line modes

Handled in `main()` ([live.c:1175](live.c:1175)). The `--snapshot` and `--bench` modes do not open a window, so they do not need a GL context.

- **`--snapshot <path>`** builds the scene, places the camera at its start position, renders one 640x360 frame with 1 sample per pixel, and writes a 24-bit BMP. It prints the render time. It returns 1 if the file cannot be opened.
- **`--bench <frames>`** renders `<frames>` frames at the currently selected resolution (default 480x270). Between frames the camera moves 0.4 units along +z and the sun angle changes slightly. It prints the average frame time and FPS.
  - `--bench 0` divides by zero and prints `nan`.
  - `--bench` with a non-number is read by `atoi` as 0, so it has the same problem.
- With no arguments, or with any other arguments, the program opens the window.

---

## Overview of the source

The file is organised into sections, marked with `---------- ... ----------` comments. The line numbers below are the ones in the current file.

| Section | Lines | Purpose |
|---|---|---|
| Includes and config | 1-60 | Header comment, platform setup, backend selection, constants |
| Vector maths | 62-73 | `V3` type and helper operations |
| Objects | 75-107 | `Obj` and `Lamp` structs, global scene arrays, `add_obj`, `add_lamp` |
| Intersections | 109-233 | Ray tests for box, sphere, cylinder, cone |
| 2D grid and traversal | 235-371 | Spatial index on the XZ plane, `traverse`, `closest`, `occluded` |
| Shading | 373-436 | Sky, `trace`, `update_light` (sun, lamps, day-night) |
| Scene | 438-469 | Random number generator, `build_scene` (the city block) |
| Camera | 471-491 | `Cam`, `cam_reset`, `current_view` |
| Render | 493-564 | `render_row`, threaded `worker`, `render_frame` |
| Timing and CPU count | 566-589 | `now_ms`, `cpu_count` |
| BMP output | 591-621 | `write_bmp` |
| Window state and actions | 623-664 | Shared globals, action enum, `do_action`, `look`, `zoom` |
| Held keys | 666-692 | Key-state enum, `key_down`, `handle_move` |
| Drawing | 694-735 | `setup_gl`, `upload_and_draw`, console messages |
| Window backends | 737-836 | GLFW and FreeGLUT callbacks, timing functions |
| Frame | 838-867 | `frame(dt)`: move, light, render, draw, update title |
| Headless modes | 869-899 | `run_snapshot`, `run_bench` |
| Main loops | 901-962 | `run_window` for GLFW and for FreeGLUT |
| `main` | 964-976 | Thread count, buffer allocation, mode dispatch |

---

## Reference by section

### Vector maths (lines 62-73)

`V3` is a 3-component `double` vector. The helpers are `V` (constructor), `add`, `sub`, `scl` (scale), `mul` (component-wise product), `dot`, `cross`, `norm` (normalise), `neg`, and `mix` (linear interpolation).

### Objects and the scene arrays (lines 75-107)

- `enum { BOX, SPHERE, CYL, CONE }`: object types.
- `Obj` holds:
  - `type`;
  - `lo`, `hi`: for a box, min and max corners; for a sphere, `lo` is the centre; for a cylinder or cone, `lo` is `(cx, y0, cz)` and `hi.y` is `y1`;
  - `r`: radius;
  - `col`: albedo;
  - `refl`: mirror amount, 0 to 1;
  - `emis`: if above 0, the object is self-lit and does not cast shadows.
- `Lamp` holds `pos`, `col` and `k` (intensity).
- Global arrays are `objs[MAX_OBJ]` (512) and `lamps[MAX_LAMP]` (16). `nobj` and `nlamp` are the counts. Sun and sky parameters are globals set by `update_light`.
- `add_obj` and `add_lamp` append to these arrays. They do not check capacity against `MAX_OBJ` or `MAX_LAMP`.

### Intersections (lines 109-233)

Each function returns 1 on a hit and fills `*t` (distance along the ray) and `*n` (normal). Each takes `tmax`, the largest distance it will accept.

- `hit_box` ([live.c:167](live.c:167)) is a slab test. It returns the entry face as the normal. Rays parallel to an axis outside the slab miss.
- `hit_sphere` ([live.c:201](live.c:201)) solves the quadratic and uses the nearer root that is greater than `EPS`. Rays starting inside the sphere are not counted as hits on the near surface.
- `hit_cyl` ([live.c:225](live.c:225)) tests the side (a quadratic in XZ, checked against `y0`..`y1`) and both caps (planes at `y0` and `y1`, checked against the radius).
- `hit_cone` ([live.c:275](live.c:275)) uses the implicit equation `x² + z² = k²(y1 - y)²`, with `k = r / (y1 - y0)`. It solves a quadratic for the side (or a linear equation when the quadratic term vanishes), checks the bounds, and tests the base cap at `y0`. The apex is at `y1`.
- `hit_obj` ([live.c:321](live.c:321)) dispatches on `type`.

`EPS` is `1e-4`. It rejects hits too close to the origin, which avoids self-intersection.

### 2D grid and traversal (lines 235-371)

The XZ plane is split into a 36x36 grid of 4-unit cells starting at `(-72, -72)`. Each cell holds up to `CELL_CAP` (96) object indices. An object is added to every cell its XZ footprint touches. Objects whose footprint is larger than `GLOBAL_EXTENT` (20 units) go in a separate `glob` list, which every ray tests. The ground plane is one of these.

`build_grid` ([live.c:359](live.c:359)) fills the grid. Cells are clamped to the grid edges. If a cell overflows, the program prints `grid cell full` and exits.

`traverse` ([live.c:398](live.c:398)) is the core query. It has two modes:

- **nearest hit** (`any == 0`): returns the closest object and its distance, or -1;
- **any hit before `tmax`** (`any == 1`): returns the first object found, or -1. Emissive objects are skipped, so lamp globes do not block light.

The algorithm:

1. Test the global objects.
2. Clip the ray to the grid square with a slab test on X and Z.
3. Step through the grid cells the ray crosses (2D DDA), testing the objects in each cell.
4. Stop when the next cell starts beyond the nearest hit found so far, or outside the grid.

`closest` ([live.c:480](live.c:480)) and `occluded` ([live.c:490](live.c:490)) are thin wrappers. `occluded` returns true if anything blocks a shadow ray of length `maxdist`.

The counter `nrays` (declared at [live.c:122](live.c:122) as thread-local) increments on every call to `traverse`. It is collected per thread but never printed.

### Shading (lines 373-436)

- `sky` ([live.c:498](live.c:498)) blends `sky_hor` and `sky_top` based on the ray's vertical component.
- `trace` ([live.c:514](live.c:514)) is recursive:
  1. Find the nearest hit. On a miss, return the sky colour.
  2. Emissive objects return `col * emis`.
  3. Start with ambient light, `col * ambient`.
  4. Add the sun: a directional light with a hard shadow. It is used only when the sun is above the horizon (`sun_dir.y > 0.02`).
  5. Add each lamp: a point light with the falloff `1 / (1 + 0.08 d²)` and a hard shadow.
  6. If the surface is mirror-like (`refl > 0`) and `depth > 0`, trace a reflected ray and blend it in by `refl`.
- `MAX_DEPTH` is 2, so reflections can bounce twice.

`update_light(a)` ([live.c:562](live.c:562)) computes everything that depends on the time of day. `a` is the sun angle in radians.

- The sun direction is `norm(-0.7, sin a, 0.35)`, so the sun moves through the sky in a plane.
- The sun's colour gets warmer as it gets lower.
- Daylight (`daylt`) scales the ambient light and sky brightness. It falls to zero at night.
- A dusk tint is applied near the horizon.
- Lamps get brighter at night (`k = 1.2 + 2.5 * (1 - daylt)`).

### Scene (lines 438-469)

`frand` ([live.c:581](live.c:581)) is a deterministic linear congruential generator with seed 12345. The same seed always gives the same buildings.

`build_scene` ([live.c:591](live.c:591)) creates the city block:

- a large dark ground box (140x140 units, top surface at y=0), with 12% mirror reflectivity;
- a grid of white road markings (boxes 0.2 units thick) along x and z;
- a 6x6 grid of block positions where about 80% have a building. Each building is a box up to 20 units tall, with a random warm colour. Buildings are 7x7 units with a 12-unit spacing;
- a red mirror sphere at the origin;
- four street lamps, each with a dark cylinder pole, a glowing sphere (`emis = 1.0`) at the top, and a point light just below it;
- the grid is built last, at [live.c:614](live.c:614).

The comment says the scene is "from rt.c". That file is not in this repository.

### Camera (lines 471-491)

`Cam` holds position `(x, y, z)`, yaw and pitch in degrees, and vertical field of view. Yaw 0 looks down +z, matching the Assignment 2 convention. The default start is `(0, 1.7, -30)` with pitch -3 and FOV 60 ([live.c:621](live.c:621)).

`current_view` builds the eye position and the forward, right and up vectors. It also stores `th = tan(fov / 2)` for the projection.

### Rendering (lines 493-564)

- `render_row` ([live.c:661](live.c:661)) renders one row. For each pixel it casts `spp × spp` rays at regularly spaced positions across the pixel, averages the colours, applies gamma (`pow(v, 1/2.2)`), and writes 8-bit RGB into the image buffer.
- `worker` ([live.c:689](live.c:689)) takes rows from a shared counter under a mutex until none are left.
- `render_frame` ([live.c:709](live.c:709)) creates `nthreads` workers, waits for them, and destroys the mutex. The thread count is the number of online CPUs, capped at 64.

The frame code always passes `spp = 1`. The parameter is there if you want more samples.

### Timing and CPU count (lines 566-589)

`now_ms` uses `QueryPerformanceCounter` on Windows and `CLOCK_MONOTONIC` elsewhere. `cpu_count` uses `GetSystemInfo` on Windows and `sysconf(_SC_NPROCESSORS_ONLN)` elsewhere.

### BMP output (lines 591-621)

`write_bmp` writes an uncompressed 24-bit BMP. It pads each row to 4 bytes, stores rows bottom-up, and swaps RGB to BGR. Helpers `put32` and `put16` write little-endian integers.

### Window state and actions (lines 623-664)

Shared globals include the render size table (`res_w`, `res_h`), `res_index` (default 1, which is 480x270), `nthreads`, `cycle_running`, `sun_angle`, the image buffer `buf` (allocated in `main`), and the GL texture handle.

Actions are an enum (`ACT_QUIT`, `ACT_RESET`, `ACT_TIME`, `ACT_RES1` to `ACT_RES3`) dispatched by `do_action`. Both backends call it, so key bindings are defined in one place per backend.

- `look(dx, dy)` changes yaw by `dx * 0.1` degrees and pitch by `-dy * 0.1` degrees, clamped to ±89.
- `zoom(steps)` changes the FOV by `-2 * steps`, clamped to 20-100.

### Held keys and movement (lines 666-692)

`key_down(k)` reads whether a key is held:

- **GLFW** polls the key state with `glfwGetKey` each frame.
- **FreeGLUT** keeps its own array, `key_held`, updated from keyboard callbacks, because FreeGLUT does not provide held-key polling.

`handle_move(dt)` moves the camera on the ground plane (y stays fixed) at `WALK_SPEED` (5 m/s), tripled while Shift is held. It also changes the zoom continuously while `+`/`-` are held.

### Drawing (lines 694-735)

`setup_gl` creates the texture, which uses linear filtering and unpacked rows of 1-byte alignment. `upload_and_draw` uploads the CPU image, reallocates the texture only if the size changes, and draws it as a full-window quad in orthographic projection with lighting and depth test disabled.

`print_scene_info` and `print_controls` print startup messages to stdout.

### Window backends (lines 737-836)

Each backend provides the same small set of functions: `win_time`, `win_framebuffer`, `win_swap`, `win_set_title`, and input callbacks.

- **GLFW** ([live.c:929](live.c:929)): the cursor is hidden and captured (`GLFW_CURSOR_DISABLED`). Mouse motion is read from the position callback as relative movement. Scroll zooms. Key presses call `do_action`. Movement keys are polled each frame.
- **FreeGLUT** ([live.c:966](live.c:966)): keyboard callbacks update `key_held` and call `do_action`. Shift is read from `glutGetModifiers` only inside key callbacks, so the run flag updates on key press and release. Mouse look keeps the pointer at the window centre with `glutWarpPointer` and uses the offset from the centre. `glut_idle` drives the frame loop.

### Frame (lines 838-867)

`frame(dt)` is called once per iteration of the main loop. `dt` is capped at 0.1 seconds. It:

1. moves the camera;
2. advances the sun angle if the cycle is running (`0.12` rad per second);
3. updates the lighting;
4. renders at the current resolution;
5. uploads and draws the result, then swaps buffers;
6. about four times per second, updates the window title with the render size, frame time, FPS and cycle state.

The frame time is smoothed with an exponential average (`0.9` old, `0.1` new).

### Headless modes (lines 869-899)

`run_snapshot` and `run_bench` build the scene and render with `render_frame` directly, without any window or GL calls. The benchmark moves the camera and sun between frames so the cost is not measured on one static view.

### Main loops (lines 901-962)

`run_window` has one version per backend.

- **GLFW**: initialises GLFW, creates a 1280x720 window, turns off vsync (`glfwSwapInterval(0)`) so the displayed FPS reflects the real render rate, sets up callbacks and GL state, builds the scene, and runs `glfwPollEvents` plus `frame` until the window closes or Esc is pressed.
- **FreeGLUT**: initialises GLUT, registers callbacks, hides the cursor, and enters `glutMainLoop`. The idle callback runs `frame`. `glutLeaveMainLoop` ends the loop when `quit_requested` is set.

### `main` (lines 964-976)

`main` sets `nthreads` from the CPU count (capped at `MAX_THREADS`), allocates the image buffer (`MAX_W * MAX_H * 3` bytes, 640x360 RGB), dispatches the headless modes, and otherwise runs the window. The buffer is freed after the window loop returns.

---

## Constants

| Name | Value | Meaning |
|---|---|---|
| `PI` | 3.14159... | |
| `INF` | `1e30` | "No limit" distance |
| `EPS` | `1e-4` | Minimum hit distance (avoids self-hits) |
| `MAX_DEPTH` | 2 | Reflection bounce limit |
| `MAX_OBJ` | 512 | Object array capacity |
| `MAX_LAMP` | 16 | Lamp array capacity |
| `MAX_THREADS` | 64 | Thread cap |
| `MAX_W`, `MAX_H` | 640, 360 | Largest render size (image buffer) |
| `WALK_SPEED` | 5.0 | Walking speed, metres per second (run is 3x) |
| `MOUSE_SENS` | 0.1 | Degrees per pixel of mouse movement |
| `GRID_N` | 36 | Grid cells per side |
| `GRID_CELL` | 4.0 | Cell size, world units |
| `GRID_MIN` | -72.0 | Grid origin |
| `CELL_CAP` | 96 | Objects per grid cell |
| `GLOBAL_EXTENT` | 20.0 | Footprint size above which an object is global |

---

## Known limitations and notes

- **Fixed scene.** The city is hard-coded in `build_scene`. There is no file loader.
- **No anti-aliasing in use.** Window and benchmark frames use one sample per pixel. `render_row` supports `spp × spp` samples on a regular grid inside each pixel, so raising `spp` in the code gives anti-aliasing, but the program never sets it above 1.
- **Hard limits, no growth.** `MAX_OBJ`, `MAX_LAMP` and `CELL_CAP` are fixed. `add_obj` and `add_lamp` do not check capacity. A grid cell overflow exits the program. The grid is also fixed at 144 units square; objects outside it are clamped to edge cells.
- **Benchmark with zero frames.** `--bench 0` (or a non-number) gives `nan` output, because the average divides by zero.
- **Ray counter unused.** `nrays` is collected per thread but never printed.
- **Scene reference.** The comment on `build_scene` refers to `rt.c`, which is not in this repository.
- **Small memory issues.** `malloc` and `calloc` results are not checked. The image buffer is not freed on the headless paths; it is freed only after the window loop.
- **Platform notes.** The header defines `_POSIX_C_SOURCE` and `_DARWIN_C_SOURCE` for POSIX and macOS builds. On Windows it uses `windows.h` and `QueryPerformanceCounter`. The Makefile has separate MSYS2 paths for Windows (see the top of the Makefile).
- **What was checked.** On macOS with GLFW, the build command in the header comment compiles with `-Wall` and no warnings. `--snapshot` writes a 640x360 BMP, and `--bench 5` runs. `--bench 0` prints `nan`, as described above. The interactive window and the FreeGLUT backend were not run as part of writing this document.
