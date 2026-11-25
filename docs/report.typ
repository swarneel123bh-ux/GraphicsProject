// Project report for IS F311. Build from the project root:
//   typst compile docs/report.typ report.pdf

#set document(title: "Real-Time CPU Ray Tracing of a Lit City Block", author: ("Swarneel Bhattacharjee", "Siddharth Sastri"))
#set page(paper: "a4", margin: (x: 2.2cm, y: 2.2cm), numbering: "1")
#set text(font: "Libertinus Serif", size: 10.5pt, lang: "en")
#set par(justify: true, leading: 0.62em)
#set heading(numbering: "1.1")
#show heading.where(level: 1): it => block(above: 1.4em, below: 0.8em, text(size: 13.5pt, it))
#show heading.where(level: 2): it => block(above: 1.1em, below: 0.6em, text(size: 11.5pt, it))
#show raw: set text(font: "DejaVu Sans Mono", size: 8.6pt)
#show raw.where(block: true): it => block(fill: rgb("#f3f1ec"), inset: 8pt, radius: 3pt, width: 100%, it)
#set math.equation(numbering: "(1)")
#show figure.caption: set text(size: 9pt)
#set table(stroke: 0.5pt + rgb("#c9c4b8"), inset: 5pt)
#show table.cell.where(y: 0): set text(weight: "bold")

#align(center)[
  #text(size: 9.5pt, tracking: 0.5pt)[IS F311 COMPUTER GRAPHICS · BITS PILANI, HYDERABAD CAMPUS · PROJECT REPORT]
  #v(6pt)
  #text(size: 18pt, weight: "bold")[Real-Time CPU Ray Tracing of a Lit City Block \ with Fast Grid Traversal]
  #v(8pt)
  Swarneel Bhattacharjee (2023AAPS0695H) #h(1.2em) Siddharth Sastri (2023B5A70230H)
]

#v(4pt)
#block(inset: (x: 1.2cm))[
  #set text(size: 9.5pt)
  *Abstract.* We implement a Whitted-style ray tracer in C that renders a procedurally generated city block on the CPU at
  interactive frame rates. The technique studied from the literature is the fast voxel traversal algorithm of Amanatides
  and Woo @amanatides, applied in two dimensions over a uniform grid @arts. The renderer supports boxes, spheres, cylinders
  and cones; Lambertian lighting from a directional sun and four point lamps; hard shadows; and recursive mirror
  reflections. Image rows are distributed dynamically over POSIX threads. On an 8-core Apple M3 the program renders
  480 × 270 frames at about 34 frames per second, and grid traversal is 3.65 times faster than testing every object.
  This report gives the mathematics of each algorithm, its implementation, and measured results.
]

= Introduction

Ray tracing computes an image by following rays from a virtual camera into the scene. Its main attraction is that
shadows, reflections and occlusion follow directly from geometry @whitted @shirley. Its main cost is the visibility query: a naive
tracer tests every ray against every object, which costs $O(N)$ per ray for $N$ objects and is too slow for interactive
use.

The aim of this project was to learn and implement a technique not covered in class. We chose the fast voxel traversal
algorithm of Amanatides and Woo @amanatides, which walks a ray through a uniform grid using only additions and
comparisons, so each ray examines only the objects near its path. The program (`scratch_raytracer/live.c`, one C11 file)
renders a city of 71 objects in a window that the user walks through with the mouse and keyboard, and has headless
modes for producing images and benchmarks.

Section 2 describes the camera model, Section 3 the ray–primitive intersection tests, Section 4 the grid and its
traversal, Section 5 the shading model, Section 6 parallel rendering, and Section 7 the results.

= Camera and primary rays

The camera stores a position $bold(e)$, a yaw $psi$ and pitch $theta$ (in degrees) and a vertical field of view $phi$. Yaw 0
looks along $+z$. The orthonormal view basis is

$ bold(f) = (sin psi cos theta, space sin theta, space cos psi cos theta), quad
  bold(r) = (bold(y) times bold(f)) / norm(bold(y) times bold(f)), quad
  bold(u) = bold(f) times bold(r), $

where $bold(y) = (0, 1, 0)$. Pitch is limited to $plus.minus 89°$ so that $bold(y) times bold(f)$ never vanishes. For an image of
$W times H$ pixels with aspect ratio $a = W slash H$ and $t_phi = tan(phi slash 2)$, the sample at sub-pixel offset
$(s_x, s_y) in [0, 1)^2$ of pixel $(x, y)$ has normalised coordinates $p_x = (x + s_x) slash W$, $p_y = (y + s_y) slash H$ and
direction

$ bold(d) = "normalize"(bold(f) + (2 p_x - 1) a t_phi bold(r) + (1 - 2 p_y) t_phi bold(u)). $ <eq:primary>

With $n times n$ supersampling the offsets are the regular grid $s = (k + 0.5) slash n$, $k = 0, dots, n-1$, and the
pixel colour is the mean of the $n^2$ samples. The interactive window uses $n = 1$.

= Ray–primitive intersection

A ray is $bold(p)(t) = bold(o) + t bold(d)$ with $norm(bold(d)) = 1$. Every test returns the smallest root
$t in [epsilon, t_"max")$, where $epsilon = 10^(-4)$ rejects self-intersections and $t_"max"$ is the nearest hit found so far
(or the distance to a light for shadow rays), together with the unit surface normal.

== Axis-aligned box (slab method)

A box $[bold(l), bold(h)]$ is the intersection of three slabs @kay. For each axis $i$ with $d_i != 0$:

$ t_(1,i) = (l_i - o_i) / d_i, quad t_(2,i) = (h_i - o_i) / d_i, quad
  t_"near" = max_i min(t_(1,i), t_(2,i)), quad t_"far" = min_i max(t_(1,i), t_(2,i)). $

The ray hits the box when $t_"near" <= t_"far"$ and $epsilon <= t_"near" < t_"max"$. The axis that produced $t_"near"$ is
the face entered, and the normal is $minus "sign"(d_i)$ along that axis. When $|d_i| < 10^(-12)$ the ray is parallel to the
slab; the code then rejects the box if $o_i in.not [l_i, h_i]$ and skips the axis. This explicit test avoids the
$0 dot infinity$ cases that make naive IEEE implementations fragile @williams.

== Sphere

For centre $bold(c)$ and radius $r$, let $bold(m) = bold(o) - bold(c)$. Because $norm(bold(d)) = 1$,
$norm(bold(m) + t bold(d))^2 = r^2$ reduces to

$ t^2 + 2 b t + c = 0, quad b = bold(m) dot bold(d), quad c = bold(m) dot bold(m) - r^2, quad
  t = -b minus.plus sqrt(b^2 - c). $

The nearer root is taken unless it is below $epsilon$ (origin inside the sphere), in which case the farther root is used.
The normal is $(bold(p)(t) - bold(c)) slash r$.

== Vertical cylinder

A cylinder with axis $(c_x, c_z)$, radius $r$ and height range $[y_0, y_1]$ is handled in two parts. With
$o'_x = o_x - c_x$, $o'_z = o_z - c_z$, the side satisfies $(o'_x + t d_x)^2 + (o'_z + t d_z)^2 = r^2$:

$ A t^2 + B t + C = 0, quad A = d_x^2 + d_z^2, quad B = 2(o'_x d_x + o'_z d_z), quad C = o'^2_x + o'^2_z - r^2. $

Each root is accepted if $y = o_y + t d_y in [y_0, y_1]$; the normal is the horizontal vector
$(o'_x + t d_x, 0, o'_z + t d_z) slash r$. The caps are the planes $y = y_0$ and $y = y_1$ at
$t = (y_c - o_y) slash d_y$, accepted when the hit lies inside the radius, with normals $minus bold(y)$ and $+bold(y)$.
The smallest valid $t$ of the four candidates wins.

== Vertical cone

The cone has its apex at height $y_1$ and a base of radius $r$ at $y_0$, so its radius at height $y$ is $k(y_1 - y)$ with
$k = r slash (y_1 - y_0)$. Substituting the ray, with $a = y_1 - o_y$:

$ (o'_x + t d_x)^2 + (o'_z + t d_z)^2 = k^2 (a - t d_y)^2, $

$ A = d_x^2 + d_z^2 - k^2 d_y^2, quad B = 2(o'_x d_x + o'_z d_z + k^2 a d_y), quad C = o'^2_x + o'^2_z - k^2 a^2. $

When $|A| approx 0$ the ray is parallel to a generator line and the equation is linear, $t = -C slash B$. Roots outside
$[y_0, y_1]$ are rejected (this also removes the mirror-image cone above the apex). The normal is the gradient of
$F = x^2 + z^2 - k^2 (y_1 - y)^2$, i.e. $(x, space k^2 (y_1 - y), space z)$ normalised. The base disc is tested like a
cylinder cap.

= Acceleration: uniform grid and fast voxel traversal

== The grid

The city is almost flat, so space is divided into a 2D grid of vertical columns over the $x$–$z$ plane @arts:
$G = 36$ cells per side, cell size $s = 4$ m, covering $[-72, 72]$ m on both axes. The cell of coordinate $v$ is
$"clamp"(floor((v - x_"min") slash s), 0, G - 1)$. Each object is entered into every cell its horizontal bounding
rectangle overlaps. Objects whose footprint exceeds 20 m (the 140 m ground) are kept in a separate global list that is
tested for every ray, instead of being copied into all 1296 cells. Each cell stores at most 96 indices; with 71 objects the
fullest cell holds far fewer.

== Traversal (Amanatides and Woo)

The key idea of @amanatides is to visit the cells pierced by a ray in order, with constant work per cell. For each axis
$i in {x, z}$ the algorithm keeps:

- $"step"_i = "sign"(d_i)$, the change in cell index along that axis;
- $t_("max",i)$, the ray parameter at which the ray crosses the next cell boundary on axis $i$;
- $t_(Delta,i) = s slash |d_i|$, the parameter increase needed to cross one full cell on axis $i$.

*Initialisation.* The ray is first clipped to the grid square with a 2D slab test, giving the entry parameter
$t_0 = max(0, t_"enter")$ and exit $t_"exit"$; if $t_0 > t_"exit"$ the ray misses the grid. The entry point
$bold(q) = bold(o) + t_0 bold(d)$ gives the start cell $(i_x, i_z)$. The next boundary on axis $x$ is
$b_x = x_"min" + (i_x + 1) s$ when $d_x > 0$ and $x_"min" + i_x s$ otherwise, so

$ t_("max",x) = t_0 + (b_x - q_x) / d_x, $

and similarly for $z$. Axes with $d_i approx 0$ get $t_("max",i) = t_(Delta,i) = infinity$.

*Incremental step.* Each iteration tests the objects in the current cell, keeping the nearest hit $t_"hit"$. Let
$t_"next" = min(t_("max",x), t_("max",z))$, the parameter at which the ray leaves the cell. Then:

```
loop:
    test every object in cell (ix, iz); t_hit = nearest hit so far
    t_next = min(tMaxX, tMaxZ)
    if t_next >= t_hit or t_next > t_exit: stop
    if tMaxX < tMaxZ:  ix += stepX;  tMaxX += tDeltaX
    else:              iz += stepZ;  tMaxZ += tDeltaZ
    if (ix, iz) is outside the grid: stop
```

Only one comparison and one addition are needed per cell.

*Correct early termination.* An object can span several cells, so a hit found while testing cell $k$ may lie in a later
cell, behind an object in that later cell that has not been tested yet. Stopping at the first hit would then return the
wrong object. The loop therefore stops only when the next cell begins beyond the best hit
($t_"next" >= t_"hit"$): at that point no untested cell can contain a closer surface. The same condition makes the
paper's "mailbox" optional: an object tested twice is simply found again at the same distance. We do not implement
mailboxing.

#figure(image("img/grid_traversal.svg", width: 72%),
  caption: [Grid traversal seen from above. The ray visits cells 1–7 in order and hits an object in cell 7; the next
  boundary lies beyond the hit, so the walk stops.]) <fig:grid>

*Shadow queries.* The same routine has an "any hit" mode used for shadow rays: it returns as soon as any non-emissive
object is hit before the light ($t < t_"max"$ = light distance). Emissive objects (the lamp bulbs) are skipped so that
a lamp is not blocked by its own bulb.

*Cost.* A ray crosses at most $2G = 72$ cells in a 2D grid of $G times G$ cells, and usually far fewer because it stops at
the first hit. The per-ray cost becomes proportional to the number of cells crossed plus the objects stored in them,
instead of the total number of objects.

= Shading

== Illumination model

For a hit point $bold(p)$ with normal $bold(n)$ (flipped to face the ray if $bold(n) dot bold(d) > 0$) on a surface of
colour $bold(c)$, the program evaluates the ambient and diffuse terms of the Phong model @phong:

$ bold(C) = bold(c) dot.o bold(A)
  + V_"sun" k_"sun" (bold(c) dot.o bold(S)) max(0, bold(n) dot bold(l)_"sun")
  + sum_i V_i k_i (bold(c) dot.o bold(L)_i) (max(0, bold(n) dot bold(l)_i)) / (1 + 0.08 d_i^2), $ <eq:shade>

where $dot.o$ is the component-wise product, $bold(A)$ the ambient colour, $bold(S)$ and $bold(L)_i$ the light colours,
$bold(l)$ unit vectors to the lights, $d_i$ the distance to lamp $i$, and $V in {0, 1}$ the visibility from a shadow ray.
The sun contributes only while it is above the horizon ($l_(y) > 0.02$). Emissive surfaces return
$bold(c) dot e$ directly, without lighting.

== Shadows

Shadow rays start from $bold(p) + 10^(-3) bold(n)$ rather than $bold(p)$. Without this offset, rounding error places the
origin slightly below the surface and the ray hits the surface it started from ("shadow acne"). Sun rays test the whole
scene; lamp rays test only up to the lamp's distance.

== Reflection

For a surface with reflectivity $rho > 0$ the mirror direction is $bold(d)' = bold(d) - 2 (bold(d) dot bold(n)) bold(n)$,
and the colour becomes

$ bold(C)' = (1 - rho) bold(C) + rho dot "trace"(bold(p) + 10^(-3) bold(n), bold(d)', "depth" - 1), $

with recursion limited to two bounces. The ground ($rho = 0.12$), buildings ($rho = 0.05$), sphere ($rho = 0.3$) and lamp
posts ($rho = 0.1$) are reflective.

== Sky, sun and output

A ray that hits nothing returns a vertical gradient $"mix"(bold(K)_"hor", bold(K)_"top", "clamp"((d_y + 1) slash 2))$.
The sun direction is $"normalize"(-0.7, sin alpha, 0.35)$ for elevation $alpha$; from its height $s_y$ the program derives
a daylight factor $"clamp"(3 s_y, 0, 1)$ that sets the ambient level, sky colours and lamp intensity
$k_i = 1.2 + 2.5(1 - "daylight")$. The scene uses a fixed $alpha = 1$ rad. Each colour channel is clamped to $[0, 1]$ and
gamma corrected, $v' = v^(1 slash 2.2)$, before conversion to 8 bits.

= Parallel rendering

Pixels are independent, so rows are rendered concurrently. `render_frame` starts one POSIX thread per online core (at
most 64). The threads share a row counter protected by a mutex: each thread locks, takes the next row, unlocks, renders the
row and repeats until no rows are left. Rows are written to disjoint parts of the image, so no other synchronisation is
needed. Dynamic hand-out balances the load because row costs differ widely: sky rows need one ray per pixel, while rows of
reflective street need primary, shadow and reflected rays. The lock is taken once per row (360 times for a 640 × 360
frame), which is negligible next to the rendering work.

The rendered buffer is uploaded to an OpenGL texture and drawn on a full-window quad. The window layer is abstracted so
that either GLFW or FreeGLUT can be used, chosen at compile time.

= Scene

`build_scene` creates 71 objects and 4 lamps: a 140 m × 140 m ground box, 28 thin boxes forming the dashed centre lines of
two crossing roads, 33 buildings (a 6 × 6 layout of 12 m blocks, each left empty with probability 0.2, 7 m wide and 4–20 m
tall), a red reflective sphere at the crossing, and four lamp posts (cylinders) with emissive bulbs and a point light under
each. A linear congruential generator with a fixed seed makes the city identical on every run.

= Results

All measurements were taken on an Apple M3 (4 performance and 4 efficiency cores) under macOS, compiled with
`cc -O2`. Times are averages of 10 frames unless stated.

#figure(
  grid(columns: 2, gutter: 6pt,
    image("img/street.png"), image("img/aerial.png"),
    image("img/sphere.png"), image("img/lamp.png")),
  caption: [Renders at 1280 × 720 with 2 × 2 samples per pixel. Top: start view; aerial view of the 33 buildings.
  Bottom: the reflective sphere (two levels of reflection and a sun shadow); a lamp post and its shadow.])

#figure(
  table(columns: 3, align: (left, right, right),
    [Method (640 × 360, 8 threads)], [Time per frame], [Frame rate],
    [Test every object], [198.7 ms], [5.0 FPS],
    [Grid traversal], [54.4 ms], [18.4 FPS]),
  caption: [Effect of grid traversal. The grid is 3.65 times faster with only 71 objects.]) <tab:grid>

#figure(
  table(columns: 3, align: (right, right, right),
    [Threads], [Time per frame (640 × 360)], [Speed-up],
    [1], [222.0 ms], [1.00×],
    [2], [112.8 ms], [1.97×],
    [4], [63.5 ms], [3.50×],
    [8], [53.7 ms], [4.13×]),
  caption: [Thread scaling. Near-linear up to the 4 performance cores; the 4 efficiency cores add little.]) <tab:threads>

#figure(
  table(columns: 4, align: (left, right, right, right),
    [Render size], [320 × 180], [480 × 270], [640 × 360],
    [Time per frame], [14.3 ms], [30.9 ms], [54.1 ms],
    [Frame rate], [≈ 70 FPS], [≈ 32 FPS], [≈ 18 FPS]),
  caption: [Interactive render sizes (1 sample per pixel, 8 threads). The built-in benchmark at 480 × 270 gives about
  34 FPS while walking down the road.]) <tab:fps>

Supersampling cost is proportional to the number of samples: 54.4 ms for 1, 209.1 ms for 4 and 467.2 ms for 9 samples
per pixel at 640 × 360.

*Discussion.* @tab:grid confirms the benefit claimed in @amanatides: even in a small scene, walking the grid removes most
intersection tests, and the gain grows with scene size because brute force is linear in $N$ while the grid walk is not.
Thread scaling (@tab:threads) shows that the row scheduler is not a bottleneck. Together they make 30+ FPS possible on a
laptop CPU.

= Limitations and future work

- *Day–night cycle.* The proposal included an animated day–night cycle. The final program lights the scene at a fixed
  sun angle; `update_light` supports any angle but nothing animates it.
- *Fixed capacities.* Grid cells hold at most 96 objects (the program exits if exceeded) and the object and lamp arrays are
  not bounds-checked.
- *Image quality.* One sample per pixel in the window causes aliasing; there is no specular term, soft shadow, fog or tone
  mapping; and the small reflectivity of every wall makes buildings look slightly translucent.
- *Interaction.* The camera has no collision detection.
- *Future work.* Mailboxing and a 3D grid or bounding volume hierarchy for non-flat scenes; SIMD or GPU tracing;
  Blinn–Phong highlights, soft shadows and fog; animating the sun to restore the day–night cycle.

= Conclusion

A Whitted-style CPU ray tracer becomes interactive once visibility queries are accelerated. Implementing the 2D form of
the Amanatides–Woo traversal took about 70 lines and gave a 3.65× speed-up on a 71-object scene; combined with dynamic row
scheduling over 8 cores, the program reaches about 34 FPS at 480 × 270 with shadows and reflections.

#bibliography("report.bib", title: "References", style: "ieee")
