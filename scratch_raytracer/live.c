/**
 * @file live.c
 * @brief Walk around a city block while it is ray traced on the CPU (IS F311 Computer Graphics project).
 *
 * @details
 *
 * Window and controls follow the Assignment 2 app (GLFW, mouse look, WASD). The image is traced
 * on the CPU with pthreads, one row at a time, and shown as a texture on a full-window quad.
 *
 * Build: use the Makefile in the project root (make). It uses GLFW or FreeGLUT, whichever is installed.
 * By hand, pass -DUSE_GLFW (GLFW 3) or -DUSE_FREEGLUT (FreeGLUT):
 *   macOS (GLFW):      cc -std=c11 -O2 -pthread -DUSE_GLFW -I$(brew --prefix glfw)/include -o live live.c -L$(brew --prefix glfw)/lib -lglfw -framework OpenGL -lm
 *   Linux (GLFW):      cc -std=c11 -O2 -pthread -DUSE_GLFW -o live live.c -lglfw -lGL -lm
 *   Linux (FreeGLUT):  cc -std=c11 -O2 -pthread -DUSE_FREEGLUT -o live live.c -lglut -lGL -lm
 *   Windows (GLFW):    gcc -std=c11 -O2 -pthread -DUSE_GLFW -o live.exe live.c -lglfw -lopengl32 -lgdi32 -lm   (MSYS2 MINGW64 shell)
 * Run:
 *   ./live                        interactive window
 *   ./live --snapshot out.bmp     render one frame at 640x360, no window
 *   ./live --bench 60             walk 60 frames along a fixed path, print FPS, no window
 *
 * Controls: mouse look, W/A/S/D walk and strafe, Shift run, scroll or +/- zoom, R reset,
 *           1/2/3 render size 320x180 / 480x270 / 640x360, Esc quit.
 */
#ifdef __APPLE__
#define _DARWIN_C_SOURCE
#endif
#if !defined(_WIN32)
#define _POSIX_C_SOURCE 200809L
#endif
#if defined(USE_GLFW)
#include <GLFW/glfw3.h>
#elif defined(USE_FREEGLUT)
#include <GL/freeglut.h>
#else
#error "Build with -DUSE_GLFW or -DUSE_FREEGLUT (the Makefile sets this for you)"
#endif
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <time.h>
#include <unistd.h>
#endif

/** @name Constants
 *  General limits and tuning values. @{ */
#define PI 3.14159265358979323846   /**< Pi, for degree/radian conversion. */
#define INF 1e30                    /**< "Infinite" ray distance. */
#define EPS 1e-4                    /**< Smallest accepted hit distance (avoids self-hits). */
#define MAX_DEPTH 2                 /**< Maximum number of mirror bounces. */
#define MAX_OBJ 512                 /**< Capacity of the object array. */
#define MAX_LAMP 16                 /**< Capacity of the lamp array. */
#define MAX_THREADS 64              /**< Most render threads started. */
#define MAX_W 640                   /**< Largest render width (size of the frame buffer). */
#define MAX_H 360                   /**< Largest render height. */

#define WALK_SPEED 5.0      /**< Walking speed in metres per second; Shift runs 3x. */
#define MOUSE_SENS 0.1      /**< Mouse-look sensitivity in degrees per pixel. */
/** @} */

/* ---------- vector maths ---------- */
/** @brief A 3D vector, also used for RGB colours (x, y, z = r, g, b). */
typedef struct { double x, y, z; } V3;
/** @brief Builds a vector from its three components. */
static V3 V(double x, double y, double z) { V3 r = {x, y, z}; return r; }
/** @brief Component-wise sum a + b. */
static V3 add(V3 a, V3 b) { return V(a.x + b.x, a.y + b.y, a.z + b.z); }
/** @brief Component-wise difference a - b. */
static V3 sub(V3 a, V3 b) { return V(a.x - b.x, a.y - b.y, a.z - b.z); }
/** @brief Scales vector a by s. */
static V3 scl(V3 a, double s) { return V(a.x * s, a.y * s, a.z * s); }
/** @brief Component-wise product (used to tint colours). */
static V3 mul(V3 a, V3 b) { return V(a.x * b.x, a.y * b.y, a.z * b.z); }
/** @brief Dot product a . b. */
static double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
/** @brief Cross product a x b. */
static V3 cross(V3 a, V3 b) { return V(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
/** @brief Returns a scaled to unit length. */
static V3 norm(V3 a) { return scl(a, 1.0 / sqrt(dot(a, a))); }
/** @brief Returns -a. */
static V3 neg(V3 a) { return V(-a.x, -a.y, -a.z); }
/** @brief Linear interpolation: a at t = 0, b at t = 1. */
static V3 mix(V3 a, V3 b, double t) { return add(scl(a, 1.0 - t), scl(b, t)); }

/* ---------- objects ---------- */
/** @brief Primitive types stored in Obj::type. */
enum { BOX, SPHERE, CYL, CONE };

/** @brief One primitive in the scene. */
typedef struct {
    int type;     /**< BOX, SPHERE, CYL or CONE. */
    V3 lo, hi;    /**< BOX: min/max corners. SPHERE: lo = centre. CYL/CONE: lo = (cx, y0, cz), hi.y = y1. */
    double r;     /**< Radius (sphere, cylinder, cone base). */
    V3 col;       /**< Diffuse colour. */
    double refl;  /**< Mirror amount, 0 (matte) to 1 (perfect mirror). */
    double emis;  /**< Emission; > 0 means self-lit (lamp bulbs), not shaded and ignored by shadow rays. */
} Obj;

/** @brief A point light: position, colour and intensity k. */
typedef struct { V3 pos; V3 col; double k; } Lamp;

/** @brief All objects in the scene. */
static Obj objs[MAX_OBJ];
/** @brief Number of objects in use in objs. */
static int nobj = 0;
/** @brief All point lights (lamp posts). */
static Lamp lamps[MAX_LAMP];
/** @brief Number of lamps in use. */
static int nlamp = 0;
/** @brief Sun direction, sun colour, sky zenith and horizon colours, and ambient light (set by update_light()). */
static V3 sun_dir, sun_col, sky_top, sky_hor, ambient;
/** @brief Sun strength and daylight factor 0..1 (set by update_light()). */
static double sun_k = 1.0, daylt = 1.0;
static _Thread_local long long nrays = 0;   /**< Rays traced by the current thread. */

/**
 * @brief Appends an object to the scene.
 * @param type BOX, SPHERE, CYL or CONE.
 * @param lo,hi Geometry, as described in Obj::lo.
 * @param r Radius (ignored for boxes).
 * @param col Diffuse colour.
 * @param refl Mirror amount 0..1.
 * @param emis Emission (> 0 for light bulbs).
 * @return Index of the new object.
 * @warning Does not check MAX_OBJ.
 */
static int add_obj(int type, V3 lo, V3 hi, double r, V3 col, double refl, double emis) {
    Obj *o = &objs[nobj];
    o->type = type; o->lo = lo; o->hi = hi; o->r = r;
    o->col = col; o->refl = refl; o->emis = emis;
    return nobj++;
}

/**
 * @brief Adds a point light.
 * @param pos Light position.
 * @param col Light colour.
 * @param k Intensity (changed again by update_light()).
 */
static void add_lamp(V3 pos, V3 col, double k) {
    lamps[nlamp].pos = pos; lamps[nlamp].col = col; lamps[nlamp].k = k;
    nlamp++;
}

/* ---------- intersections (directions normalised; tmax = nearest accepted t) ---------- */
/**
 * @brief Ray / axis-aligned box intersection by the slab method (Kay and Kajiya).
 *
 * The ray is intersected with the pair of planes on each axis; the box is hit when the three
 * [near, far] intervals overlap. The axis of the last entering plane gives the normal.
 * @param o Ray origin.
 * @param d Normalised ray direction.
 * @param lo,hi Minimum and maximum corners.
 * @param tmax Only hits closer than this are accepted.
 * @param[out] t Distance to the hit.
 * @param[out] n Outward normal at the hit.
 * @return 1 on a hit, 0 otherwise.
 */
static int hit_box(V3 o, V3 d, V3 lo, V3 hi, double tmax, double *t, V3 *n) {
    double oa[3] = {o.x, o.y, o.z}, da[3] = {d.x, d.y, d.z};
    double la[3] = {lo.x, lo.y, lo.z}, ha[3] = {hi.x, hi.y, hi.z};
    double tn = -INF, tf = INF, sg = 0;
    int ax = -1;
    /* tn/tf: latest entry and earliest exit over the three slabs; ax remembers the entry axis */
    for (int i = 0; i < 3; i++) {
        if (fabs(da[i]) < 1e-12) {
            if (oa[i] < la[i] || oa[i] > ha[i]) return 0;
            continue;
        }
        double inv = 1.0 / da[i];
        double t1 = (la[i] - oa[i]) * inv, t2 = (ha[i] - oa[i]) * inv;
        double near_t = t1 < t2 ? t1 : t2, far_t = t1 < t2 ? t2 : t1;
        if (near_t > tn) { tn = near_t; ax = i; sg = (t1 < t2) ? -1.0 : 1.0; }
        if (far_t < tf) tf = far_t;
    }
    if (ax < 0 || tn > tf || tn < EPS || tn >= tmax) return 0;
    double nn[3] = {0, 0, 0};
    nn[ax] = sg;
    *t = tn;
    *n = V(nn[0], nn[1], nn[2]);
    return 1;
}

/**
 * @brief Ray / sphere intersection.
 *
 * Solves |o + t d - c|^2 = r^2. Because d has unit length the quadratic reduces to
 * t^2 + 2bt + cc = 0. The nearer root is used unless it is behind the origin.
 * @param o Ray origin. @param d Normalised direction. @param c Centre. @param r Radius.
 * @param tmax Upper distance limit. @param[out] t Hit distance. @param[out] n Unit normal.
 * @return 1 on a hit, 0 otherwise.
 */
static int hit_sphere(V3 o, V3 d, V3 c, double r, double tmax, double *t, V3 *n) {
    V3 oc = sub(o, c);
    double b = dot(oc, d), cc = dot(oc, oc) - r * r;
    double disc = b * b - cc;
    if (disc < 0) return 0;
    double s = sqrt(disc);
    double tt = -b - s;
    if (tt < EPS) tt = -b + s;
    if (tt < EPS || tt >= tmax) return 0;
    *t = tt;
    *n = norm(sub(add(o, scl(d, tt)), c));
    return 1;
}

/**
 * @brief Ray / vertical capped cylinder intersection.
 *
 * The side is a quadratic in x and z only, followed by a height check against [y0, y1];
 * the top and bottom caps are discs tested separately. The nearest valid hit wins.
 * @param o Ray origin. @param d Normalised direction. @param c Axis position (x, z used).
 * @param y0,y1 Bottom and top heights. @param r Radius. @param tmax Upper distance limit.
 * @param[out] t Hit distance. @param[out] n Unit normal.
 * @return 1 on a hit, 0 otherwise.
 */
static int hit_cyl(V3 o, V3 d, V3 c, double y0, double y1, double r, double tmax, double *t, V3 *n) {
    double best = tmax, ox = o.x - c.x, oz = o.z - c.z;
    int hit = 0;
    V3 bn = V(0, 0, 0);
    /* side: (ox + t dx)^2 + (oz + t dz)^2 = r^2 */
    double A = d.x * d.x + d.z * d.z;
    if (A > 1e-12) {
        double B = 2.0 * (ox * d.x + oz * d.z), C = ox * ox + oz * oz - r * r;
        double disc = B * B - 4.0 * A * C;
        if (disc >= 0) {
            double s = sqrt(disc);
            for (int k = 0; k < 2; k++) {
                double tt = (-B + (k ? s : -s)) / (2.0 * A);
                if (tt < EPS || tt >= best) continue;
                double y = o.y + tt * d.y;
                if (y < y0 || y > y1) continue;
                best = tt;
                bn = norm(V(ox + tt * d.x, 0, oz + tt * d.z));
                hit = 1;
            }
        }
    }
    /* caps: planes y = y0 and y = y1, accepted inside the radius */
    if (fabs(d.y) > 1e-12) {
        for (int k = 0; k < 2; k++) {
            double yc = k ? y1 : y0;
            double tt = (yc - o.y) / d.y;
            if (tt < EPS || tt >= best) continue;
            double px = ox + tt * d.x, pz = oz + tt * d.z;
            if (px * px + pz * pz > r * r) continue;
            best = tt;
            bn = V(0, k ? 1.0 : -1.0, 0);
            hit = 1;
        }
    }
    if (hit) { *t = best; *n = bn; }
    return hit;
}

/**
 * @brief Ray / vertical cone intersection (apex at y1, base of radius r at y0).
 *
 * The radius at height y is k (y1 - y), with k = r / (y1 - y0). Substituting the ray gives a
 * quadratic (or a linear equation when the ray is parallel to the side), then a height check.
 * The base disc is tested separately.
 * @param o Ray origin. @param d Normalised direction. @param c Axis position (x, z used).
 * @param y0,y1 Base and apex heights. @param r Base radius. @param tmax Upper distance limit.
 * @param[out] t Hit distance. @param[out] n Unit normal.
 * @return 1 on a hit, 0 otherwise.
 */
static int hit_cone(V3 o, V3 d, V3 c, double y0, double y1, double r, double tmax, double *t, V3 *n) {
    double k = r / (y1 - y0), k2 = k * k;
    double best = tmax, ox = o.x - c.x, oz = o.z - c.z, a = y1 - o.y;
    int hit = 0;
    V3 bn = V(0, 0, 0);
    double A = d.x * d.x + d.z * d.z - k2 * d.y * d.y;
    double B = 2.0 * (ox * d.x + oz * d.z + k2 * a * d.y);
    double C = ox * ox + oz * oz - k2 * a * a;
    double roots[2];
    int nr = 0;
    if (fabs(A) > 1e-12) {
        double disc = B * B - 4.0 * A * C;
        if (disc >= 0) {
            double s = sqrt(disc);
            roots[nr++] = (-B - s) / (2.0 * A);
            roots[nr++] = (-B + s) / (2.0 * A);
        }
    } else if (fabs(B) > 1e-12) {
        roots[nr++] = -C / B;
    }
    for (int i = 0; i < nr; i++) {
        double tt = roots[i];
        if (tt < EPS || tt >= best) continue;
        double y = o.y + tt * d.y;
        if (y < y0 || y > y1) continue;
        double px = ox + tt * d.x, pz = oz + tt * d.z;
        best = tt;
        bn = norm(V(px, k2 * (y1 - y), pz));
        hit = 1;
    }
    /* base disc at y = y0 */
    if (fabs(d.y) > 1e-12) {
        double tt = (y0 - o.y) / d.y;
        if (tt >= EPS && tt < best) {
            double px = ox + tt * d.x, pz = oz + tt * d.z;
            if (px * px + pz * pz <= r * r) { best = tt; bn = V(0, -1, 0); hit = 1; }
        }
    }
    if (hit) { *t = best; *n = bn; }
    return hit;
}

/**
 * @brief Dispatches to the intersection routine for the object's type.
 * @return 1 on a hit closer than tmax, with distance t and normal n.
 */
static int hit_obj(const Obj *ob, V3 o, V3 d, double tmax, double *t, V3 *n) {
    switch (ob->type) {
    case BOX:    return hit_box(o, d, ob->lo, ob->hi, tmax, t, n);
    case SPHERE: return hit_sphere(o, d, ob->lo, ob->r, tmax, t, n);
    case CYL:    return hit_cyl(o, d, ob->lo, ob->lo.y, ob->hi.y, ob->r, tmax, t, n);
    default:     return hit_cone(o, d, ob->lo, ob->lo.y, ob->hi.y, ob->r, tmax, t, n);
    }
}

/* ---------- 2D grid over the XZ plane (Amanatides and Woo, 1987) ----------
 * Each object is listed in every grid cell its XZ footprint touches. Objects with a huge
 * footprint (the ground) are tested for every ray. A ray walks the cells it passes through
 * (2D DDA) and stops once the nearest hit found so far is closer than the next cell. */
#define GRID_N 36            /**< Grid cells per side. */
#define GRID_CELL 4.0        /**< Cell size in metres. */
#define GRID_MIN -72.0       /**< World x and z of the grid's lower edge. */
#define CELL_CAP 96          /**< Most objects one cell can list. */
#define GLOBAL_EXTENT 20.0   /**< Objects wider than this go to the global list. */

static int cell_n[GRID_N * GRID_N];               /**< Number of objects in each cell. */
static int cell_obj[GRID_N * GRID_N][CELL_CAP];   /**< Object indices per cell. */
static int glob[MAX_OBJ];                         /**< Large objects tested for every ray. */
static int nglob = 0;                             /**< Number of entries in glob. */

/** @brief Grid cell index of world coordinate v, clamped to the grid. */
static int cell_of(double v) {
    int k = (int)floor((v - GRID_MIN) / GRID_CELL);
    if (k < 0) k = 0;
    if (k >= GRID_N) k = GRID_N - 1;
    return k;
}

/**
 * @brief Lists every object in each grid cell that its x-z footprint overlaps.
 *
 * Objects wider than GLOBAL_EXTENT (the ground) are put in the global list instead.
 * Exits the program if a cell would hold more than CELL_CAP objects.
 */
static void build_grid(void) {
    nglob = 0;
    memset(cell_n, 0, sizeof cell_n);
    for (int i = 0; i < nobj; i++) {
        Obj *ob = &objs[i];
        double x0, x1, z0, z1;
        if (ob->type == BOX) {
            x0 = ob->lo.x; x1 = ob->hi.x; z0 = ob->lo.z; z1 = ob->hi.z;
        } else if (ob->type == SPHERE) {
            x0 = ob->lo.x - ob->r; x1 = ob->lo.x + ob->r; z0 = ob->lo.z - ob->r; z1 = ob->lo.z + ob->r;
        } else {
            x0 = ob->lo.x - ob->r; x1 = ob->lo.x + ob->r; z0 = ob->lo.z - ob->r; z1 = ob->lo.z + ob->r;
        }
        if (x1 - x0 > GLOBAL_EXTENT || z1 - z0 > GLOBAL_EXTENT) {
            glob[nglob++] = i;
            continue;
        }
        for (int ix = cell_of(x0); ix <= cell_of(x1); ix++)
            for (int iz = cell_of(z0); iz <= cell_of(z1); iz++) {
                int c = ix * GRID_N + iz;
                if (cell_n[c] >= CELL_CAP) { fprintf(stderr, "grid cell full\n"); exit(1); }
                cell_obj[c][cell_n[c]++] = i;
            }
    }
}

/**
 * @brief Finds what a ray hits by walking the grid (2D form of Amanatides and Woo's voxel traversal).
 *
 * The global objects are tested first. The ray is then clipped to the grid square, and the cells it
 * crosses are visited in order: each step moves along the axis whose next boundary (tMax) is nearest
 * and adds that axis's tDelta. The walk stops when the next boundary is beyond the best hit so far,
 * which keeps the nearest hit correct even for objects that span several cells.
 * @param o Ray origin. @param d Normalised direction.
 * @param tmax Only hits closer than this count (light distance for shadow rays).
 * @param any 0: find the nearest hit. 1: return at the first hit, skipping emissive objects (shadow test).
 * @param[out] tout Hit distance (any == 0 only). @param[out] nout Hit normal (any == 0 only).
 * @return Index of the object hit, or -1.
 */
static int traverse(V3 o, V3 d, double tmax, int any, double *tout, V3 *nout) {
    double best = tmax;
    int found = -1;
    V3 bn = V(0, 0, 0);
    nrays++;

    for (int g = 0; g < nglob; g++) {
        int i = glob[g];
        if (any && objs[i].emis > 0) continue;
        double tt;
        V3 nn;
        if (hit_obj(&objs[i], o, d, best, &tt, &nn)) {
            if (any) return i;
            best = tt; found = i; bn = nn;
        }
    }

    /* global objects were tested above; now walk the grid */
    /* where the ray enters and leaves the grid square (slab test in x and z) */
    const double gmin = GRID_MIN, gmax = GRID_MIN + GRID_N * GRID_CELL;
    double t0 = 0, gexit = INF;
    double oa[2] = {o.x, o.z}, da[2] = {d.x, d.z};
    for (int a = 0; a < 2; a++) {
        if (fabs(da[a]) < 1e-12) {
            if (oa[a] < gmin || oa[a] > gmax) goto finish;
            continue;
        }
        double inv = 1.0 / da[a];
        double ta = (gmin - oa[a]) * inv, tb = (gmax - oa[a]) * inv;
        if (ta > tb) { double s = ta; ta = tb; tb = s; }
        if (ta > t0) t0 = ta;
        if (tb < gexit) gexit = tb;
        if (t0 > gexit) goto finish;
    }
    if (gexit > best) gexit = best;

    {
        /* start cell, step direction (sx, sz), tDelta (tdx, tdz) and tMax (tmx, tmz) as in the paper */
        double px = o.x + d.x * t0, pz = o.z + d.z * t0;
        int ix = cell_of(px), iz = cell_of(pz);
        int sx = d.x > 0 ? 1 : -1, sz = d.z > 0 ? 1 : -1;
        double tdx = fabs(d.x) > 1e-12 ? GRID_CELL / fabs(d.x) : INF;
        double tdz = fabs(d.z) > 1e-12 ? GRID_CELL / fabs(d.z) : INF;
        double bx = sx > 0 ? GRID_MIN + (ix + 1) * GRID_CELL : GRID_MIN + ix * GRID_CELL;
        double bz = sz > 0 ? GRID_MIN + (iz + 1) * GRID_CELL : GRID_MIN + iz * GRID_CELL;
        double tmx = fabs(d.x) > 1e-12 ? t0 + (bx - px) / d.x : INF;
        double tmz = fabs(d.z) > 1e-12 ? t0 + (bz - pz) / d.z : INF;

        for (;;) {
            int c = ix * GRID_N + iz;
            for (int k = 0; k < cell_n[c]; k++) {
                int i = cell_obj[c][k];
                if (any && objs[i].emis > 0) continue;
                double tt;
                V3 nn;
                if (hit_obj(&objs[i], o, d, best, &tt, &nn)) {
                    if (any) return i;
                    best = tt; found = i; bn = nn;
                }
            }
            double tex = tmx < tmz ? tmx : tmz;
            if (tex >= best || tex > gexit) break;
            if (tmx < tmz) { ix += sx; tmx += tdx; }
            else           { iz += sz; tmz += tdz; }
            if (ix < 0 || ix >= GRID_N || iz < 0 || iz >= GRID_N) break;
        }
    }

finish:
    if (any) return -1;
    if (found < 0) return -1;
    *tout = best;
    *nout = bn;
    return found;
}

/**
 * @brief Nearest hit along a ray.
 * @param o Ray origin. @param d Normalised direction.
 * @param[out] t Distance. @param[out] n Normal. @param[out] idx Object index.
 * @return 1 if something was hit.
 */
static int closest(V3 o, V3 d, double *t, V3 *n, int *idx) {
    double tt;
    V3 nn;
    int i = traverse(o, d, INF, 0, &tt, &nn);
    if (i < 0) return 0;
    *t = tt; *n = nn; *idx = i;
    return 1;
}

/** @brief Shadow test: 1 if any non-emissive object lies along the ray closer than maxdist. */
static int occluded(V3 o, V3 d, double maxdist) {
    double tt;
    V3 nn;
    return traverse(o, d, maxdist, 1, &tt, &nn) >= 0;
}

/* ---------- shading ---------- */
/** @brief Sky colour for a ray that hits nothing: horizon-to-zenith gradient. */
static V3 sky(V3 d) {
    double t = 0.5 * (d.y + 1.0);
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return mix(sky_hor, sky_top, t);
}

/**
 * @brief Whitted-style shading of one ray.
 *
 * Colour = ambient + sun (if not shadowed) + each lamp with 1 / (1 + 0.08 d^2) falloff (if not
 * shadowed), all Lambertian; then blended with a recursively traced mirror ray for reflective objects.
 * @param o Ray origin. @param d Normalised direction.
 * @param depth Remaining reflection bounces.
 * @return Linear RGB colour (not clamped).
 */
static V3 trace(V3 o, V3 d, int depth) {
    double t;
    V3 n;
    int idx;
    if (!closest(o, d, &t, &n, &idx)) return sky(d);
    Obj *ob = &objs[idx];
    if (ob->emis > 0) return scl(ob->col, ob->emis);

    V3 p = add(o, scl(d, t));
    if (dot(n, d) > 0) n = neg(n);
    V3 po = add(p, scl(n, 1e-3));

    V3 col = mul(ob->col, ambient);

    /* sun: directional, hard shadow, only when above the horizon */
    double ndl = dot(n, sun_dir);
    if (sun_dir.y > 0.02 && ndl > 0 && !occluded(po, sun_dir, INF))
        col = add(col, scl(mul(ob->col, sun_col), sun_k * ndl));

    /* lamp posts: point lights with distance falloff and hard shadows */
    for (int i = 0; i < nlamp; i++) {
        V3 to = sub(lamps[i].pos, p);
        double dist = sqrt(dot(to, to));
        V3 L = scl(to, 1.0 / dist);
        double l = dot(n, L);
        if (l <= 0) continue;
        if (occluded(po, L, dist)) continue;
        double att = 1.0 / (1.0 + 0.08 * dist * dist);
        col = add(col, scl(mul(ob->col, lamps[i].col), lamps[i].k * att * l));
    }

    /* mirror reflection */
    if (ob->refl > 0 && depth > 0) {
        V3 r = sub(d, scl(n, 2.0 * dot(d, n)));
        V3 rc = trace(po, norm(r), depth - 1);
        col = add(scl(col, 1.0 - ob->refl), scl(rc, ob->refl));
    }
    return col;
}

/** @brief Sun elevation in radians used for the scene (fixed time of day). */
#define SUN_ANGLE 1.0
/**
 * @brief Sets the sun, sky, ambient and lamp intensities for a sun elevation angle.
 *
 * A low sun gives warmer sunlight, a darker and bluer sky and brighter lamps.
 * @param a Sun elevation angle in radians.
 */
static void update_light(double a) {
    sun_dir = norm(V(-0.7, sin(a), 0.35));
    daylt = fmin(1.0, fmax(0.0, sun_dir.y * 3.0));
    sun_k = fmax(0.0, sun_dir.y) * 1.2;
    double warm = fmin(1.0, fmax(0.0, 1.0 - sun_dir.y * 2.5));
    sun_col = V(1.0, 0.9 - 0.2 * warm, 0.75 - 0.3 * warm);
    ambient = V(0.05 + 0.2 * daylt, 0.05 + 0.2 * daylt, 0.08 + 0.25 * daylt);

    V3 hor = mix(V(0.03, 0.03, 0.07), V(0.7, 0.8, 0.92), daylt);
    double dusk = fmax(0.0, 1.0 - fabs(sun_dir.y) * 4.0) * 0.8;
    sky_hor = mix(hor, V(0.95, 0.55, 0.32), dusk);
    sky_top = mix(V(0.0, 0.0, 0.03), V(0.25, 0.45, 0.9), daylt);

    for (int i = 0; i < nlamp; i++) lamps[i].k = 1.2 + 2.5 * (1.0 - daylt);
}

/* ---------- scene: the city block from rt.c ---------- */
static unsigned int seed = 12345u;   /**< Fixed seed, so the city is the same on every run. */
/** @brief Linear congruential random number in [0, 1]. */
static double frand(void) {
    seed = seed * 1103515245u + 12345u;
    return ((seed >> 8) & 0xFFFF) / 65535.0;
}

/**
 * @brief Creates the city: ground, road markings, buildings, the mirror sphere and four lamp posts.
 *
 * Then builds the grid and sets the lighting with update_light(SUN_ANGLE).
 */
static void build_scene(void) {
    add_obj(BOX, V(-70, -0.5, -70), V(70, 0, 70), 0, V(0.16, 0.16, 0.18), 0.12, 0);
    for (double x = -40; x < 40; x += 6)
        add_obj(BOX, V(x, 0, -0.1), V(x + 3, 0.01, 0.1), 0, V(0.85, 0.85, 0.8), 0, 0);
    for (double z = -40; z < 40; z += 6)
        add_obj(BOX, V(-0.1, 0, z), V(0.1, 0.01, z + 3), 0, V(0.85, 0.85, 0.8), 0, 0);
    for (int ix = -3; ix <= 2; ix++)
        for (int iz = -3; iz <= 2; iz++) {
            if (frand() < 0.2) continue;
            double cx = 12.0 * ix + 6.0, cz = 12.0 * iz + 6.0;
            double h = 4.0 + 16.0 * frand();
            V3 col = V(0.4 + 0.4 * frand(), 0.35 + 0.3 * frand(), 0.3 + 0.3 * frand());
            add_obj(BOX, V(cx - 3.5, 0, cz - 3.5), V(cx + 3.5, h, cz + 3.5), 0, col, 0.05, 0);
        }
    add_obj(SPHERE, V(0, 0.8, 0), V(0, 0, 0), 0.8, V(0.8, 0.15, 0.15), 0.3, 0);

    double lp[4][2] = {{2.2, -12}, {-2.2, 12}, {12, 2.2}, {-12, -2.2}};
    for (int i = 0; i < 4; i++) {
        double x = lp[i][0], z = lp[i][1];
        add_obj(CYL, V(x, 0, z), V(0, 4.8, 0), 0.1, V(0.2, 0.2, 0.22), 0.1, 0);
        add_obj(SPHERE, V(x, 5.2, z), V(0, 0, 0), 0.28, V(1.0, 0.85, 0.55), 0, 1.0);
        add_lamp(V(x, 4.6, z), V(1.0, 0.8, 0.5), 1.2);
    }
    build_grid();
    update_light(SUN_ANGLE);
}

/* ---------- camera (same conventions as Assignment 2: yaw 0 looks down +z) ---------- */
/** @brief First-person camera: position, yaw and pitch in degrees, vertical field of view in degrees. */
typedef struct { double x, y, z, yaw, pitch, fov; } Cam;
static Cam cam;   /**< The camera. */

/** @brief Puts the camera back at the start of the main road, looking along +z. */
static void cam_reset(void) {
    cam.x = 0; cam.y = 1.7; cam.z = -30;
    cam.yaw = 0; cam.pitch = -3; cam.fov = 60;
}

/** @brief Camera basis for one frame: eye, forward/right/up unit vectors, th = tan(fov / 2). */
typedef struct { V3 eye, fwd, right, up; double th; } View;

/** @brief Builds the View basis from the camera's yaw, pitch and field of view. */
static View current_view(void) {
    double yr = cam.yaw * PI / 180.0, pr = cam.pitch * PI / 180.0;
    View v;
    v.eye = V(cam.x, cam.y, cam.z);
    v.fwd = norm(V(sin(yr) * cos(pr), sin(pr), cos(yr) * cos(pr)));
    v.right = norm(cross(V(0, 1, 0), v.fwd));
    v.up = cross(v.fwd, v.right);
    v.th = tan(cam.fov * 0.5 * PI / 180.0);
    return v;
}

/* ---------- render: threads take one row at a time ---------- */
/** @brief Work shared by the render threads for one frame. */
typedef struct {
    int W, H, spp;          /**< Image size and samples per pixel side (spp x spp per pixel). */
    View vw;                /**< Camera for this frame. */
    unsigned char *img;     /**< Output RGB image, W x H x 3 bytes. */
    int next_row;           /**< Next row to hand out. */
    pthread_mutex_t lock;   /**< Protects next_row. */
} Job;

/** @brief Per-thread data: the shared job and the thread's ray count. */
typedef struct { Job *job; long long rays; } Worker;

/**
 * @brief Traces one image row: spp x spp regularly spaced samples per pixel, averaged,
 *        clamped and gamma corrected (1/2.2).
 */
static void render_row(Job *j, int y) {
    int n = j->spp;
    double aspect = (double)j->W / j->H;
    for (int x = 0; x < j->W; x++) {
        V3 acc = V(0, 0, 0);
        for (int sy = 0; sy < n; sy++)
            for (int sx = 0; sx < n; sx++) {
                double px = (x + (sx + 0.5) / n) / j->W;
                double py = (y + (sy + 0.5) / n) / j->H;
                double u = (2.0 * px - 1.0) * aspect * j->vw.th;
                double w = (1.0 - 2.0 * py) * j->vw.th;
                V3 d = norm(add(j->vw.fwd, add(scl(j->vw.right, u), scl(j->vw.up, w))));
                acc = add(acc, trace(j->vw.eye, d, MAX_DEPTH));
            }
        acc = scl(acc, 1.0 / (n * n));
        unsigned char *p = j->img + 3 * (y * j->W + x);
        double c[3] = {acc.x, acc.y, acc.z};
        for (int i = 0; i < 3; i++) {
            double v = c[i];
            if (v < 0) v = 0;
            if (v > 1) v = 1;
            v = pow(v, 1.0 / 2.2);
            p[i] = (unsigned char)(v * 255.0 + 0.5);
        }
    }
}

/** @brief Thread body: keeps taking the next free row until the image is done. */
static void *worker(void *arg) {
    Worker *w = (Worker *)arg;
    Job *j = w->job;
    nrays = 0;
    for (;;) {
        pthread_mutex_lock(&j->lock);
        int y = j->next_row < j->H ? j->next_row++ : -1;
        pthread_mutex_unlock(&j->lock);
        if (y < 0) break;
        render_row(j, y);
    }
    w->rays = nrays;
    return NULL;
}

/**
 * @brief Renders a whole image with nthreads threads that take rows one at a time.
 * @param img Output buffer (W x H x 3). @param W,H Image size. @param vw Camera.
 * @param spp Samples per pixel side. @param nthreads Number of threads (at most MAX_THREADS).
 */
static void render_frame(unsigned char *img, int W, int H, View vw, int spp, int nthreads) {
    Job j;
    memset(&j, 0, sizeof j);
    j.W = W; j.H = H; j.spp = spp; j.vw = vw; j.img = img; j.next_row = 0;
    pthread_mutex_init(&j.lock, NULL);

    pthread_t tid[MAX_THREADS];
    Worker wk[MAX_THREADS];
    for (int i = 0; i < nthreads; i++) {
        wk[i].job = &j;
        wk[i].rays = 0;
        if (pthread_create(&tid[i], NULL, worker, &wk[i]) != 0) {
            fprintf(stderr, "pthread_create failed\n");
            exit(1);
        }
    }
    for (int i = 0; i < nthreads; i++) pthread_join(tid[i], NULL);
    pthread_mutex_destroy(&j.lock);
}

/* ---------- timing ---------- */
/** @brief Monotonic time in milliseconds. */
static double now_ms(void) {
#if defined(_WIN32)
    LARGE_INTEGER freq, count;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&count);
    return (double)count.QuadPart * 1000.0 / (double)freq.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
#endif
}

/** @brief Number of online CPU cores (used as the thread count). */
static int cpu_count(void) {
#if defined(_WIN32)
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    return (int)si.dwNumberOfProcessors;
#else
    long n = sysconf(_SC_NPROCESSORS_ONLN);
    return n > 0 ? (int)n : 1;
#endif
}

/* ---------- BMP output (for --snapshot) ---------- */
/** @brief Writes v as 4 little-endian bytes. */
static void put32(unsigned char *p, unsigned int v) {
    p[0] = v & 255; p[1] = (v >> 8) & 255; p[2] = (v >> 16) & 255; p[3] = (v >> 24) & 255;
}
/** @brief Writes v as 2 little-endian bytes. */
static void put16(unsigned char *p, unsigned int v) { p[0] = v & 255; p[1] = (v >> 8) & 255; }

/**
 * @brief Saves an RGB image as an uncompressed 24-bit BMP file.
 * @param path File name. @param w,h Image size. @param rgb Top-down RGB pixels.
 * @return 1 on success, 0 if the file could not be opened.
 */
static int write_bmp(const char *path, int w, int h, const unsigned char *rgb) {
    int row = (w * 3 + 3) & ~3;
    unsigned int size = 54u + (unsigned int)row * (unsigned int)h;
    unsigned char hdr[54];
    memset(hdr, 0, sizeof hdr);
    hdr[0] = 'B'; hdr[1] = 'M';
    put32(hdr + 2, size); put32(hdr + 10, 54); put32(hdr + 14, 40);
    put32(hdr + 18, (unsigned int)w); put32(hdr + 22, (unsigned int)h);
    put16(hdr + 26, 1); put16(hdr + 28, 24);
    put32(hdr + 34, (unsigned int)row * (unsigned int)h);
    FILE *f = fopen(path, "wb");
    if (!f) return 0;
    fwrite(hdr, 1, 54, f);
    unsigned char *line = calloc(row, 1);
    for (int y = h - 1; y >= 0; y--) {
        for (int x = 0; x < w; x++) {
            const unsigned char *p = rgb + 3 * (y * w + x);
            line[3 * x] = p[2]; line[3 * x + 1] = p[1]; line[3 * x + 2] = p[0];
        }
        fwrite(line, 1, row, f);
    }
    free(line);
    fclose(f);
    return 1;
}

/* ---------- window state (shared by both backends) ---------- */
static const int res_w[3] = {320, 480, 640};   /**< Render widths for keys 1, 2, 3. */
static const int res_h[3] = {180, 270, 360};   /**< Render heights for keys 1, 2, 3. */
static int res_index = 1;                      /**< Current render size (0..2). */
static int nthreads = 1;                       /**< Render threads (set from cpu_count()). */
static unsigned char *buf;                     /**< Frame buffer, MAX_W x MAX_H x 3 bytes. */
static GLuint tex = 0;                         /**< OpenGL texture that shows the frame. */
static int tex_w = 0, tex_h = 0;               /**< Current texture size. */
static int win_w = 1280, win_h = 720;          /**< Initial window size. */
static int quit_requested = 0;                 /**< Set by Esc to leave the main loop. */

/* ---------- actions and camera control (both backends call these) ---------- */
/** @brief Key actions shared by both window back ends. */
enum { ACT_QUIT, ACT_RESET, ACT_RES1, ACT_RES2, ACT_RES3 };

/** @brief Performs a key action (quit, reset camera, change render size). */
static void do_action(int a) {
    switch (a) {
    case ACT_QUIT:  quit_requested = 1; break;
    case ACT_RESET: cam_reset(); break;
    case ACT_RES1:  res_index = 0; break;
    case ACT_RES2:  res_index = 1; break;
    case ACT_RES3:  res_index = 2; break;
    default: break;
    }
}

/** @brief Mouse look: turns the camera by (dx, dy) window pixels; pitch is limited to +-89 degrees. */
static void look(double dx, double dy) {
    cam.yaw += dx * MOUSE_SENS;
    cam.pitch -= dy * MOUSE_SENS;
    if (cam.pitch > 89) cam.pitch = 89;
    if (cam.pitch < -89) cam.pitch = -89;
}

/** @brief Changes the field of view by 2 degrees per step (steps > 0 zooms in), within 20..100. */
static void zoom(double steps) {
    cam.fov -= steps * 2.0;
    if (cam.fov < 20) cam.fov = 20;
    if (cam.fov > 100) cam.fov = 100;
}

/* ---------- held keys: GLFW polls them, FreeGLUT fills key_held from its callbacks ---------- */
/** @brief Keys that are held down (polled each frame). */
enum { KEY_W, KEY_A, KEY_S, KEY_D, KEY_RUN, KEY_ZOOM_IN, KEY_ZOOM_OUT, KEY_COUNT };

#if defined(USE_GLFW)
/** @brief The GLFW window. */
static GLFWwindow *g_win;
/** @brief GLFW key code for each held-key slot. */
static const int glfw_keymap[KEY_COUNT] = {
    GLFW_KEY_W, GLFW_KEY_A, GLFW_KEY_S, GLFW_KEY_D,
    GLFW_KEY_LEFT_SHIFT, GLFW_KEY_EQUAL, GLFW_KEY_MINUS
};
/** @brief 1 if held-key slot k is pressed. */
static int key_down(int k) { return glfwGetKey(g_win, glfw_keymap[k]) == GLFW_PRESS; }
#else
static int key_held[KEY_COUNT];
static int key_down(int k) { return key_held[k]; }
#endif

/**
 * @brief Moves the camera on the ground plane from WASD and changes zoom from +/-.
 * @param dt Frame time in seconds, so speed does not depend on frame rate.
 */
static void handle_move(double dt) {
    double move = WALK_SPEED * dt * (key_down(KEY_RUN) ? 3.0 : 1.0);
    double yr = cam.yaw * PI / 180.0;
    double fx = sin(yr), fz = cos(yr);      /* ground-plane forward */
    double rx = cos(yr), rz = -sin(yr);     /* ground-plane right */
    if (key_down(KEY_W)) { cam.x += fx * move; cam.z += fz * move; }
    if (key_down(KEY_S)) { cam.x -= fx * move; cam.z -= fz * move; }
    if (key_down(KEY_D)) { cam.x += rx * move; cam.z += rz * move; }
    if (key_down(KEY_A)) { cam.x -= rx * move; cam.z -= rz * move; }
    if (key_down(KEY_ZOOM_IN))  cam.fov = fmax(20, cam.fov - 30 * dt);
    if (key_down(KEY_ZOOM_OUT)) cam.fov = fmin(100, cam.fov + 30 * dt);
}

/* ---------- drawing helpers (same for both backends) ---------- */
/** @brief Creates the texture used to show the ray traced image. */
static void setup_gl(void) {
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glClearColor(0, 0, 0, 1);
}

/**
 * @brief Copies the frame into the texture and draws it on a quad that fills the window.
 * @param w,h Render size. @param fbw,fbh Framebuffer size in pixels (larger on high-DPI screens).
 */
static void upload_and_draw(int w, int h, int fbw, int fbh) {
    glBindTexture(GL_TEXTURE_2D, tex);
    if (w != tex_w || h != tex_h) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
        tex_w = w; tex_h = h;
    }
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, buf);

    glViewport(0, 0, fbw, fbh);
    glClear(GL_COLOR_BUFFER_BIT);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glColor3f(1, 1, 1);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0, 1, 0, 1, -1, 1);
    glMatrixMode(GL_MODELVIEW);  glLoadIdentity();
    glBegin(GL_QUADS);
    glTexCoord2f(0, 1); glVertex2f(0, 0);
    glTexCoord2f(1, 1); glVertex2f(1, 0);
    glTexCoord2f(1, 0); glVertex2f(1, 1);
    glTexCoord2f(0, 0); glVertex2f(0, 1);
    glEnd();
    glDisable(GL_TEXTURE_2D);
}

/** @brief Prints object, lamp and thread counts. */
static void print_scene_info(void) {
    printf("scene: %d objects, %d lamps, 1 sun, %d threads\n", nobj, nlamp, nthreads);
}

/** @brief Prints the controls. */
static void print_controls(void) {
    printf("controls: mouse look, WASD, Shift run, scroll/+- zoom, R reset, 1/2/3 resolution, Esc quit\n");
}

/* ---------- backend: window, input callbacks and timing ---------- */
/** @brief Runs one frame (declared here, defined below). */
static void frame(double dt);

#if defined(USE_GLFW)
/** @brief Seconds since the window library started. */
static double win_time(void) { return glfwGetTime(); }
/** @brief Framebuffer size in pixels. */
static void win_framebuffer(int *w, int *h) { glfwGetFramebufferSize(g_win, w, h); }
/** @brief Shows the finished frame. */
static void win_swap(void) { glfwSwapBuffers(g_win); }
/** @brief Sets the window title. */
static void win_set_title(const char *t) { glfwSetWindowTitle(g_win, t); }

/** @brief GLFW cursor callback: turns mouse movement into look(). */
static void glfw_cursor(GLFWwindow *w, double x, double y) {
    static double last_x, last_y;
    static int have = 0;
    (void)w;
    if (have) look(x - last_x, y - last_y);
    last_x = x; last_y = y; have = 1;
}

/** @brief GLFW scroll callback: zooms. */
static void glfw_scroll(GLFWwindow *w, double dx, double dy) {
    (void)w; (void)dx;
    zoom(dy);
}

/** @brief GLFW key callback for one-shot keys (Esc, R, 1, 2, 3). */
static void glfw_key(GLFWwindow *w, int key, int sc, int action, int mods) {
    (void)w; (void)sc; (void)mods;
    if (action != GLFW_PRESS) return;
    switch (key) {
    case GLFW_KEY_ESCAPE: do_action(ACT_QUIT); break;
    case GLFW_KEY_R:      do_action(ACT_RESET); break;
    case GLFW_KEY_1:      do_action(ACT_RES1); break;
    case GLFW_KEY_2:      do_action(ACT_RES2); break;
    case GLFW_KEY_3:      do_action(ACT_RES3); break;
    default: break;
    }
}
#else
static double g_last_t = 0;

static double win_time(void) { return glutGet(GLUT_ELAPSED_TIME) / 1000.0; }
static void win_framebuffer(int *w, int *h) { *w = glutGet(GLUT_WINDOW_WIDTH); *h = glutGet(GLUT_WINDOW_HEIGHT); }
static void win_swap(void) { glutSwapBuffers(); }
static void win_set_title(const char *t) { glutSetWindowTitle(t); }

/* FreeGLUT reports Shift only inside key callbacks, so run mode updates on key presses and releases */
static void glut_key_down(unsigned char key, int x, int y) {
    (void)x; (void)y;
    key_held[KEY_RUN] = (glutGetModifiers() & GLUT_ACTIVE_SHIFT) != 0;
    switch (tolower(key)) {
    case 'w': key_held[KEY_W] = 1; break;
    case 'a': key_held[KEY_A] = 1; break;
    case 's': key_held[KEY_S] = 1; break;
    case 'd': key_held[KEY_D] = 1; break;
    case '=': case '+': key_held[KEY_ZOOM_IN] = 1; break;
    case '-': case '_': key_held[KEY_ZOOM_OUT] = 1; break;
    case 27:  do_action(ACT_QUIT); break;
    case 'r': do_action(ACT_RESET); break;
    case '1': do_action(ACT_RES1); break;
    case '2': do_action(ACT_RES2); break;
    case '3': do_action(ACT_RES3); break;
    default: break;
    }
}

static void glut_key_up(unsigned char key, int x, int y) {
    (void)x; (void)y;
    key_held[KEY_RUN] = (glutGetModifiers() & GLUT_ACTIVE_SHIFT) != 0;
    switch (tolower(key)) {
    case 'w': key_held[KEY_W] = 0; break;
    case 'a': key_held[KEY_A] = 0; break;
    case 's': key_held[KEY_S] = 0; break;
    case 'd': key_held[KEY_D] = 0; break;
    case '=': case '+': key_held[KEY_ZOOM_IN] = 0; break;
    case '-': case '_': key_held[KEY_ZOOM_OUT] = 0; break;
    default: break;
    }
}

/* Mouse look without a hidden cursor: keep the pointer at the window centre and use the offset */
static void glut_motion(int x, int y) {
    int cx = glutGet(GLUT_WINDOW_WIDTH) / 2, cy = glutGet(GLUT_WINDOW_HEIGHT) / 2;
    if (x == cx && y == cy) return;
    look(x - cx, y - cy);
    glutWarpPointer(cx, cy);
}

/** @brief FreeGLUT mouse-wheel callback: zooms. */
static void glut_wheel(int wheel, int direction, int x, int y) {
    (void)wheel; (void)x; (void)y;
    zoom(direction);
}

static void glut_display(void) { /* frames are drawn from the idle callback */ }

/** @brief FreeGLUT idle callback: renders a frame and leaves the loop on quit. */
static void glut_idle(void) {
    double now = win_time();
    frame(now - g_last_t);
    g_last_t = now;
    if (quit_requested) glutLeaveMainLoop();
}
#endif

/* ---------- one frame: move, light, trace, show ---------- */
/**
 * @brief One interactive frame: move the camera, ray trace, show the image, update the FPS in the title.
 * @param dt Seconds since the last frame (capped at 0.1 s).
 */
static void frame(double dt) {
    static double ms_avg = 0, title_t = 0;
    char title[256];
    if (dt > 0.1) dt = 0.1;

    handle_move(dt);

    int w = res_w[res_index], h = res_h[res_index];
    int fbw, fbh;
    win_framebuffer(&fbw, &fbh);

    double t0 = now_ms();
    render_frame(buf, w, h, current_view(), 1, nthreads);
    double ms = now_ms() - t0;
    ms_avg = ms_avg == 0 ? ms : 0.9 * ms_avg + 0.1 * ms;

    upload_and_draw(w, h, fbw, fbh);
    win_swap();

    double now = win_time();
    if (now - title_t > 0.25) {
        title_t = now;
        snprintf(title, sizeof title, "City ray tracer | %dx%d | %.1f FPS (%.1f ms)",
                 w, h, 1000.0 / ms_avg, ms_avg);
        win_set_title(title);
    }
}

/* ---------- headless modes ---------- */
/**
 * @brief Headless mode: renders the start view at 640 x 360 and saves it as a BMP.
 * @param path Output file. @return 0 on success.
 */
static int run_snapshot(const char *path) {
    build_scene();
    cam_reset();
    double t0 = now_ms();
    render_frame(buf, 640, 360, current_view(), 1, nthreads);
    double ms = now_ms() - t0;
    printf("snapshot 640x360: %.1f ms\n", ms);
    if (!write_bmp(path, 640, 360, buf)) { fprintf(stderr, "could not write %s\n", path); return 1; }
    printf("wrote %s\n", path);
    return 0;
}

/**
 * @brief Headless benchmark: walks 0.4 m per frame down the road and prints the average frame time.
 * @param frames Number of frames. @return 0.
 */
static int run_bench(int frames) {
    build_scene();
    cam_reset();
    double total = 0;
    int w = res_w[res_index], h = res_h[res_index];
    printf("bench: %dx%d, %d frames, %d threads\n", w, h, frames, nthreads);
    for (int f = 0; f < frames; f++) {
        cam.z += 0.4;                              /* walk down the road */
        double t0 = now_ms();
        render_frame(buf, w, h, current_view(), 1, nthreads);
        total += now_ms() - t0;
    }
    double avg = total / frames;
    printf("avg frame %.1f ms, FPS %.2f\n", avg, 1000.0 / avg);
    return 0;
}

/* ---------- window main loop ---------- */
#if defined(USE_GLFW)
/** @brief Opens the GLFW window and runs the interactive loop until Esc or close. @return 0 on success. */
static int run_window(int argc, char **argv) {
    (void)argc; (void)argv;
    if (!glfwInit()) { fprintf(stderr, "Could not start GLFW\n"); return 1; }
    g_win = glfwCreateWindow(win_w, win_h, "City ray tracer", NULL, NULL);
    if (!g_win) { fprintf(stderr, "Could not open a window\n"); glfwTerminate(); return 1; }
    glfwMakeContextCurrent(g_win);
    glfwSwapInterval(0);                           /* no vsync: show the real render rate */
    glfwSetInputMode(g_win, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSetCursorPosCallback(g_win, glfw_cursor);
    glfwSetScrollCallback(g_win, glfw_scroll);
    glfwSetKeyCallback(g_win, glfw_key);

    setup_gl();
    build_scene();
    cam_reset();
    print_scene_info();
    print_controls();

    double last = glfwGetTime();
    while (!glfwWindowShouldClose(g_win) && !quit_requested) {
        glfwPollEvents();
        double now = glfwGetTime();
        frame(now - last);
        last = now;
    }

    glDeleteTextures(1, &tex);
    glfwDestroyWindow(g_win);
    glfwTerminate();
    return 0;
}
#else
/** @brief Opens the FreeGLUT window and runs the interactive loop. @return 0. */
static int run_window(int argc, char **argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
    glutInitWindowSize(win_w, win_h);
    glutCreateWindow("City ray tracer");
    glutDisplayFunc(glut_display);
    glutIdleFunc(glut_idle);
    glutKeyboardFunc(glut_key_down);
    glutKeyboardUpFunc(glut_key_up);
    glutPassiveMotionFunc(glut_motion);
    glutMotionFunc(glut_motion);
    glutMouseWheelFunc(glut_wheel);
    glutSetCursor(GLUT_CURSOR_NONE);
    glutWarpPointer(win_w / 2, win_h / 2);

    setup_gl();
    build_scene();
    cam_reset();
    print_scene_info();
    print_controls();

    g_last_t = win_time();
    glutMainLoop();

    glDeleteTextures(1, &tex);
    return 0;
}
#endif

/* ---------- main ---------- */
/**
 * @brief Entry point.
 *
 * Usage: no arguments opens the window; `--snapshot out.bmp` saves one image;
 * `--bench N` runs the benchmark for N frames.
 */
int main(int argc, char **argv) {
    nthreads = cpu_count();
    if (nthreads > MAX_THREADS) nthreads = MAX_THREADS;
    buf = malloc((size_t)MAX_W * MAX_H * 3);

    if (argc >= 3 && strcmp(argv[1], "--snapshot") == 0) return run_snapshot(argv[2]);
    if (argc >= 3 && strcmp(argv[1], "--bench") == 0) return run_bench(atoi(argv[2]));

    int rc = run_window(argc, argv);
    free(buf);
    return rc;
}
