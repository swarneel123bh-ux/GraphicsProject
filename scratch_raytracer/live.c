/* live.c - walk around a city block while it is ray traced on the CPU (scratch, not for submission).
 *
 * Window and controls follow the Assignment 2 app (GLFW, mouse look, WASD). The image is traced
 * on the CPU with pthreads, one row at a time, and shown as a texture on a full-window quad.
 *
 * Build (macOS, Homebrew GLFW):
 *   cc -std=c11 -O2 -Wall -pthread -I/opt/homebrew/include -L/opt/homebrew/lib -o live live.c -lglfw -framework OpenGL -lm
 * Run:
 *   ./live                        interactive window
 *   ./live --snapshot out.bmp     render one frame at 640x360, no window
 *   ./live --bench 60             walk 60 frames along a fixed path, print FPS, no window
 *
 * Controls: mouse look, W/A/S/D walk and strafe, Shift run, scroll or +/- zoom, R reset,
 *           1/2/3 render size 320x180 / 480x270 / 640x360, T pause/resume day-night cycle, Esc quit.
 */
#define _DARWIN_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>

#define PI 3.14159265358979323846
#define INF 1e30
#define EPS 1e-4
#define MAX_DEPTH 2
#define MAX_OBJ 512
#define MAX_LAMP 16
#define MAX_THREADS 64
#define MAX_W 640
#define MAX_H 360

#define WALK_SPEED 5.0      /* metres per second; Shift runs 3x */
#define MOUSE_SENS 0.1      /* degrees per pixel */

/* ---------- vector maths ---------- */
typedef struct { double x, y, z; } V3;
static V3 V(double x, double y, double z) { V3 r = {x, y, z}; return r; }
static V3 add(V3 a, V3 b) { return V(a.x + b.x, a.y + b.y, a.z + b.z); }
static V3 sub(V3 a, V3 b) { return V(a.x - b.x, a.y - b.y, a.z - b.z); }
static V3 scl(V3 a, double s) { return V(a.x * s, a.y * s, a.z * s); }
static V3 mul(V3 a, V3 b) { return V(a.x * b.x, a.y * b.y, a.z * b.z); }
static double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static V3 cross(V3 a, V3 b) { return V(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
static V3 norm(V3 a) { return scl(a, 1.0 / sqrt(dot(a, a))); }
static V3 neg(V3 a) { return V(-a.x, -a.y, -a.z); }
static V3 mix(V3 a, V3 b, double t) { return add(scl(a, 1.0 - t), scl(b, t)); }

/* ---------- objects ---------- */
enum { BOX, SPHERE, CYL, CONE };

typedef struct {
    int type;
    V3 lo, hi;    /* BOX: min/max. SPHERE: lo = centre. CYL/CONE: lo = (cx, y0, cz), hi.y = y1 */
    double r;     /* radius */
    V3 col;
    double refl;  /* mirror amount 0..1 */
    double emis;  /* > 0: self-lit, not shadowed */
} Obj;

typedef struct { V3 pos; V3 col; double k; } Lamp;

static Obj objs[MAX_OBJ];
static int nobj = 0;
static Lamp lamps[MAX_LAMP];
static int nlamp = 0;
static V3 sun_dir, sun_col, sky_top, sky_hor, ambient;
static double sun_k = 1.0, daylt = 1.0;
static _Thread_local long long nrays = 0;   /* per-thread ray counter */

static int add_obj(int type, V3 lo, V3 hi, double r, V3 col, double refl, double emis) {
    Obj *o = &objs[nobj];
    o->type = type; o->lo = lo; o->hi = hi; o->r = r;
    o->col = col; o->refl = refl; o->emis = emis;
    return nobj++;
}

static void add_lamp(V3 pos, V3 col, double k) {
    lamps[nlamp].pos = pos; lamps[nlamp].col = col; lamps[nlamp].k = k;
    nlamp++;
}

/* ---------- intersections (directions normalised; tmax = nearest accepted t) ---------- */
static int hit_box(V3 o, V3 d, V3 lo, V3 hi, double tmax, double *t, V3 *n) {
    double oa[3] = {o.x, o.y, o.z}, da[3] = {d.x, d.y, d.z};
    double la[3] = {lo.x, lo.y, lo.z}, ha[3] = {hi.x, hi.y, hi.z};
    double tn = -INF, tf = INF, sg = 0;
    int ax = -1;
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

static int hit_cyl(V3 o, V3 d, V3 c, double y0, double y1, double r, double tmax, double *t, V3 *n) {
    double best = tmax, ox = o.x - c.x, oz = o.z - c.z;
    int hit = 0;
    V3 bn = V(0, 0, 0);
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

static int hit_obj(const Obj *ob, V3 o, V3 d, double tmax, double *t, V3 *n) {
    switch (ob->type) {
    case BOX:    return hit_box(o, d, ob->lo, ob->hi, tmax, t, n);
    case SPHERE: return hit_sphere(o, d, ob->lo, ob->r, tmax, t, n);
    case CYL:    return hit_cyl(o, d, ob->lo, ob->lo.y, ob->hi.y, ob->r, tmax, t, n);
    default:     return hit_cone(o, d, ob->lo, ob->lo.y, ob->hi.y, ob->r, tmax, t, n);
    }
}

/* ---------- 2D grid over the XZ plane ----------
 * Each object is listed in every grid cell its XZ footprint touches. Objects with a huge
 * footprint (the ground) are tested for every ray. A ray walks the cells it passes through
 * (2D DDA) and stops once the nearest hit found so far is closer than the next cell. */
#define GRID_N 36
#define GRID_CELL 4.0
#define GRID_MIN -72.0
#define CELL_CAP 96
#define GLOBAL_EXTENT 20.0

static int cell_n[GRID_N * GRID_N];
static int cell_obj[GRID_N * GRID_N][CELL_CAP];
static int glob[MAX_OBJ];
static int nglob = 0;

static int cell_of(double v) {
    int k = (int)floor((v - GRID_MIN) / GRID_CELL);
    if (k < 0) k = 0;
    if (k >= GRID_N) k = GRID_N - 1;
    return k;
}

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

/* Nearest hit (any == 0) or first hit with t < tmax (any == 1, returns index or -1). */
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

static int closest(V3 o, V3 d, double *t, V3 *n, int *idx) {
    double tt;
    V3 nn;
    int i = traverse(o, d, INF, 0, &tt, &nn);
    if (i < 0) return 0;
    *t = tt; *n = nn; *idx = i;
    return 1;
}

static int occluded(V3 o, V3 d, double maxdist) {
    double tt;
    V3 nn;
    return traverse(o, d, maxdist, 1, &tt, &nn) >= 0;
}

/* ---------- shading ---------- */
static V3 sky(V3 d) {
    double t = 0.5 * (d.y + 1.0);
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return mix(sky_hor, sky_top, t);
}

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

/* Sun angle a drives the sun's height: a > 0 is day, a < 0 is night. Lamps brighten at night. */
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
static unsigned int seed = 12345u;
static double frand(void) {
    seed = seed * 1103515245u + 12345u;
    return ((seed >> 8) & 0xFFFF) / 65535.0;
}

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
}

/* ---------- camera (same conventions as Assignment 2: yaw 0 looks down +z) ---------- */
typedef struct { double x, y, z, yaw, pitch, fov; } Cam;
static Cam cam;

static void cam_reset(void) {
    cam.x = 0; cam.y = 1.7; cam.z = -30;
    cam.yaw = 0; cam.pitch = -3; cam.fov = 60;
}

typedef struct { V3 eye, fwd, right, up; double th; } View;

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
typedef struct {
    int W, H, spp;
    View vw;
    unsigned char *img;
    int next_row;
    pthread_mutex_t lock;
} Job;

typedef struct { Job *job; long long rays; } Worker;

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
static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

/* ---------- BMP output (for --snapshot) ---------- */
static void put32(unsigned char *p, unsigned int v) {
    p[0] = v & 255; p[1] = (v >> 8) & 255; p[2] = (v >> 16) & 255; p[3] = (v >> 24) & 255;
}
static void put16(unsigned char *p, unsigned int v) { p[0] = v & 255; p[1] = (v >> 8) & 255; }

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

/* ---------- globals for the window ---------- */
static const int res_w[3] = {320, 480, 640};
static const int res_h[3] = {180, 270, 360};
static int res_index = 1;
static int nthreads = 1;
static int cycle_running = 1;
static double sun_angle = 1.0;     /* radians; day-night cycle advances this */
static unsigned char *buf;         /* MAX_W * MAX_H * 3 */
static GLuint tex = 0;
static int tex_w = 0, tex_h = 0;
static int have_mouse = 0;
static double last_mx, last_my;

static void on_mouse(GLFWwindow *win, double x, double y) {
    (void)win;
    if (have_mouse) {
        cam.yaw += (x - last_mx) * MOUSE_SENS;
        cam.pitch += (last_my - y) * MOUSE_SENS;
        if (cam.pitch > 89) cam.pitch = 89;
        if (cam.pitch < -89) cam.pitch = -89;
    }
    last_mx = x; last_my = y; have_mouse = 1;
}

static void on_scroll(GLFWwindow *win, double dx, double dy) {
    (void)win; (void)dx;
    cam.fov -= dy * 2.0;
    if (cam.fov < 20) cam.fov = 20;
    if (cam.fov > 100) cam.fov = 100;
}

static void on_key(GLFWwindow *win, int key, int sc, int action, int mods) {
    (void)sc; (void)mods;
    if (action != GLFW_PRESS) return;
    if (key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(win, 1);
    if (key == GLFW_KEY_R) cam_reset();
    if (key == GLFW_KEY_T) cycle_running = !cycle_running;
    if (key == GLFW_KEY_1) res_index = 0;
    if (key == GLFW_KEY_2) res_index = 1;
    if (key == GLFW_KEY_3) res_index = 2;
}

static int down(GLFWwindow *win, int key) { return glfwGetKey(win, key) == GLFW_PRESS; }

static void handle_keys(GLFWwindow *win, double dt) {
    double move = WALK_SPEED * dt * (down(win, GLFW_KEY_LEFT_SHIFT) ? 3.0 : 1.0);
    double yr = cam.yaw * PI / 180.0;
    double fx = sin(yr), fz = cos(yr);      /* ground-plane forward */
    double rx = cos(yr), rz = -sin(yr);     /* ground-plane right */
    if (down(win, GLFW_KEY_W)) { cam.x += fx * move; cam.z += fz * move; }
    if (down(win, GLFW_KEY_S)) { cam.x -= fx * move; cam.z -= fz * move; }
    if (down(win, GLFW_KEY_D)) { cam.x += rx * move; cam.z += rz * move; }
    if (down(win, GLFW_KEY_A)) { cam.x -= rx * move; cam.z -= rz * move; }
    if (down(win, GLFW_KEY_EQUAL)) cam.fov = fmax(20, cam.fov - 30 * dt);
    if (down(win, GLFW_KEY_MINUS)) cam.fov = fmin(100, cam.fov + 30 * dt);
}

static void setup_gl(void) {
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glClearColor(0, 0, 0, 1);
}

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

static void print_scene_info(void) {
    printf("scene: %d objects, %d lamps, 1 sun, %d threads\n", nobj, nlamp, nthreads);
}

/* ---------- headless modes ---------- */
static int run_snapshot(const char *path) {
    build_scene();
    update_light(sun_angle);
    cam_reset();
    double t0 = now_ms();
    render_frame(buf, 640, 360, current_view(), 1, nthreads);
    double ms = now_ms() - t0;
    printf("snapshot 640x360: %.1f ms\n", ms);
    if (!write_bmp(path, 640, 360, buf)) { fprintf(stderr, "could not write %s\n", path); return 1; }
    printf("wrote %s\n", path);
    return 0;
}

static int run_bench(int frames) {
    build_scene();
    cam_reset();
    double total = 0;
    int w = res_w[res_index], h = res_h[res_index];
    printf("bench: %dx%d, %d frames, %d threads\n", w, h, frames, nthreads);
    for (int f = 0; f < frames; f++) {
        cam.z += 0.4;                              /* walk down the road */
        update_light(sun_angle + 0.01 * f);        /* and let the light move */
        double t0 = now_ms();
        render_frame(buf, w, h, current_view(), 1, nthreads);
        total += now_ms() - t0;
    }
    double avg = total / frames;
    printf("avg frame %.1f ms, FPS %.2f\n", avg, 1000.0 / avg);
    return 0;
}

/* ---------- main ---------- */
int main(int argc, char **argv) {
    long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
    nthreads = ncpu > 0 ? (int)ncpu : 1;
    if (nthreads > MAX_THREADS) nthreads = MAX_THREADS;
    buf = malloc((size_t)MAX_W * MAX_H * 3);

    if (argc >= 3 && strcmp(argv[1], "--snapshot") == 0) return run_snapshot(argv[2]);
    if (argc >= 3 && strcmp(argv[1], "--bench") == 0) return run_bench(atoi(argv[2]));

    if (!glfwInit()) { fprintf(stderr, "Could not start GLFW\n"); return 1; }
    GLFWwindow *window = glfwCreateWindow(1280, 720, "City ray tracer", NULL, NULL);
    if (!window) { fprintf(stderr, "Could not open a window\n"); glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(0);                           /* no vsync: show the real render rate */
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSetCursorPosCallback(window, on_mouse);
    glfwSetScrollCallback(window, on_scroll);
    glfwSetKeyCallback(window, on_key);

    setup_gl();
    build_scene();
    cam_reset();
    print_scene_info();
    printf("controls: mouse look, WASD, Shift run, scroll/+- zoom, R reset, 1/2/3 resolution, T day-night, Esc quit\n");

    double last = glfwGetTime();
    double ms_avg = 0, title_t = 0;
    char title[256];
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        double now = glfwGetTime();
        double dt = now - last;
        last = now;
        if (dt > 0.1) dt = 0.1;

        handle_keys(window, dt);
        if (cycle_running) sun_angle += 0.12 * dt;
        update_light(sun_angle);

        int w = res_w[res_index], h = res_h[res_index];
        int fbw, fbh;
        glfwGetFramebufferSize(window, &fbw, &fbh);

        double t0 = now_ms();
        render_frame(buf, w, h, current_view(), 1, nthreads);
        double ms = now_ms() - t0;
        ms_avg = ms_avg == 0 ? ms : 0.9 * ms_avg + 0.1 * ms;

        upload_and_draw(w, h, fbw, fbh);
        glfwSwapBuffers(window);

        if (now - title_t > 0.25) {
            title_t = now;
            snprintf(title, sizeof title, "City ray tracer | %dx%d | %.1f FPS (%.1f ms) | %s",
                     w, h, 1000.0 / ms_avg, ms_avg, cycle_running ? "day/night on" : "time paused");
            glfwSetWindowTitle(window, title);
        }
    }

    glDeleteTextures(1, &tex);
    glfwDestroyWindow(window);
    glfwTerminate();
    free(buf);
    return 0;
}
