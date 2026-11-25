/* experiments.c - produces the images and timing tables used in the HTML report.
 *
 * It includes live.c unchanged (renaming its main) so the measured code is exactly the project code.
 * Build and run from the project root:
 *   cc -std=c11 -O2 -pthread -DUSE_GLFW -I$(brew --prefix glfw)/include -o build/experiments \
 *      docs/tools/experiments.c -L$(brew --prefix glfw)/lib -lglfw -framework OpenGL -lm
 *   ./build/experiments docs/img   (then convert the .bmp files to .png)
 */
#define main live_main
#include "../../scratch_raytracer/live.c"
#undef main

#define OUT_W 1280
#define OUT_H 720

static unsigned char *img;
static const char *outdir;

/* Place the camera, render one image and save it as <outdir>/<name>.bmp */
static void shot(const char *name, double x, double y, double z, double yaw, double pitch,
                 double fov, int spp) {
    char path[512];
    cam.x = x; cam.y = y; cam.z = z; cam.yaw = yaw; cam.pitch = pitch; cam.fov = fov;
    double t0 = now_ms();
    render_frame(img, OUT_W, OUT_H, current_view(), spp, nthreads);
    double ms = now_ms() - t0;
    snprintf(path, sizeof path, "%s/%s.bmp", outdir, name);
    write_bmp(path, OUT_W, OUT_H, img);
    printf("image %-10s %dx%d spp=%d  %.0f ms\n", name, OUT_W, OUT_H, spp * spp, ms);
}

/* Average time of `reps` renders of the default view */
static double time_render(int w, int h, int spp, int threads, int reps) {
    cam_reset();
    render_frame(img, w, h, current_view(), spp, threads);   /* warm-up */
    double t0 = now_ms();
    for (int i = 0; i < reps; i++) render_frame(img, w, h, current_view(), spp, threads);
    return (now_ms() - t0) / reps;
}

/* Put every object in the "global" list so each ray tests all objects (no grid) */
static void disable_grid(void) {
    memset(cell_n, 0, sizeof cell_n);
    nglob = nobj;
    for (int i = 0; i < nobj; i++) glob[i] = i;
}

int main(int argc, char **argv) {
    outdir = argc > 1 ? argv[1] : ".";
    nthreads = cpu_count();
    if (nthreads > MAX_THREADS) nthreads = MAX_THREADS;
    img = malloc((size_t)OUT_W * OUT_H * 3);
    build_scene();
    printf("scene: %d objects, %d lamps, %d threads\n", nobj, nlamp, nthreads);

    /* 1. gallery (2x2 supersampling for cleaner edges) */
    shot("street",   0, 1.7, -30,   0,  -3, 60, 2);
    shot("aerial", -48, 38, -48,   45, -32, 60, 2);
    shot("sphere",   0, 1.3, -4.5,  0,  -8, 45, 2);
    shot("lamp",     6, 2.2, -20, -25,   4, 55, 2);
    update_light(0.12);                              /* low sun: lamps dominate */
    shot("dusk",     0, 1.7, -30,   0,  -3, 60, 2);
    update_light(SUN_ANGLE);

    /* 2. thread scaling, 640x360, 1 sample */
    printf("\nthreads,ms\n");
    for (int t = 1; t <= nthreads; t *= 2) printf("%d,%.1f\n", t, time_render(640, 360, 1, t, 10));

    /* 3. resolution, all threads */
    printf("\nresolution,ms\n");
    for (int r = 0; r < 3; r++)
        printf("%dx%d,%.1f\n", res_w[r], res_h[r], time_render(res_w[r], res_h[r], 1, nthreads, 10));

    /* 4. supersampling cost, 640x360 */
    printf("\nsamples,ms\n");
    for (int s = 1; s <= 3; s++) printf("%d,%.1f\n", s * s, time_render(640, 360, s, nthreads, 5));

    /* 5. grid vs. testing every object */
    double grid_ms = time_render(640, 360, 1, nthreads, 10);
    disable_grid();
    double brute_ms = time_render(640, 360, 1, nthreads, 10);
    printf("\naccel,ms\ngrid,%.1f\nbrute,%.1f\nspeedup,%.2f\n", grid_ms, brute_ms, brute_ms / grid_ms);

    free(img);
    return 0;
}
