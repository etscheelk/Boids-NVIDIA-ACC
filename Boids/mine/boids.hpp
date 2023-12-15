#ifndef BOIDS_HPP
#define BOIDS_HPP

#include "misc.h"

namespace boids {
    #define LEN(x, y) sqrt(SQR(x) + SQR(y))
    #define DIST(x1, y1, x2, y2) LEN(((x1) - (x2)), ((y1) - (y2)))
    #define DOT(x1, y1, x2, y2) ((x1) * (x2) + (y1) * (y2))

    struct Params
    {
        int width;
        int height;
        int num;
        int len;
        int mag;
        int seed;
        int invert;
        int steps;
        int psdump;

        double angle;
        double vangle;
        double minv;
        double ddt;
        double dt;
        double rcopy;
        double rcent;
        double rviso;
        double rvoid;
        double wcopy;
        double wcent;
        double wviso;
        double wvoid;
        // double  wrand = 0.0;   // eliminate for simplicity

        int threads; // will ignore for openACC version; used for multicore

        char *term;
    };

    // LS use struct for default parameters
    // ETS moved to header file
    // struct Params defaultParams = 
    // {
	// 	.width = 640, .height = 480, 
	// 	.num = 20, .len = 20, 
	// 	.mag = 1, .seed = 0, 
	// 	.invert = 0, .steps = 100000000, 
	// 	.psdump = 0, .angle = 270.0, 
	// 	.vangle = 90, .minv = 0.5, 
	// 	.ddt = 0.95, .dt = 3.0, 
	// 	.rcopy = 80, .rcent = 30, 
	// 	.rviso = 40, .rvoid = 15, 
	// 	.wcopy = 0.2, .wcent = 0.4, 
	// 	.wviso = 0.8, .wvoid = 1.0, .
	// 	threads = 1, .term = NULL
    // };

    // OPTION* setOptions(struct Params& params) {
    //     OPTION options[] = {
    //         {"-width", OPT_INT, &params.width, "Width of the plot in pixels."},
    //         {"-height", OPT_INT, &params.height, "Height of the plot in pixels."},
    //         {"-num", OPT_INT, &params.num, "Number of boids."},
    //         {"-steps", OPT_INT, &params.steps, "Number of simulated steps."},
    //         {"-seed", OPT_INT, &params.seed, "Random seed for initial state."},
    //         {"-angle", OPT_DOUBLE, &params.angle, "Number of viewing degrees."},
    //         {"-vangle", OPT_DOUBLE, &params.vangle, "Visual avoidance angle."},
    //         {"-rcopy", OPT_DOUBLE, &params.rcopy, "Radius for copy vector."},
    //         {"-rcent", OPT_DOUBLE, &params.rcent, "Radius for centroid vector."},
    //         {"-rvoid", OPT_DOUBLE, &params.rvoid, "Radius for avoidance vector."},
    //         {"-rviso", OPT_DOUBLE, &params.rviso, "Radius for visual avoidance vector."},
    //         {"-wcopy", OPT_DOUBLE, &params.wcopy, "Weight for copy vector."},
    //         {"-wcent", OPT_DOUBLE, &params.wcent, "Weight for centroid vector."},
    //         {"-wvoid", OPT_DOUBLE, &params.wvoid, "Weight for avoidance vector."},
    //         {"-wviso", OPT_DOUBLE, &params.wviso, "Weight for visual avoidance vector."},
    //         // { "-wrand",  OPT_DOUBLE,  &params.wrand,  "Weight for random vector." },
    //         {"-dt", OPT_DOUBLE, &params.dt, "Time-step increment."},
    //         {"-ddt", OPT_DOUBLE, &params.ddt, "Momentum factor (0 < ddt < 1)."},
    //         {"-minv", OPT_DOUBLE, &params.minv, "Minimum velocity."},
    //         {"-len", OPT_INT, &params.len, "Tail length."},
    //         {"-psdump", OPT_SWITCH, &params.psdump, "Dump PS at the very end?"},
    //         {"-inv", OPT_SWITCH, &params.invert, "Invert all colors?"},
    //         {"-mag", OPT_INT, &params.mag, "Magnification factor."},
    //         {"-term", OPT_STRING, &params.term, "How to plot points."},
    //         {"-t", OPT_INT, &params.threads, "Number of threads."},
    //         {NULL, OPT_NULL, NULL, NULL}
	//     };

    //     return options;
    // }

    void norm(float* x, float* y);

    void compute_new_headings(struct Params p, float* xp, float* yp, float* xv, float* yv, float* xnv, float* ynv);

    void draw_boid(struct Params p, int which, int color, float *xp, float *yp, float *xv, float *yv);

}
#endif