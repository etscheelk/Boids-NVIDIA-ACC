// #pragma once
#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <cmath>
#include <stdio.h>
#include <stdlib.h>


namespace boids {
    #define LEN(x, y) sqrt(SQR(x) + SQR(y))
    #define DIST(x1, y1, x2, y2) LEN(((x1) - (x2)), ((y1) - (y2)))
    #define DOT(x1, y1, x2, y2) ((x1) * (x2) + (y1) * (y2))

    #define MIN(x, y) ((x) < (y) ? (x) : (y))
    #define MAX(x, y) ((x) > (y) ? (x) : (y))
    #define ABS(x) ((x) >= 0 ? (x) : -(x))
    #define SIGN(x) ((x) >= 0 ? 1 : -1)
    #define SQR(x) ((x) * (x))
    #define GETBIT(c, i) (((1 << (i)) & (c)) ? 1 : 0)
    #define SETBIT(c, i, b) ((b) ? ((c) | (1 << (i))) : ((c) & ~(1 << (i))))

    int width = 640, height = 480, num = 1024, len = 20, mag = 1;
    int seed = 0, invert = 0, steps = 1000, psdump = 0;

    float angle = 270.0, vangle = 90, minv = 0.5, ddt = 0.95, dt = 3.0;
    float rcopy = 80, rcent = 30, rviso = 40, rvoid = 15;
    float wcopy = 0.2, wcent = 0.4, wviso = 0.8, wvoid = 1.0, wrand = 0.0;

    char *term = NULL;

    char help_string[] = "\
    Simulate a flock of boids according to rules that determine their \
    individual behaviors as well as the ``physics'' of their universe. \
    A boid greedily attempts to apply four rules with respect to its \
    neighbors: it wants to fly in the same direction, be in the center \
    of the local cluster of boids, avoid collisions with boids too close, \
    and maintain a clear view ahead by skirting around others that block \
    its view.  Changing these rules can make the boids behave like birds, \
    gnats, bees, fish, or magnetic particles.  See the RULES section of \
    the manual pages for more details.\
    ";

    float *xp, *yp, *xv, *yv, *xnv, *ynv;

    #pragma acc routine
    float random_range(float low, float high);

    #pragma acc routine
    void norm(float *x, float *y);

    #pragma acc routine
    void compute_new_heading(int which);

    #pragma acc routine
    void compute_all_headings(int numThreads);
}

#endif