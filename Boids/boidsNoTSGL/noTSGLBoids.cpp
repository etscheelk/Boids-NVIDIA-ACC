#include "boids.hpp"
#include "misc.h"
#include <omp.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <memory>

struct boids::Params p;

struct boids::Params defaultParams = 
    {
		.width = 1024, .height = 1024, 
		.num = 1024, .len = 20, 
		.mag = 1, .seed = 0, 
		.invert = 0, .steps = 100000000, 
		.psdump = 0, .angle = 270.0, 
		.vangle = 90, .minv = 0.5, 
		.ddt = 0.95, .dt = 3.0, 
		.rcopy = 80, .rcent = 30, 
		.rviso = 40, .rvoid = 15, 
		.wcopy = 0.2, .wcent = 0.4, 
		.wviso = 0.8, .wvoid = 1.0, .
		threads = 1, .term = NULL
    };

float* xp;
float* yp;
float* xv;
float* yv;
float* xnv;
float* ynv;

void initiateBoidArrays(
    struct boids::Params p, 
    float* xp, float* yp,
    float* xv, float* yv
) 
{
    for (int i = 0; i < p.num; ++i) {
        xp[i] = random_range(-p.width/2, p.width/2);
        yp[i] = random_range(-p.height/2, p.height/2);
        xv[i] = random_range(-1., 1.);
        yv[i] = random_range(-1., 1.);
        boids::norm(&xv[i], &yv[i]);
    }
}

void boidIteration (
    boids::Params p,
    float* xp, float* yp,
    float* xv, float* yv,
    float* xnv, float* ynv
) 
{
    boids::compute_new_headings(p, xp, yp, xv, yv, xnv, ynv);

    for (int i = 0; i < p.num; ++i) 
    {
        xv[i] = xnv[i];
        yv[i] = ynv[i];
        xp[i] += xv[i] * p.dt;
        yp[i] += yv[i] * p.dt;
        
        
        // Wrap around screen coordinates
        if (xp[i] < -p.width / 2) {
            xp[i] += p.width;
        }
        else if (xp[i] >= p.width / 2) {
            xp[i] -= p.width;
        }

        if (yp[i] < -p.height / 2) {
            yp[i] += p.height;
        }
        else if (yp[i] >= p.height / 2) {
            yp[i] -= p.height;
        }
    }
}

int main(int argc, char* argv[]) {
    
    p = defaultParams;


    // OPTION* o = boids::setOptions(p);
    // get_options(argc, argv, o, "test");

    p.num = 128;

    p.width = 2000;
    p.height = 1300;

    if (argc > 1) {
        p.threads = atoi(argv[1]);
    }

    if (argc > 2) {
        p.num = atoi(argv[2]);
    }

    xp  = new float[p.num];
    yp  = new float[p.num];
    xv  = new float[p.num];
    yv  = new float[p.num];
    xnv = new float[p.num];
    ynv = new float[p.num];

    initiateBoidArrays(p, xp, yp, xv, yv);
    fprintf(stderr, "Boid size of %d starting\n", p.num);
    double t1 = omp_get_wtime();
    for (int i = 0; i < 1000; ++i) {
        boidIteration(p, xp, yp, xv, yv, xnv, ynv);
        if (i % 50 == 0) {
            fprintf(stderr, "\tit %d done\n", i);
        }
    }
    double t2 = omp_get_wtime();

    
    fprintf(stdout, "%lf", t2 - t1);
    fprintf(stderr, "\n%lf\n\n", t2 - t1);

    delete [] xp;
    delete [] yp;
    delete [] xv;
    delete [] yv;
    delete [] xnv;
    delete [] ynv;
}