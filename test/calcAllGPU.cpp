#include "calcAllGPU.hpp"

#include "boids.hpp"

void calcAllGPU::calculate_all_headings() 
{
    #pragma acc kernels
    #pragma acc loop independent 
    // #pragma acc data copy(xp[:num], yp[:num], xnv[:num], ynv[:num]) copyin(xv[:num], yv[:num], width, height, rvoid) 
    #pragma acc routine(random) seq
    #pragma acc routine(boids::compute_new_heading) seq
    for (int i = 0; i < boids::num; ++i) {
        boids::compute_new_heading(i);
    }
}