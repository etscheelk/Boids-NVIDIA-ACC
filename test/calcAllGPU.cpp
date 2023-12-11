#ifndef BOIDS_CPP
#define BOIDS_CPP
#include "boids.cpp"
#endif

namespace calcAllGPU 
{
    void calculate_all_headings() 
    {
        #pragma acc kernels
        #pragma acc loop independent present(xp[0:num], yp[0:num])
        for (int i = 0; i < num; ++i) {
            compute_new_heading(i);
        }
    }
}