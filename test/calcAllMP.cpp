#ifndef BOIDS_CPP
#define BOIDS_CPP
#include "boids.cpp"
#endif

namespace calcAllMP 
{
    void calculate_all_headings(int numThreads) 
    {
        #pragma acc parallel loop independent num_gangs(numThreads) collapse(1)
        for (int i = 0; i < num; ++i) {
            compute_new_heading(i);
        }
    }
}