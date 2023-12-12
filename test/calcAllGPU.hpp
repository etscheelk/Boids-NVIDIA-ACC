// nvc++ -c calcAllGPU.cpp boids.o -acc=gpu -gpu=managed -Minfo=accel -o calcAllGPU.o -DGPU -fast

#include "boids.hpp"

#ifndef CALC_GPU
#define CALC_GPU

namespace calcAllGPU 
{
    void calculate_all_headings();
}

#endif