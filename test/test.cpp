#include <iostream>
#include <stdlib.h>

#include <tsgl.h>
#include "myVector.cpp"

// #ifndef BOIDS_CPP
// #include "boids.cpp"
// #endif

#ifdef GPU
#include "calcAllGPU.cpp"
#else
#include "calcAllMP.cpp"
#endif

namespace calcAllMP 
{
    void calculate_all_headings(int numThreads);
}

namespace calcAllGPU 
{
    void calculate_all_headings();
}

#include <vector>
#include <memory>

#include <cmath>

#define DEBUG

using namespace tsgl;

int numThreads = 8;
/*
    Arrays I need:
        boids x's
        boids y's
        
        boids velocity x
        boids velocity y
        
        boids new velocity x
        boids new velocity y
*/
// float *bx;  // x, y
// float *by;

// float *vx;  // vel x, y
// float *vy;
// float *nvx; // new vel x, y
// float *nvy;


size_t numBoids = 128;

void testScreen(Canvas &canvas, unsigned numBoids);

inline float distance(float x1, float y1, float x2, float y2) {
    return sqrt(abs(x2-x1)*abs(x2-x1) + abs(y2-y1)*abs(y2-y1));
}

class boid {
private:
    Arrow* arrow;
    Canvas* can;
public:
    int index;
    /**
     * @brief Construct a new boid object, which is only ever stored on the CPU.
     * We use pointer arrays so we don't have to send my boid objects to the GPU.
     * 
     * @param x 
     * @param y 
     * @param can 
     */
    boid(float x, float y, int index, Canvas* canvasP) : can(canvasP) {
        this->index = index;
        arrow = new Arrow(
            x, y, 0, 
            20, 20, 
            0, 0, 0, 
            CYAN
        );
        can->add(arrow);
    }

    /**
     * @brief Destroy the boid object
     * 
     */
    ~boid() {
        can->remove(arrow);
        delete arrow;
    }

    void setColor(tsgl::ColorFloat color) {
        arrow->setColor(color);
    }

    void updatePosition(float x, float y) {
        arrow->setCenter(x, y, 0);
    }

    void updateDirection(float velx, float vely) {
        float yaw = atan(vely / velx) * 180. / PI;
        if (velx > 0) {
            yaw += 180;
        }
        arrow->setYaw(yaw);
    }
};

/**
 * @brief Create the initial conditions for the boids 
 * 
 * @param WW Window width
 * @param WH Window height
 * @param boids Passed reference to the list of boids
 * @param canvasP 
 */
void initiateBoids(int WW, int WH, std::vector<std::unique_ptr<boid>>& boids, Canvas* canvasP) {
    // #pragma acc parallel loop independent collapse(1) num_gangs(numThreads) 
    for (int i = 0; i < boids.size(); ++i) {
        xp[i] = random_range(-WW / 2, WW / 2);
        yp[i] = random_range(-WH / 2, WH / 2);

        xv[i] = random_range(-1.0, 1.0);
        yv[i] = random_range(-1.0, 1.0);
        norm(&xv[i], &yv[i]);

        boids[i] = std::make_unique<boid>(xp[i], yp[i], i, canvasP);

        boids[i]->updateDirection(xv[i], yv[i]);
    }

    #ifdef DEBUG
    fprintf(stderr, "Initiate boids complete!\n");
    #endif
}

void testScreen(Canvas& canvas, unsigned numBoids) {
    const int WW = canvas.getWindowWidth(),
              WH = canvas.getWindowHeight();

    std::vector<std::unique_ptr<boid>> boids(numBoids);
    initiateBoids(WW, WH, boids, &canvas);
    // flushNewToOld();




    // canvas.setShowFPS(true);
    while (canvas.isOpen()) {
        canvas.sleep(); // This slows it down
        // printf("fps: %f\n", canvas.getFPS());

        // #pragma acc parallel loop independent num_gangs(numThreads)
        
        // #ifdef GPU
        // #pragma acc kernels
        // #pragma acc data copyin(bx[:numBoids], by[:numBoids]) copy(vx[:numBoids], vy[:numBoids])
        // #pragma acc loop independent
        // #else
        // #pragma acc parallel loop independent num_gangs(numThreads) collapse(1)
        // #endif
        // for (int i = 0; i < numBoids; ++i) {
        //     compute_new_heading(i);
        // }

        #ifdef GPU
        calcAllGPU::calculate_all_headings();
        #else
        calcAllMP::calculate_all_headings(numThreads);
        #endif



        // Update Positions
        #pragma acc parallel loop independent num_gangs(numThreads) collapse(1)
        for (int i = 0; i < numBoids; ++i) {
            xv[i] = xnv[i];
            yv[i] = ynv[i];
            xp[i] += xv[i] * dt;
            yp[i] += yv[i] * dt;
            
            // Wrap around screen coordinates
            if (xp[i] < -WW / 2) {
                xp[i] += WW;
            }
            else if (xp[i] >= WW / 2) {
                xp[i] -= WW;
            }

            if (yp[i] < -WH / 2) {
                yp[i] += WH;
            }
            else if (yp[i] >= WH / 2) {
                yp[i] -= WH;
            }


            boids[i]->updatePosition(xp[i], yp[i]);
            boids[i]->updateDirection(xv[i], yv[i]);

            // boids[i]->setColor(ColorFloat(omp_get_thread_num() / 6. + 0.1));
        }
    }

    
} 

int main (int argc, char* argv[]) 
{
    std::cout << "Hello world!" <<  std::endl;
    
    num = 512;
    numBoids = num;

    width = 1600;
    height = 900;
    /* Make space for the positions, velocities, and new velocities. */
	xp  = (float *) malloc(sizeof(float) * num);
	yp  = (float *) malloc(sizeof(float) * num);
	xv  = (float *) malloc(sizeof(float) * num);
	yv  = (float *) malloc(sizeof(float) * num);
	xnv = (float *) malloc(sizeof(float) * num);
	ynv = (float *) malloc(sizeof(float) * num);



    numThreads = atoi(argv[1]);
    printf("NumThreads: %d\n", numThreads);

    Canvas can(-1, -1, width, height, "Test Screen", BLACK);
    can.run(testScreen, numBoids);
    


    // #define TEST2
    #ifdef TEST2
    size_t size = 20000;
    std::vector<int> k(size, 1);
    std::vector<int> l(size, 3);

    #ifdef GPU
    #pragma acc kernels
    #pragma acc loop independent collapse(1)
    #else
    #pragma acc parallel loop independent collapse(1)
    #endif
    for (size_t i = 0; i < size; ++i) {
        // printf("Hello! Thread %d on %d\n", omp_get_thread_num(), i);
        // printf("Hello there! I'm hopefully threaded lol %d\n", i);
        k[i] += l[i];
    }

    for (size_t i = 0; i < size; ++i) {
        if (k[i] != 4)
            fprintf(stderr, "Incorrect %d", i);
    }
    #endif

    free(xp);
    free(yp);
    free(xv);
    free(yv);
    free(xnv);
    free(ynv);

    return 0;
}