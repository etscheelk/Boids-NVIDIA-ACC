#include <iostream>
#include <stdlib.h>

#include <tsgl.h>
#include "myVector.cpp"

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
float *bx;
float *by;
float *vx;  
float *vy;
float *nvx; 
float *nvy;


size_t numBoids = 128;


Vector2 Rule1GroupUp(int boidIndex);
Vector2 Rule2Avoid(int boidIndex);
Vector2 Rule3Align(int boidIndex);

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
        printf("%d\n", sizeof(can));
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
    for (int index = 0; index < boids.size(); ++index) {
        float x = rand() % WW - WW / 2; bx[index] = x;
        float y = rand() % WH - WH / 2; by[index] = y;
        boids[index] = std::make_unique<boid>(x, y, index, canvasP);

        float velx = rand() % 100 - 50; vx[index] = velx;
        float vely = rand() % 100 - 50; vy[index] = vely;

        boids[index]->updateDirection(velx, vely);
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
    

    // boid* b = new boid(0, 0, 0, canvas);

    // Background* back = canvas.getBackground();
    // back->drawSquare(0, 0, 0, 50, 0, 0, 0, WHITE, true);

    // back->drawPixel(0, 0, ColorInt(120, 120, 120));




    /* NOTES:
        Background has (0, 0) in the center for some reason

        Whenever I want to flush the updated arrays to the background, 
        I should do this on the CPU. 

        Due to what is likely the mutex lock on editing the background pixels,
        updating the background pixels is simply slower when threaded. 
    */
    // #define TEST
    #ifdef TEST
    int w = WW / 2;
    int h = WH / 2;
    double startT = omp_get_wtime();
    int i, j;
    #ifdef GPU
    #pragma acc kernels
    #pragma acc loop independent collapse(2) private(i, j)
    #else
    #pragma acc parallel loop independent collapse(2) private(i, j) num_gangs(numThreads)
    #endif
    for (i = -h; i < h; ++i) {
        for (j = -w; j < w; ++j) {
            int color = 240;

            #ifndef GPU
            color = omp_get_thread_num() * 25 + 25;
            #endif

            back->drawPixel(j, i, ColorInt(color));
            // canvas.sleepFor(0.000005);
        }
    }
    printf("Duration: %lf\n", omp_get_wtime() - startT);
    #endif


    while (canvas.isOpen()) {
        canvas.sleep(); // This slows it down

        // #pragma acc parallel loop independent num_gangs(numThreads)
        
        #pragma acc kernels
        #pragma acc data copyin(bx[:numBoids], by[:numBoids]) copy(vx[:numBoids], vy[:numBoids])
        #pragma acc loop independent
        for (int i = 0; i < numBoids; ++i) {
            printf("Inside fork\n");
            // printf("%d\n", omp_get_thread_num());
            Vector2 r1 = Rule1GroupUp(i);

            if (i == 0) {
                printf("r1 = (%f, %f)\n", r1.x, r1.y);
            }


            Vector2 r2 = Rule2Avoid(i);


            Vector2 r3 = Vector2();
            // Vector2 r3 = Rule3Align(i);

            vx[i] += r1.x + r2.x + r3.x;
            vy[i] += r1.y + r2.y + r3.y;
        }

        printf("Speed of index 0: (%f, %f)\n", vx[0], vy[0]);


        float a = omp_get_wtime();

        // #pragma acc parallel loop independent num_gangs(numThreads)
        for (int i = 0; i < boids.size(); ++i) {
            bx[i] += 0.005 * vx[i];
            by[i] += 0.005 * vy[i];
        }

        float b = omp_get_wtime();

        // #pragma acc parallel loop independent num_gangs(numThreads)
        for (int i = 0; i < boids.size(); ++i) {
            boids[i]->updatePosition(bx[i], by[i]);
            boids[i]->updateDirection(vx[i], vy[i]);
        }

        float c = omp_get_wtime();

        printf("Time to update positions: %f\nTime to update boid objects: %f\nTotal time elapsed: %f\n\n", b - a, c - b, c - a);

        // for (int iter = 0; iter < 1000; ++iter) {
            /*
            pragma compiler directives
            for each boid i
                for each boid j
                    if j is i continue

                    do the boid calculation stuff, new position and velocity stored in new arrays

            
            update position prev position to new position

            have CPU update screen
            */

        // }
    }

    
} 

Vector2 Rule1GroupUp(int boidIndex) {
    Vector2 p(0., 0.);
    
    for (int i = 0; i < numBoids; ++i) {
        if (i == boidIndex) continue;

        p.x += bx[boidIndex];
        p.y += by[boidIndex];
    }

    p *= 1. / (numBoids - 1); 

    if (boidIndex == 0) {
        printf("(%f, %f)\n", p.x, p.y);
    }

    return Vector2(p.x - bx[boidIndex], p.y - by[boidIndex]) * 0.01;
}

Vector2 Rule2Avoid(int boidIndex) {
    Vector2 c(0., 0.);

    for (int i = 0; i < numBoids; ++i) {
        if (i == boidIndex) continue;

        if (distance(bx[boidIndex], by[boidIndex], bx[i], by[i]) < 120) {
            c = c - Vector2(bx[boidIndex], by[boidIndex]) + Vector2(bx[i], by[i]);
        }
    }
    
    return c;
}

Vector2 Rule3Align(int boidIndex) {
    Vector2 z(0., 0.);

    for (int i = 0; i < numBoids; ++i) {
        if (i == boidIndex) continue;

        z += Vector2(vx[i], vy[i]);
    }

    z *= 1. / (numBoids - 1);
    
    return Vector2(z.x - vx[boidIndex], z.y - vy[boidIndex]) * 0.05;
}

int main (int argc, char* argv[]) {
    std::cout << "Hello world!" <<  std::endl;
    
    numBoids = 1024;
    bx  = (float*) malloc(numBoids * sizeof(float));
    by  = (float*) malloc(numBoids * sizeof(float));
    vx  = (float*) malloc(numBoids * sizeof(float));
    vy  = (float*) malloc(numBoids * sizeof(float));
    nvx = (float*) malloc(numBoids * sizeof(float));
    nvy = (float*) malloc(numBoids * sizeof(float));

    numThreads = atoi(argv[1]);
    printf("NumThreads: %d\n", numThreads);

    Canvas can(-1, -1, 1600, 900, "Test Screen", BLACK);
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

    free(bx);
    free(by);
    free(vx);
    free(vy);
    free(nvx);
    free(nvy);

    return 0;
}