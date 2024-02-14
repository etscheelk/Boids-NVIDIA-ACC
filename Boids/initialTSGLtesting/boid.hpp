#ifndef BOID_H
#define BOID_H

#include <tsgl.h>

using namespace tsgl;

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
    boid(float x, float y, int index, Canvas* canvasP);

    /**
     * @brief Destroy the boid object
     * 
     */
    ~boid();

    void setColor(ColorFloat color);

    void updatePosition(float x, float y);

    void updateDirection(float velx, float vely);
};

#endif