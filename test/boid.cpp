#include "boid.hpp"

boid::boid(float x, float y, int index, Canvas* canvasP) {
    this->index = index;
    this->can = canvasP;
    arrow = new Arrow(
        x, y, 0, 
        20, 20, 
        0, 0, 0, 
        CYAN
    );
    can->add(arrow);
}

boid::~boid() {
    can->remove(arrow);
    delete arrow;
}

void boid::setColor(tsgl::ColorFloat color) {
    arrow->setColor(color);
}

void boid::updatePosition(float x, float y) {
    arrow->setCenter(x, y, 0);
}

void boid::updateDirection(float velx, float vely) {
    float yaw = atan(vely / velx) * 180. / PI;
    if (velx > 0) {
        yaw += 180;
    }
    arrow->setYaw(yaw);
}