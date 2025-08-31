#include "polar2cartesian.hpp"

void polar2Cartesian(float radius, uint16_t angle, CartesianPointStructDef* cartesianPoint) {
    cartesianPoint->x = radius * cos(angle);
    cartesianPoint->y = radius * sin(angle);
}