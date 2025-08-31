#ifndef POLAR2CARTESIAN_HPP
#define POLAR2CARTESIAN_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include <cmath>
#include "stm32f4xx_hal.h"

typedef struct{
    float x;
    float y;
    uint16_t timestamp;
} CartesianPointStructDef;

void polar2Cartesian(float radius, uint16_t angle, CartesianPointStructDef* cartesianPoint);

#ifdef __cplusplus
}
#endif


#endif // POLAR2CARTESIAN_HPP