#ifndef EXTERN_VARIABLE_H
#define EXTERN_VARIABLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

#ifdef __cplusplus
}
#endif

#endif /* EXTERN_VARIABLE_H */