#include "mainpp.hpp"

#include "stm32f446xx.h"
#include "DataProcess.hpp"
#include "extern_variable.h"
#include "Lidar_LD06.hpp"
#include "polar2cartesian.hpp"
#include <vector>
#include <cstring>

DataBuffer dataBuf;
bool BufferFlag = 0;
int header_index = 0;
LiDARFrameTypeDef lidarData;
uint16_t prev_timestamp = 0;
std::vector<CartesianPointStructDef> ScanPointsVector;
int vector_size = 0;


void main_function(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_UART_Receive_DMA(&huart1, dataBuf.buffer, CIRCULAR_BUFFER_SIZE);
    DataBuffer_Init(&dataBuf);
    while(true){

    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    DataBuffer_Process(&dataBuf, &lidarData);
    HAL_UART_Receive_DMA(&huart1, dataBuf.buffer, CIRCULAR_BUFFER_SIZE);
  }
}