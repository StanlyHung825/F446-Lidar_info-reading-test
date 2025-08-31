#include "mainpp.hpp"

#include "stm32f446xx.h"
#include "extern_variable.h"
#include "Lidar_LD06.hpp"
#include "polar2cartesian.hpp"
#include <vector>
#include <cstring>

uint8_t rxBuffer[84]; // 3 times of rawData
// uint8_t rxBuffer1[47];
// int header_index1 = 0;
// uint8_t rxBuffer2[47];
// int header_index2 = 0;
// uint8_t rawData[47];
bool BufferFlag = 0;
int header_index = 0;
LiDARFrameTypeDef lidarData;
uint16_t prev_timestamp = 0;
std::vector<CartesianPointStructDef> ScanPointsVector;
int vector_size = 0;


void main_function(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_UART_Receive_DMA(&huart1, rxBuffer, sizeof(rxBuffer));
    while(true){

        
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    header_index = 0;
    while (header_index != -1) {
      header_index = FindHeaderIndex(rxBuffer, sizeof(rxBuffer), header_index);
      if(header_index != -1 && header_index + 47 <= sizeof(rxBuffer)){
        if (rxBuffer[header_index + 46] == CalCRC8(rxBuffer, 46, header_index)) {
          lidarData = AssignValues(rxBuffer, header_index);
          StoreScanData(ScanPointsVector, lidarData);
        }
      }
    }
    vector_size = ScanPointsVector.size();
    HAL_UART_Receive_DMA(&huart1, rxBuffer, sizeof(rxBuffer));
  }
}


// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//     if (huart->Instance == USART1){
//         header_index = 0;
//         if(BufferFlag){
//             header_index1 = FindHeaderIndex(rxBuffer1, sizeof(rxBuffer1), 0);
//             memcpy(rawData, LidarDataProcess(rxBuffer1, rxBuffer2, header_index1), 47);
//             if (rawData[46] == CalCRC8(rawData, 46, 0)) {
//                 lidarData = AssignValues(rawData, 0);
//                 StoreScanData(ScanPointsVector, lidarData);
//             }
//             BufferFlag = 0;
//             HAL_UART_Receive_DMA(&huart1, rxBuffer2, sizeof(rxBuffer2));
//         }else{
//             header_index2 = FindHeaderIndex(rxBuffer2, sizeof(rxBuffer2), 0);
//             memcpy(rawData, LidarDataProcess(rxBuffer2, rxBuffer1, header_index2), 47);
//             if (rawData[46] == CalCRC8(rawData, 46, 0)) {
//                 lidarData = AssignValues(rawData, 0);
//                 StoreScanData(ScanPointsVector, lidarData);
//             }
//             BufferFlag = 1;
//             HAL_UART_Receive_DMA(&huart1, rxBuffer1, sizeof(rxBuffer1));
//         }
//     }
// }