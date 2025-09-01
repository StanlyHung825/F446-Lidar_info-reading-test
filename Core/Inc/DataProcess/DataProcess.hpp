#ifndef DATAPROCESS_HPP
#define DATAPROCESS_HPP

#include "stm32f446xx.h"
#include "Lidar_LD06.hpp"
#include <cstring>

#ifdef __cplusplus
extern "C" {
#endif

#define PKG_LEN 47
#define CIRCULAR_BUFFER_SIZE PKG_LEN * 11

typedef struct {
    uint8_t buffer[CIRCULAR_BUFFER_SIZE];
    uint16_t head;
    uint16_t header_index;
    bool synced;
} DataBuffer;

void DataBuffer_Init(DataBuffer* buf);
void DataBuffer_Process(DataBuffer* buf, LiDARFrameTypeDef* lidarData);
void DataBuffer_UpdateHead(DataBuffer* buf);
bool DataBuffer_GetPacket(DataBuffer* buf, LiDARFrameTypeDef* lidarData, uint16_t header_index);
uint8_t FindHeaderIndex(uint8_t package[], uint8_t len);

#ifdef __cplusplus
}
#endif

#endif 