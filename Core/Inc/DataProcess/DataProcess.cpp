#include "DataProcess.hpp"

void DataBuffer_Init(DataBuffer* buf) {
    memset(buf->buffer, 0, CIRCULAR_BUFFER_SIZE);
    buf->head = 0;
    buf->header_index = 0;
    buf->synced = false;
}

void DataBuffer_Process(DataBuffer* buf, LiDARFrameTypeDef* lidarData) {
    buf->header_index = buf->head;
    if(buf->synced == false){
        DataBuffer_UpdateHead(buf);
        buf->header_index = buf->head;
    } else {
      while(DataBuffer_GetPacket(buf, lidarData, buf->header_index))
      {
          buf->header_index += PKG_LEN;
      }
    }
}

void DataBuffer_UpdateHead(DataBuffer* buf) {
    buf->head = FindHeaderIndex(buf->buffer, PKG_LEN);
    buf->synced = (buf->head != -1);
}

bool DataBuffer_GetPacket(DataBuffer* buf, LiDARFrameTypeDef* lidarData, uint16_t header_index) {
    if (buf->buffer[header_index] == HEADER && header_index + PKG_LEN <= CIRCULAR_BUFFER_SIZE) {
        if (CalCRC8(buf->buffer, PKG_LEN - 1, header_index) == buf->buffer[header_index + PKG_LEN - 1]) {
            *lidarData = AssignValues(buf->buffer, header_index);
            return true;
        }
    } else if (buf->buffer[header_index] == HEADER && header_index + PKG_LEN > CIRCULAR_BUFFER_SIZE) {
        buf->synced = true;
        return false;
    } else {
        buf->synced = false;
        return false;
    }
}

uint8_t FindHeaderIndex(uint8_t buf[], uint8_t len) {
    for (int i = 0; i < len; i++) {
        if (buf[i] == HEADER) {
            return i;
        }
    }
    return -1; // Header not found
}