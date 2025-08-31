#include "Lidar_LD06.hpp"

uint16_t angle_per_step;
bool flag = 0;

uint8_t* LidarDataProcess(uint8_t* buffer1, uint8_t* buffer2, int header_index){
    static uint8_t processed_data[47];
    int memory_left = 47 - header_index;
    for(int i = 0; i < memory_left; i++){
        processed_data[i] = buffer1[header_index + i];
    }
    for(int i = 0; i < header_index; i++){
        processed_data[memory_left + i] = buffer2[i];
    }
    return processed_data;
}


uint8_t CalCRC8(uint8_t package[], uint8_t len, int header_index)
{
    uint8_t crc = 0;
    uint16_t i;
    for(i = 0; i < len; i++)
    {
        crc = CrcTable[(crc ^ package[header_index + i]) & 0xff];
    }
    return crc;
}

LiDARFrameTypeDef AssignValues(uint8_t package[], int header_index)
{
    LiDARFrameTypeDef frame;
    frame.header = package[header_index];
    frame.ver_len = package[header_index + 1];
    frame.speed = (package[header_index + 3] << 8) | package[header_index + 2];
    frame.start_angle = (package[header_index + 5] << 8) | package[header_index + 4];
    frame.end_angle = (package[header_index + 43] << 8) | package[header_index + 42];
    frame.timestamp = (package[header_index + 45] << 8) | package[header_index + 44];
    frame.crc8 = package[header_index + 46];

    if(frame.end_angle < frame.start_angle){
        angle_per_step = (36000 - frame.start_angle + frame.end_angle) / (POINT_PER_PACK - 1);
    }else{
        angle_per_step = (frame.end_angle - frame.start_angle) / (POINT_PER_PACK - 1);
    }
    

    for(int i = 0; i < POINT_PER_PACK; i++)
    {
        frame.point[i].distance = (package[header_index + 7 + i * 3] << 8) | package[header_index + 6 + i * 3];
        frame.point[i].confidence = package[header_index + 8 + i * 3];
        frame.point[i].angle = (frame.start_angle) + (i * angle_per_step);
        if(frame.point[i].angle >= 3600)
        {
            frame.point[i].angle -= 3600;
        }
    }
    
    
    return frame;
}

int FindHeaderIndex(uint8_t package[], uint8_t len, int start_index)
{
    for(int i = start_index; i < len; i++)
    {
        if(package[i] == HEADER)
        {
            return i;
        }
    }
    return -1;
}

void LidarVelCtrl(LiDARFrameTypeDef lidarData, uint8_t speed)
{
    static float previous_error = 0;
    static float integral = 0;
    static const float Kp = 1.0f;  // Proportional gain
    static const float Ki = 0.1f;  // Integral gain
    // static const float Kd = 0.05f; // Derivative gain

    float current_speed = lidarData.speed;
    float error = speed - current_speed;

    if(integral > 10) integral = 10; // Anti-windup
    else integral += error;

    // float derivative = error - previous_error;

    float output = Kp * error + Ki * integral;

    if(output > 1000) output = 1000;
    else if(output < 0) output = 0;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, output);


    previous_error = error;
}

void StoreScanData(std::vector<CartesianPointStructDef>& scanPoints, LiDARFrameTypeDef lidarData)
{
    CartesianPointStructDef scan_point;

    if(scanPoints.empty()){
        for(int i = 0; i < POINT_PER_PACK; i++){
            scan_point.x = lidarData.point[i].distance * cos(lidarData.point[i].angle / 100.0 * M_PI / 180.0);
            scan_point.y = lidarData.point[i].distance * sin(lidarData.point[i].angle / 100.0 * M_PI / 180.0);
            scan_point.timestamp = lidarData.timestamp;
            scanPoints.push_back(scan_point);
        }
    } else {
        // if(abs(lidarData.timestamp - scanPoints.front().timestamp) > 100){
        //     if(scanPoints.size() > 450)
        //         scanPoints.erase(scanPoints.begin(), scanPoints.begin() + 450-1);
        //     else 
        //         scanPoints.clear();
        //     flag = 1;
        // } else {
        //     flag = 0;
        // }
        if (scanPoints.size() > 900)
            scanPoints.erase(scanPoints.begin(), scanPoints.begin() + 450-1);
         // flag = 1;
        for(int i = 0; i < POINT_PER_PACK; i++){
                scan_point.x = lidarData.point[i].distance * cos(lidarData.point[i].angle / 100.0 * M_PI / 180.0);
                scan_point.y = lidarData.point[i].distance * sin(lidarData.point[i].angle / 100.0 * M_PI / 180.0);
                scan_point.timestamp = lidarData.timestamp;
                scanPoints.push_back(scan_point);
        }
    }
}