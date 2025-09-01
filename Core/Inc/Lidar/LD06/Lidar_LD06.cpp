#include "Lidar_LD06.hpp"

uint16_t angle_per_step;
bool flag = 0;


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