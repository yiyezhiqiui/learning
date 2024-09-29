#ifndef VIDEO_HPP
#define VIDEO_HPP

#include <iostream>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>

#include "./quater_pos.hpp"

class Process
{
public:

    // Odom到Gimabl
    position gim_translation;
    quaternion gim_rotation;
    // Gimbal到Camera
    position cam_translation;
    quaternion cam_rotation;
    // 结果
    position res_position;
    quaternion res_quaternion;

    Process(double gimtranslation[3], double gimrotation[4], double camtranslation[3], double camrotation[4])
        : gim_translation(gimtranslation[0], gimtranslation[1], gimtranslation[2]),
          gim_rotation(gimrotation[3], gimrotation[0], gimrotation[2], gimrotation[3]),

          cam_translation(camtranslation[0], camtranslation[1], camtranslation[2]),
          cam_rotation(camrotation[3], camrotation[0], camrotation[2], camrotation[3]),

          res_position(0, 0, 0),
          res_quaternion(0, 0, 0, 0)
    {
    }

    ~Process() {};
};

void picture_process(cv::Mat &frame,Process &process);
void kalman(position &original_position, double w, double r, double yaw);
cv::Mat invoke_armor_visual(cv::Mat frame,cv::Point2f (*points)[4],int& group_cnt,int &type);


#endif
