#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include "Uart.hpp"
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>   

using namespace cv;
using namespace std;

const float REAL_KFS_HEIGHT = 0.35f;   // KFS边长（米）
const float CAMERA_HEIGHT = 0.6f;     // 相机离地高度（米） 选取二区高的方块中心离地高度
const float GRASP_DISTANCE = 0.70f;    // 抓取距离阈值（米）
const float CENTER_OFFSET_RATIO = 0.9f;
float squareSize = 26.0f; // 单位：毫米


int main() {

    Uart uart;
    bool serial_ok = false;
    try {
        uart.InitSerialPort("/dev/ttyUSB0");  
        serial_ok = true;
        // cerr << "[串口] 打开成功" << endl;
    } catch (...) {
        cerr << "[串口] 打开失败" << endl;
    }


    // 1. 加载标定数据
    FileStorage fs("calibration_data.yml", FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "[错误] 找不到 calibration_data.yml！请先运行 ./calibrate" << endl;
        return -1;
    }
    Mat K, D;
    fs["camera_matrix"] >> K;
    fs["distortion_coefficients"] >> D;
    fs.release();

    if (K.empty() || K.rows != 3) {
        cerr << "[错误] 内参矩阵无效" << endl;
        return -1;
    }
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    if (fx < 1e-6) {
        cerr << "[错误] 焦距 fx 无效" << endl;
        return -1;
    }
    cerr << "[C++] 标定加载成功，fx=" << fx << ", cx=" << cx << ", cy=" << cy << endl;



    // 摄像头
    // VideoCapture cap("/dev/video4");



    // 接收传输的坐标
    string line;
    while (getline(cin, line)) {
        if (line.empty()) continue;

        stringstream ss(line);
        int x1, y1, x2, y2, cls;
        float angle; // 姿态角
        string interference_str = "0"; // 这里利用字符串可以轻松存多个干扰， 后面用 stoi 将字符串解析为整数，通过串口传输

        
        if (!(ss >> x1 >> y1 >> x2 >> y2 >> cls >> angle)) {
            cerr << "[C++] 解析失败，跳过: " << line << endl;
            continue;
        }
        
        if (!(ss >> interference_str)) {
            interference_str = "0";
        }

        // 打印干扰位置（使用 cerr 以便调试，也可改为 cout 供串口）

        // 区别：cout会被下一个接住， cerr只在终端，例：
        //  ./yolo_inference > result.txt  # 只把 cout 的内容存进文件，cerr 依然打印在屏幕
        // cerr << "[干扰位置] " << interference_str << endl;



        // 坐标有序
        if (x1 > x2) swap(x1, x2);
        if (y1 > y2) swap(y1, y2);

        int pixel_h = y2 - y1;
        int pixel_w = x2 - x1;
        if (pixel_h < 2 || pixel_w < 2) {
            cerr << "检测框太小，跳过" << endl;
            continue;
        }

        // ---- 测距 --

        // 真实物体高度 H (米)  ——>  成像高度 h (像素)
        // 物体距离 Z (米)      ——>  焦距 f (像素)  （这里 f 是 fx）
        // 相似三角形，H/Z = h/f
        // 除以标定的时候带来的系数 squareSize *100


        float Z = (REAL_KFS_HEIGHT * fx) / (float)pixel_h; // pixel_h 与角度有关，需要在调整，或者改变框的方法，与俯仰角有关
        // 或者不用距离来描述

        // ---- 去畸变 
        Point2f center((x1 + x2) / 2.0f, (y1 + y2) / 2.0f);
        vector<Point2f> src = {center}, dst;
        try {
            undistortPoints(src, dst, K, D, noArray(), K);
        } catch (...) {
            cerr << "去畸变异常，跳过" << endl;
            continue;
        }
        if (dst.empty()) continue;
        Point2f p = dst[0];
        // p 是去畸变后的像素中心点坐标x，y

        // ---- 计算三维坐标 ----
        float X = ((p.x - cx) / fx) * Z;
        float Y = ((p.y - cy) / fy) * Z;
        float KFS_HEIGHT = CAMERA_HEIGHT - Y;

        // ---- 方位角 ----
        float azimuth_deg = atan2(X, Z) * 180.0 / CV_PI;

        // ---- 判断是否到达 ----
        bool dist_ok = (Z > 0.02f && Z < GRASP_DISTANCE);
        bool center_ok = (abs(p.x - cx) / (640.0f / 2.0f) < CENTER_OFFSET_RATIO);

        // ---- 输出结果----
        if (dist_ok && center_ok) {
            cout << "[到达] X=" << X << "m, Z=" << Z << "m, 高度=" << KFS_HEIGHT << "m, "
                << "°, 方位角=" << azimuth_deg << "°, 干扰位置=" << interference_str << endl;
        } else {
            cout << "[跟踪] Z=" << Z << "m, 高度=" << KFS_HEIGHT << "m, "
                << "°, 方位角=" << azimuth_deg << "°, 干扰位置=" << interference_str << endl;
        }
        cout.flush();
    

    if (serial_ok) {
    // 清空写缓冲区
    memset(uart.writeBuff, 0, sizeof(uart.writeBuff));

    // 按照资料协议构造帧（? ! S X Y Z !）（见夏令营资料串口协议格式）
    uart.writeBuff[0] = '?';
    uart.writeBuff[1] = '!';    // 校验位
    uart.writeBuff[2] = 'S';   // 命令字

    // 发送 X（米转毫米，存为 uint16_t）
    uint16_t X_mm = (uint16_t)(X * 1000);
    uint16_t Z_mm = (uint16_t)(Z * 1000);
    // uint8_t flag = 1;  // 自定义标志

    memcpy(&uart.writeBuff[3], &X_mm, 2);   // 偏移3-4
    memcpy(&uart.writeBuff[5], &Z_mm, 2);   // 偏移5-6
    // uart.writeBuff[7] = (uint8_t)interference_str;       // 偏移7
    uart.writeBuff[7] = (uint8_t)stoi(interference_str);
    uart.writeBuff[8] = '!';                // 帧尾  长度根据实际调整

    // 调用写函数
    uart.WriteBuffer();
    uart.ShowWriteBuff(); 
    }

    }
    cerr << "[C++] 输入流结束，正常退出" << endl;
    return 0;
}
