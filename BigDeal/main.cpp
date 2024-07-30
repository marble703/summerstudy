#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <time.h>

#include "include/Armor.hpp"
#include "include/ImagePreprocesser.hpp"
#include "include/KalmanFilter.hpp"
#include "include/Detect.hpp"

int main(){
    cv::VideoCapture capture;
    cv::Mat frame;
    std::vector<int64_t> frame_time;
    capture.open("/media/chen/Data/programme/Visual/SummerStudy/BigDeal/source/7月14日.mp4");

        while (capture.read(frame)) {
            std::cout<< "frame start" <<frame_time.size()<< std::endl;
            auto start = std::chrono::system_clock::now();


            cv::Mat image = frame;
            Detect detector(frame);
            // 输入true单帧播放;输入true显示图像
            detector.detect(false, true);
            cv::imshow("frame", frame);


            std::cout << "\nframe finish\n\n\n" << std::endl;
            auto end = std::chrono::system_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); 
            std::cout <<"frame time: "<< duration << std::endl;
            frame_time.push_back(duration);
            cv::waitKey(1);
        }

        double frame_time_t = 0;
        for (int i = 0; i < frame_time.size(); i++) {
            frame_time_t += frame_time[i];
        }

        std::cout << "average time: " << frame_time_t / frame_time.size() << std::endl;
        std::cout << "DFPS: " << 1000 / (frame_time_t / frame_time.size()) << std::endl;
        double radiuses_t = 0;

    return 0;
}