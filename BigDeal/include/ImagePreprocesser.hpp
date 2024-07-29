#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <vector>

/**
 * @marble703
 * 
 * @brief 预处理图片
 * 
 * @param image 处理后的图片
 * 
 * @param origin_image 原图
 * @param gray_ 灰度图
 * @param binary_ 二值图
 * 
 * @param x_erode_kernal_size 腐蚀核的x大小
 * @param y_erode_kernal_size 腐蚀核的y大小
 * @param erode_kernal 腐蚀核
 * @param eroded_image 腐蚀后的图片
 * 
 * @param x_dilate_kernal_size 膨胀核的x大小
 * @param y_dilate_kernal_size 膨胀核的y大小
 * @param dilate_kernal 膨胀核
 * @param dilated_image 膨胀后的图片
 * 
 */
class ImagePreprocesser{
private:
    cv::Mat image;
    cv::Mat origin_image;
    cv::Mat gray, binary;

    int x_erode_kernal_size = 3;
    int y_erode_kernal_size = 3;
    cv::Mat erode_kernal ;

    int x_dilate_kernal_size = 7;
    int y_dilate_kernal_size = 7;
    cv::Mat dilate_kernal;

    cv::Mat eroded_image;
    cv::Mat dilated_image;

public:
    ImagePreprocesser() = default;
    ~ImagePreprocesser() = default;
    /**
     * @brief 构造函数
     * @param image 图片
     */
    ImagePreprocesser(cv::Mat image_);
    
    /**
     * @brief 处理接受的图片
     * @param image 图片
     * @return 处理后的图片
     */
    cv::Mat preprocess(cv::Mat image_);

    /**
     * @brief 处理存储的图片
     * @return 处理后的图片
     */
    cv::Mat preprocess();

    /**
     * @brief 显示处理过程, 用于调试
     * 
     * 显示原图、灰度图、二值图、腐蚀后的图片、膨胀后的图片
     */
    void show_process();
};
