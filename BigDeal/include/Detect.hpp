#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "Armor.hpp"
#include "number_classifier.hpp"
#include "ImagePreprocesser.hpp"

/**
 * @brief 装甲板检测类
 * @note 使用了一些丑陋的方法
 */
class Detect {
public:
    /**
     * @brief 构造函数
     * @param image 图片
     * @note 传入图片
     */
    Detect(cv::Mat image);
    /**
     * @brief 检测函数
     * @param wait 是否等待
     * @param show 是否显示
     */
    int detect(bool wait = false, bool show = true);
    std::vector<cv::Point2f> get_points();
    bool get_find();
private:
    cv::Mat image;
    std::vector<cv::Point2f> points;
    bool find;

    //轮廓
    std::vector<std::vector<cv::Point>> contours;

    cv::RotatedRect minRect;

    void draw_lines_numbers(std::vector<cv::Point2f> points, std::string classification_result);
};