#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "Armor.hpp"
#include "number_classifier.hpp"
#include "ImagePreprocesser.hpp"

class Detect {
public:
    Detect(cv::Mat image);
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

    void draw_lines_numbers(std::vector<cv::Point2f> points, std::vector<Armor> armors);
};