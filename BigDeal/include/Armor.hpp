//TODO: 添加装甲板类型识别

// 类中所有坐标按照lb, lt, rt, rb的顺序存储

#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @brief 存储灯条位置
 * 
 * @param bottom 灯条底部的坐标
 * @param top 灯条顶部的坐标
 */
struct Light{
    cv::Point2f bottom;
    cv::Point2f top;
};

/**
 * @brief 存储装甲板大小
 */
enum ArmorType {
        SMALL,
        LARGE
};

class Armor{
public:
    cv::Mat image;
    cv::Mat number_image;

    std::string classification_result;
    std::string number;

    enum ArmorType {
        SMALL,
        LARGE
    } type;

    Light left_light;
    Light right_light;

    double classification_confidence;


    /**
     * @brief 自动纠正坐标
     * @note 有未知风险，建议不使用
     */
    void self_correction();

    /**
     * @brief 自动矫正左右灯条的位置
     * @param points 四个点位置
     */
    void self_correction_lr_lights(std::vector<cv::Point2f> points);


    /**
     * @brief 构造函数
     * @param image 图片
     */
    Armor(cv::Mat image);

    /**
     * @brief 构造函数
     * @param image 图片
     * @param left_bottom, left_top, right_top, right_bottom 四个点
     */
    Armor(cv::Mat image, cv::Point2f left_bottom, cv::Point2f left_top, cv::Point2f right_top, cv::Point2f right_bottom);

    /**
     * @brief 构造函数
     * @param image 图片
     * @param points 四个点
     */
    Armor(cv::Mat image, std::vector<cv::Point2f> points);

    std::string get_classification_result();

    double get_classification_confidence();
};