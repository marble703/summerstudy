#include <opencv2/opencv.hpp>
#include <vector>

#include "../include/Armor.hpp"
#include "../include/ImagePreprocesser.hpp"

Armor::Armor(cv::Mat image){
    type = ArmorType::SMALL;
    number_image = image;
}

Armor::Armor(cv::Mat image, cv::Point2f left_bottom, cv::Point2f left_top, cv::Point2f right_top, cv::Point2f right_bottom){
    std::vector<cv::Point2f> points;
    points.push_back(left_bottom);
    points.push_back(left_top);
    points.push_back(right_top);
    points.push_back(right_bottom);
    
    Armor::self_correction_lr_lights(points);
    number_image = image;
}

Armor::Armor(cv::Mat image, std::vector<cv::Point2f> points){
    Armor::self_correction_lr_lights(points);
    number_image = image;
}

void Armor::self_correction_lr_lights(std::vector<cv::Point2f> points){
    if(points[0].x < points[2].x){
        cv::Point2f point = points[0];
        points[0] = points[2];
        points[2] = point;

        point = points[1];
        points[1] = points[3];
        points[3] = point;
    }

    left_light.bottom = points[3];
    left_light.top = points[2];
    right_light.top = points[0];
    right_light.bottom = points[1];
}

std::string Armor::get_classification_result(){
    return classification_result;
}

double Armor::get_classification_confidence(){
    return classification_confidence;
}

void Armor::self_correction(){
    if(left_light.bottom.y < left_light.top.y){
        cv::Point2f point = left_light.bottom;
        left_light.bottom = left_light.top;
        left_light.top = point;
    }

    if(right_light.bottom.y < right_light.top.y){
        cv::Point2f point = right_light.bottom;
        right_light.bottom = right_light.top;
        right_light.top = point;
    }

    if(right_light.top.x < left_light.top.x){
        cv::Point2f point = right_light.top;
        right_light.top = left_light.top;
        left_light.top = point;
    }

    if(right_light.bottom.x < left_light.bottom.x){
        cv::Point2f point = right_light.bottom;
        right_light.bottom = left_light.bottom;
        left_light.bottom = point;
    }
}