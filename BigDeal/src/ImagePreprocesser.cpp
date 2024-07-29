#include "../include/ImagePreprocesser.hpp"

ImagePreprocesser::ImagePreprocesser(cv::Mat image_) {
    image = image_;
    origin_image = image_;
}

cv::Mat ImagePreprocesser::preprocess(cv::Mat image_) {
    //灰度化
    cv::cvtColor(image_, gray, cv::COLOR_BGR2GRAY);
    //二值化
    cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY); 

    cv::Mat erode_kernal = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(x_erode_kernal_size, y_erode_kernal_size));

    cv::Mat dilate_kernal = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(x_dilate_kernal_size, y_dilate_kernal_size));

    //腐蚀
    cv::erode(binary, eroded_image, erode_kernal);

    //膨胀
    cv::dilate(eroded_image, dilated_image, dilate_kernal);
	//cv::imshow("dilated_image", dilated_image);
	//cv::waitKey(0);

    return dilated_image;
}

cv::Mat ImagePreprocesser::preprocess(){
    return preprocess(image);
}

void ImagePreprocesser::show_process() {
    cv::imshow("origin_image", origin_image);
    cv::imshow("gray", gray);
    cv::imshow("binary", binary);
    cv::imshow("eroded_image", eroded_image);
    cv::imshow("dilated_image", dilated_image);
    cv::waitKey(0);
}