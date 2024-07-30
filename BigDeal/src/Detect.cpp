#include "../include/Detect.hpp"

Detect::Detect(cv::Mat image_){
    image = image_;
}

int Detect::detect(bool wait, bool show){
    if (image.empty()) {
            std::cerr << "Failed to read image." << std::endl;
            return -1;
        }
    cv::Mat origin_image = image.clone();
    ImagePreprocesser imagepreprocesseror = ImagePreprocesser(image);
    cv::Mat processed_image = imagepreprocesseror.preprocess();

    // 查找轮廓
    cv::findContours(processed_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int flag = 0;
    std::vector<int> flags;
    std::vector<double> angles;
    std::vector<double> areas;

    for (int i = 0; i < contours.size(); i++) {
        // 对每个轮廓计算最小外接旋转矩形
        minRect = cv::minAreaRect(contours[i]);

        // 获取矩形的宽度和高度和角度
        double width = minRect.size.width;
        double height = minRect.size.height;
        double angle = minRect.angle;
        double area = minRect.size.area();
        
        angles.push_back(angle);
        areas.push_back(area);

        // 判断是长条方向
        if (width > height) {
            flag += 1;
            flags.push_back(1);

        }
        else{
            flag -= 1;
            flags.push_back(-1);

        }
    }

    //排除非装甲板灯条
    for (int i = 0; i < contours.size(); i++){
        cv::RotatedRect minRect = cv::minAreaRect(contours[i]);
        float width = minRect.size.width;
        float height = minRect.size.height;

        //删除条件：1.横着的灯条，2.灯条的宽高比

        int longer = width > height ? width : height;
        int shorter = width > height ? height : width;
        if ((longer / shorter > 5 || longer / shorter < 2)
//                || (width * height < 400)
            ){
            contours.erase(contours.begin() + i);
            flags.erase(flags.begin() + i);
            angles.erase(angles.begin() + i);
        }
    }

    // 没有灯条直接返回
    if(contours.size() < 1){
        return 0;
    }

    for (int i = 0; i < contours.size(); i++){
        minRect = cv::minAreaRect(contours[i]);
        if(show){
            cv::Point2f rectPoints[4];
            minRect.points(rectPoints);
            for (int j = 0; j < 4; j++) {
                cv::line(image, rectPoints[j], rectPoints[(j+1)%4], cv::Scalar(0,255,0), 2);
            }
        }
    }

    // 使用两个外接矩形的短边中点作为边界点
    std::vector<cv::Point2f> rectPointsVec;

    for (int i = 0; i < contours.size(); i++) {
        cv::RotatedRect minRect = cv::minAreaRect(contours[i]);
        cv::Point2f rectPoints[4];
        minRect.points(rectPoints);

        for (int  i = 0; i < 4; i++) {
            rectPointsVec.push_back(rectPoints[i]);
        }
    }

    // 将灯条从左到右排序并组合，基于每四个点的x坐标平均值
    for (int i = 0; i < rectPointsVec.size() / 4 - 1; i++) {
        for (int j = i + 1; j < rectPointsVec.size() / 4; j++) {
            // 计算每组x坐标平均值
            float avgX_i = (rectPointsVec[i * 4].x + rectPointsVec[i * 4 + 1].x + rectPointsVec[i * 4 + 2].x + rectPointsVec[i * 4 + 3].x) / 4;
            float avgX_j = (rectPointsVec[j * 4].x + rectPointsVec[j * 4 + 1].x + rectPointsVec[j * 4 + 2].x + rectPointsVec[j * 4 + 3].x) / 4;

            // 交换灯条，同时交换角度(面积等后面用不到，所以这里没交换)
            if (avgX_i > avgX_j) {
                for (int k = 0; k < 4; k++) {
                    cv::Point2f temp = rectPointsVec[i * 4 + k];
                    rectPointsVec[i * 4 + k] = rectPointsVec[j * 4 + k];
                    rectPointsVec[j * 4 + k] = temp;
                }
                double temp_angle = angles[i];
                angles[i] = angles[j];
                angles[j] = temp_angle;
            }
        }
    }

    //识别装甲板
    std::vector<std::vector<cv::Point2f>> points_comb;
    for (int i = 0; i < contours.size() - 1; i++){
        cv::RotatedRect minRect0 = cv::minAreaRect(contours[i]);
        cv::RotatedRect minRect1 = cv::minAreaRect(contours[i + 1]);

        //判断条件：角度差小于5度，或者角度和与90度的差小于5度，即接近平行, 且宽高比在1到2之间
        if ((abs(angles[i] - angles[i + 1]) < 5) || 
            (abs(angles[i] + angles[i + 1] - 90)) < 5
            ){
            std::vector<cv::Point2f> points;
            std::vector<cv::Point> temp_points;

            cv::Point2f p1;
            cv::Point2f p2;
            cv::Point2f p3;
            cv::Point2f p4;

            temp_points.push_back(rectPointsVec[i * 4]);
            temp_points.push_back(rectPointsVec[i * 4 + 1]);
            temp_points.push_back(rectPointsVec[i * 4 + 2]);
            temp_points.push_back(rectPointsVec[i * 4 + 3]);

            temp_points.push_back(rectPointsVec[i * 4 + 4]);
            temp_points.push_back(rectPointsVec[i * 4 + 5]);
            temp_points.push_back(rectPointsVec[i * 4 + 6]);
            temp_points.push_back(rectPointsVec[i * 4 + 7]);

            int flag_i = abs(temp_points[0].y - temp_points[1].y) < abs(temp_points[1].y - temp_points[2].y) ? 1 : 0;
            int flag_j = abs(temp_points[4].y - temp_points[5].y) < abs(temp_points[5].y - temp_points[6].y) ? 1 : 0;

            if(flag_i){
                p1 = (temp_points[0] + temp_points[1]) / 2;
                p2 = (temp_points[2] + temp_points[3]) / 2;
            }
            else{
                p1 = (temp_points[1] + temp_points[2]) / 2;
                p2 = (temp_points[0] + temp_points[3]) / 2;
            }
            if(flag_j){
                p3 = (temp_points[4] + temp_points[5]) / 2;
                p4 = (temp_points[6] + temp_points[7]) / 2;
            }
            else{
                p3 = (temp_points[5] + temp_points[6]) / 2;
                p4 = (temp_points[4] + temp_points[7]) / 2;
            }

            //判断条件：装甲板宽度高比1.5到3
            if (abs(p3.x - p1.x) / abs(p2.y - p1.y) > 1 && abs(p3.x - p1.x) / abs(p2.y - p1.y) < 3){
                points.push_back(p1);
                points.push_back(p2);
                points.push_back(p3);
                points.push_back(p4);
            points_comb.push_back(points);
            }
        }
    }

    std::vector<Armor> armors;
    for (int i = 0; i < points_comb.size(); i++){
        std::vector<cv::Point2f> points;
        points = points_comb[i];

        Armor armor(origin_image, points[1], points[0], points[2], points[3]);
        armor.self_correction();

        armors.push_back(armor);
    }

    std::vector<std::string> ignore = {"negative"};
    armor::NumberClassifier number_classifier("../model/mlp.onnx", 
        "../model/label.txt", 
        0.2);

    for (int i = 0; i < points_comb.size(); i++){

        std::vector<Armor> armors_copy;
        armors_copy.push_back(armors[i]);

        number_classifier.ExtractNumbers(origin_image, armors_copy);
        number_classifier.Classify(armors_copy);

        points = points_comb[i];

        if (armors_copy[0].classification_result.find("negative")) {
            std::cout << "classification_result0: " << armors_copy[0].classification_result << std::endl;
            find = 1;

            if(show){
                draw_lines_numbers(points, armors_copy[0].classification_result);

                std::cout << "armors"<< i << " classification_result: " << armors_copy[0].classification_result << std::endl;
                std::cout << "armors"<< i << " classification_confidence: " << armors_copy[0].classification_confidence << std::endl;
            }
        }
        else {
            find = 0;
        }
    }
    if(wait){
        cv::waitKey(0);
    }
    return 0;
}

void Detect::draw_lines_numbers(std::vector<cv::Point2f> points, std::string classification_result){

    cv::Point2f number_position = (points[0] + points[1] + points[2] + points[3]) / 4;
    // 绘制边界点
    for (int i = 0; i < points.size(); i++) {
        cv::circle(image, points[i], 5, cv::Scalar(0,0,255), -1);
    }
    std::cout << "classification_result: " << classification_result << std::endl;

    //连接对角线
    cv::line(image, points[1], points[2], cv::Scalar(255,0,0), 2);
    cv::line(image, points[0], points[3], cv::Scalar(255,0,0), 2);
    std::cout << "classification_result: " << classification_result << std::endl;



    //在image上显示数字
    cv::putText(image, classification_result, number_position, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
}

bool Detect::get_find(){
    return find;
}

std::vector<cv::Point2f> Detect::get_points(){
    return points;
}