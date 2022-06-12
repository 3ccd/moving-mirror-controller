#include <iostream>
#include <opencv2/opencv.hpp>

#include "../include/transform.h"

int main() {
    cv::Size2f window(1920, 1080);

    cv::namedWindow("window", cv::WINDOW_NORMAL);
    cv::setWindowProperty("window", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    cv::Mat base(window, CV_8UC3);
    cv::Mat img = cv::imread("/home/chihiro/CLionProjects/moving-mirror-controller/resource/Parrots.bmp", cv::IMREAD_COLOR);
    cv::Size2f target((float)img.cols, (float)img.rows);
    cv::Mat perMat;

    mmc::transform mmc(window, target);
    float x = 0.0f;
    float y = 0.0f;

    while(true){
        mmc.updateRotateVector(x, y, 0.0f);
        mmc.calc(perMat);
        cv::warpPerspective(img, base, perMat, window);
        cv::line(base, cv::Point(960, 540), cv::Point(0, 540), cv::Scalar_<double>(255,0,0));
        cv::imshow("window", base);
        int input = cv::waitKey(0);
        //std::cout << input << std::endl;
        if(input == 113) break;
        switch(input){
            case 119:
                x = x + 1.0f;
                break;
            case 97:
                y = y - 1.0f;
                break;
            case 115:
                x = x - 1.0f;
                break;
            case 100:
                y = y + 1.0f;
                break;
            default:
                break;
        }
    }

    return 0;
}
