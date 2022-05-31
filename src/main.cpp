#include <iostream>
#include <opencv2/opencv.hpp>

#include "../include/transform.h"

int main() {
    cv::Size2f window(1920, 1080);
    cv::Size2f target(400, 400);

    cv::namedWindow("window", cv::WINDOW_NORMAL);
    //cv::setWindowProperty("window", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    mmc::transform mmc(window, target);
    float x = 0.0f;
    float y = 0.0f;

    while(true){
        mmc.updateRotateVector(x, y, 0.0f);
        cv::Mat* output = mmc.calc();
        cv::imshow("window", *output);
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
