//
// Created by chihiro on 22/05/28.
//

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Geometry>

#ifndef MOVING_MIRROR_CONTROLLER_TRANSFORM_H
#define MOVING_MIRROR_CONTROLLER_TRANSFORM_H

#define DEG2RAD(a) ((a)/180.0 * M_PI)

namespace mmc {

    void expandMatrix(Eigen::MatrixXf &mat);
    Eigen::MatrixXf getRotate(Eigen::Vector3f &vec);
    Eigen::MatrixXf getTranslate(Eigen::Vector3f &vec);
    Eigen::MatrixXf getNormalizeView(float zMin, float zMax, float depth, float width, float height);
    Eigen::Matrix4f getPerspective(float zMin, float zMax);

    class transform {
    public:

        transform(cv::Size window, cv::Size target);
        cv::Mat* calc();
        void updateRotateVector(float x, float y, float z);

    private:
        cv::Size2f windowSize;
        cv::Size2f targetSize;
        cv::Mat output;
        cv::Mat targetImg;
        float zMax = 1000.0f;
        float zMin = 100.0f;
        float depth = 500.0f;
        std::vector<Eigen::Vector3f> pts;
        cv::Point2f *srcPt;
        Eigen::Vector3f rotate;
        Eigen::Vector3f translate;
        Eigen::Matrix4f rotateMatrix;
        Eigen::Matrix4f translateMatrix;
        Eigen::Matrix4f normalizeMatrix;
        Eigen::Matrix4f perspectiveMatrix;

    };

}


#endif //MOVING_MIRROR_CONTROLLER_TRANSFORM_H
