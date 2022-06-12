//
// Created by chihiro on 22/05/28.
//

#include "../include/transform.h"

namespace mmc{

    void expandMatrix(Eigen::MatrixXf &mat){
        mat.conservativeResize(4,4);
        mat.col(3).setZero();
        mat.row(3).setZero();
        mat(3, 3) = 1;
    }

    /*
     * 回転行列を取得
     */
    Eigen::MatrixXf getRotate(Eigen::Vector3f &vec){
        Eigen::Matrix3f tmpMat;
        tmpMat = Eigen::AngleAxisf(vec.x(), Eigen::Vector3f::UnitX())
                 * Eigen::AngleAxisf(vec.y(), Eigen::Vector3f::UnitY())
                 * Eigen::AngleAxisf(vec.z(), Eigen::Vector3f::UnitZ());
        Eigen::MatrixXf mat = tmpMat;
        expandMatrix(mat);
        return mat;
    }

    /*
     * 平行移動行列を取得
     */
    Eigen::MatrixXf getTranslate(Eigen::Vector3f &vec){
        Eigen::Matrix4f tmpMat;
        tmpMat <<
               1, 0, 0, vec.x(),
                0, 1, 0, vec.y(),
                0, 0, 1, vec.z(),
                0, 0, 0, 1;
        return tmpMat;
    }

    /*
     * 正規化されたビューを取得
     */
    Eigen::MatrixXf getNormalizeView(const float zMin, const float zMax, const float depth,
                                     const float width, const float height){
        Eigen::MatrixXf tmpMat;
        tmpMat = Eigen::Scaling((depth / (width /2 * zMax)), (depth / (height /2 * zMax)), (1 / zMax));
        expandMatrix(tmpMat);
        return tmpMat;
    }

    /*
     * 透視投影を取得
     */
    Eigen::Matrix4f getPerspective(const float zMin, const float zMax){
        Eigen::Matrix4f tmpMat;
        float zMinTilde = zMin / zMax;
        float z = 1 / (1 - zMinTilde);
        float w = (-1 * zMinTilde) / (1 - zMinTilde);
        tmpMat <<
               1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, z, w,
                0, 0, 1, 0;
        return tmpMat;
    }

    transform::transform(cv::Size window, cv::Size target) : windowSize(window), targetSize(target) {

        float ptxh = (float)target.width / 2;
        float ptyh = (float)target.height / 2;
        pts = std::vector<Eigen::Vector3f>{
                Eigen::Vector3f(-1 * ptxh,      ptyh, 0),
                Eigen::Vector3f(-1 * ptxh, -1 * ptyh, 0),
                Eigen::Vector3f(     ptxh, -1 * ptyh, 0),
                Eigen::Vector3f(     ptxh,      ptyh, 0)
        };

        float tcx = (float)targetSize.width;
        float tcy = (float)targetSize.height;
        srcPt[0] = cv::Point2f(0.0f, 0.0f);
        srcPt[1] = cv::Point2f(0.0f, tcy);
        srcPt[2] = cv::Point2f(tcx, tcy);
        srcPt[3] = cv::Point2f(tcx, 0.0f);

        rotate = Eigen::Vector3f(DEG2RAD(34.0), DEG2RAD(43.0), DEG2RAD(0.0));
        translate = Eigen::Vector3f(0.0f, 0.0f, -300.0f);

        rotateMatrix = getRotate(rotate);
        translateMatrix = getTranslate(translate);
        normalizeMatrix = getNormalizeView(zMin, zMax, depth, windowSize.width, windowSize.height);
        perspectiveMatrix = getPerspective(zMin, zMax);
    }

    void transform::updateRotateVector(float x, float y, float z){
        rotate = Eigen::Vector3f(DEG2RAD(x), DEG2RAD(y), DEG2RAD(z));
        rotateMatrix = getRotate(rotate);
    }

    void transform::calc(cv::OutputArray matrix){
        std::vector<Eigen::Vector3f> transPts = {};
        cv::Mat mat = matrix.getMat();

        for (auto & pt : pts){
            Eigen::Vector4f tmpVec = translateMatrix * rotateMatrix * pt.homogeneous();     // 三次元空間での回転と平行移動
            tmpVec = perspectiveMatrix * normalizeMatrix * tmpVec;      // 投影空間の正規化と透視投影
            transPts.insert(transPts.end(), Eigen::Vector3f(tmpVec.x(), tmpVec.y(), tmpVec.z()));
        }

        float xOffset = float(windowSize.width) / 2;
        float yOffset = float(windowSize.height) / 2;
        cv::Point2f dstPt[4];
        for (int i = 0; i < 4; i++) {
            dstPt[i] = cv::Point2f((transPts[i].x() / transPts[i].z() * xOffset) + xOffset,
                                   (transPts[i].y() / transPts[i].z() * yOffset) + yOffset);
        }

        cv::Mat perMat = cv::getPerspectiveTransform(srcPt, dstPt);
        perMat.copyTo(matrix);

    }

}

