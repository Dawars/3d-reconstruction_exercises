#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);

		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements
		Matrix4f estimatedPose = Matrix4f::Identity();

		estimatedPose.block(0,0, 3, 3) = rotation;
		estimatedPose.block(0,3, 3, 1) = translation;


		std::cout << estimatedPose << std::endl;

		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// Compute the mean of input points.
        auto N = points.size();

        Vector3f mean(0,0,0);
        for(const auto& point : points) {
            mean += point;
        }

        mean = mean / N;

        return mean;
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm. 
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.

		// vector<Vec3d>  [N, 3]

		int N = sourcePoints.size();

//		[3 N] [N 3]
//		svd(X^T X^)

        Matrix3f result;
        result << MatrixXf::Zero(3,3);

//        MatrixXf X(N, 3); // target
//        MatrixXf X_hat(N, 3); // source
//        for (int i = 0; i < N; ++i) { // row of X^T (col of X)      [0, 3]
//            for (int j = 0; j < 3; ++j) { //
//                X(i, j) = targetPoints[i][j] - targetMean[j];
//                X_hat(i, j) = sourcePoints[i][j] - sourceMean[j];
//            }
//        }
//        result = X.transpose() * X_hat;

        for (int i = 0; i < 3; ++i) { // row of X^T (col of X)      [0, 3]
            for (int j = 0; j < 3; ++j) { // col of X^               [0, 3]
                for (int k = 0; k < N; ++k) { //                    [0, N]
                    result(i, j) += (targetPoints[k][i] - targetMean[i]) * (sourcePoints[k][j] - sourceMean[j]); // ith row, jth col
                }
            }
        }


//        std::cout << result << std::endl;
        JacobiSVD<MatrixXf> svd(result, ComputeThinU | ComputeThinV);

//        std::cout << svd.matrixU() << std::endl;
//        std::cout << svd.matrixV() << std::endl;
//        std::cout << svd.singularValues() << std::endl;

        //  [3 3] = [3 r] [r 3]
        return svd.matrixU() * svd.matrixV().transpose();
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target opints.

		//      [3 3]       [3, 1]
		return -rotation * sourceMean + targetMean;
	}
};