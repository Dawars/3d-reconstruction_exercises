#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


// TODO: Implement the cost function
struct RegistrationCostFunction
{
    RegistrationCostFunction(const Point2D& pointA,const Point2D& pointB,const Weight weight)
            : pointA(pointA), pointB(pointB), w(weight)
    {
    }

    template<typename T>
    bool operator()(const T* const theta, const T* const tx, const T* const ty, T* residual) const
    {
        // TODO: Implement the cost function
//        using Mat =  Eigen::Matrix<T, 2, 2>;
//        using Vec =  Eigen::Matrix<T, 2, 1>;
//        Vec p1(pointA.x, pointA.y);
//        Vec p2(pointB.x, pointB.y);

//        T m00 = ceres::cos(*theta);
//        T m01 = -ceres::sin(*theta);
//        T m10 = ceres::sin(*theta);
//        T m11 = ceres::cos(*theta);

        auto diff1 = ceres::cos(*theta)*pointA.x - ceres::sin(*theta)*pointA.y + *tx  -T(pointB.x);
        auto diff2 = ceres::sin(*theta)*pointA.x + ceres::cos(*theta)*pointA.y + *ty  -T(pointB.y);

        residual[0] = T(w.w) *(diff1 * diff1 + diff2 * diff2);

        return true;
    }

private:
    const Point2D pointA;
    const Point2D pointB;
    const Weight w;
};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// TODO: Read data points and the weights. Define the parameters of the problem
	const std::string file_path_1 = "../data/points_dragon_1.txt";
	const std::string file_path_2 = "../data/points_dragon_2.txt";
	const std::string file_path_weights = "../data/weights_dragon.txt";

    const auto pointsA = read_points_from_file<Point2D>(file_path_1);
    const auto pointsB = read_points_from_file<Point2D>(file_path_2);
    const auto weights = read_points_from_file<Weight>(file_path_weights);

	ceres::Problem problem;
// Good initial values make the optimization easier
    const double theta_initial = 0.0;
    const double tx_initial = 0.0;
    const double ty_initial = 0.0;

    double theta = theta_initial;
    double tx = tx_initial;
    double ty = ty_initial;

	// TODO: For each weighted correspondence create one residual block
    for (int i = 0; i < weights.size(); ++i) {
        auto& w = weights[i];
        auto& pointA = pointsA[i];
        auto& pointB = pointsB[i];

        problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<RegistrationCostFunction, 1, 1, 1, 1>(
                        new RegistrationCostFunction(pointA, pointB, w)),
                nullptr, &theta, &tx, &ty
        );
    }

	ceres::Solver::Options options;
	options.max_num_iterations = 25;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;

	// TODO: Output the final values of the translation and rotation (in degree)
    std::cout << "Initial theta: " << theta_initial << "\ttx: " << tx_initial << "\tty: " << ty_initial << std::endl;
    std::cout << "Final --deg " << 180 * theta/M_PI << " --tx " << tx << " --ty " << ty << std::endl;

    system("pause");
	return 0;
}