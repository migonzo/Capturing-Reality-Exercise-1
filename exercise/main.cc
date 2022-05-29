#include "image.h"

#include "ply.h"

#include <stdexcept>
#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <string>

// #define FITTING_EXAMPLE
using namespace std;

#ifdef FITTING_EXAMPLE
#include "ceres/ceres.h"
#endif

void example(const char *filename)
{
	std::cout << "Running example code..." << std::endl;

	image_b img_b = image_io::load(filename);

	float r = 50.f;
	if (img_b.width() < r + r + 10 || img_b.height() < r + r + 10) throw std::runtime_error("Image too small");

	for (int y = -r; y < r; ++y) {
		for (int x = -r; x < r; ++x) {
			Eigen::Vector2d p(x, y);
			Eigen::Vector2d pa = p + Eigen::Vector2d(r + 10, r + 10);
			if (p.norm() <= r) {
				img_b.at_v(pa, 0) = 255;
				img_b.at_v(pa, 1) = 0;
				img_b.at_v(pa, 2) = 0;
			}
		}
	}

	image_f img_f = image_manip::grayscale(img_b);

	image_io::save(img_b, "out.jpg");
	image_io::save(img_b, "out.png");
	image_io::save(img_f, "out.pfm");
}

#ifdef FITTING_EXAMPLE
// adopted from: https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/helloworld.cc
struct ExampleCostFunctor {
	template <typename T>
	bool operator()(const T *const x, T *residual) const
	{
		residual[0] = 10.0 - x[0];
		return true;
	}

	static ceres::CostFunction *Create()
	{
		return new ceres::AutoDiffCostFunction<ExampleCostFunctor, 1, 1>(new ExampleCostFunctor);
	}
};

void fitting_example()
{
	std::cout << "Running fitting example code..." << std::endl;

	double x = 0.5;
	const double initial_x = x;

	// Build the problem.
	ceres::Problem problem;

	// Set up the only cost function (also known as residual). This uses
	// auto-differentiation to obtain the derivative (jacobian).
	ceres::CostFunction *cost_function = ExampleCostFunctor::Create();
	problem.AddResidualBlock(cost_function, NULL, &x);

	// Run the solver!
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;
	std::cout << "x: " << initial_x << " -> " << x << std::endl;
}
#endif

int ply_example()
{
	std::vector<Vertex> vertices = { { 0, 0, 0, 1, 0, 0, 255, 0, 0 }, { 0, 1, 0, 1, 0, 0, 0, 255, 0 }, { 1, 0, 0, 1, 0, 0, 0, 0, 255 } };
	std::vector<Face> faces = { { 0, 1, 2 } };

	std::ofstream os("out.ply", std::ios_base::binary);
	write_ply_mesh(os, vertices, faces);
}

int main(int argc, const char **argv)
{
// 	if (argc != 2) throw std::runtime_error("Invalid arguments");
// 	example(argv[1]);

	const char *path = "/home/arthur/Pictures/spherical_panorama2.jpg";

	image<unsigned char> img = image_io::load(path);
	int width = img.width(), height = img.height();
	cout << width << " " << height << endl;

	int center_x = width / 2, center_z = height / 2;
	int radius = center_x;
	int r_2 = radius * radius;

	Eigen::Vector3d R = {0, -1, 0};

	for (int i = center_x - 2; i < center_x + 2; i++)
	{
		for (int j = center_z - 2; j < center_z + 2; j++)
		{
			int x = (i - center_x), z = (j - center_z);
			double new_x, new_y, new_z;

			printf("=======================================\n");

			if(x == 0 && z == 0) 
			{
				new_x = new_z = 0;
				new_y = radius;
			}
			else 
			{
				int sqr_dist = x * x + z * z;
				int diff = r_2 - sqr_dist;

				if(diff < 0) continue;

				new_x = x;
				new_z = z;
				new_y = sqrt(diff);

				printf("x: %d, z: %d\n", x, z);
				printf("radius^2: %d, sqrdist: %d\n", r_2, sqr_dist);
			}

			printf("newx: %lf, newy: %lf, newz: %lf, \n", new_x,new_y,new_z);
			Eigen::Vector3d N = {new_x, new_y, new_z};
			Eigen::Vector3d O = R - 2 * R.dot(N) * N / (N.norm());

			// t1 = a + x, t2 = b + y
			double t1 = O(0) + new_x, t2 = O(1) + new_y;
			double scale = sqrt( r_2 / ( t1 * t1 + t2 * t2 ) );
			//scale /= O.norm(); // normalize original direction

			double point_in_cyll = N + scale * O;

			//cout << N << endl;
			cout << t1 << " " << t2 << endl;
			cout << scale << endl;
			cout << O << endl;
			//cout << R << endl;
		}
	}
	

	cout << "ok" << endl;

	return EXIT_SUCCESS;
}