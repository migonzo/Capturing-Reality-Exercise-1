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

	int center_x = width / 2, center_y = height / 2;
	int radius = center_x;

	for (int i = center_x - 5; i < center_x + 5; i++)
	{
		for (int j = center_y - 5; j < center_y + 5; j++)
		{
			int x = (i - center_x), y = (j - center_y);
			double new_x, new_y, new_z;

			if(x == 0 && y == 0) 
			{
				new_x = new_y = new_z = 0;
			}
			else 
			{
				int sqr_dist = x * x + y * y;
				int diff = radius * radius - sqr_dist;

				if(diff < 0) continue;

				new_x = (x * radius) / sqrt(sqr_dist);
				new_y = (y * radius) / sqrt(sqr_dist);
				new_z = sqrt(radius * radius - sqr_dist);

				printf("radius^2: %d, sqrdist: %d\n", radius * radius, sqr_dist);
			}

			printf("newx: %lf, newy: %lf, newz: %lf, \n", new_x,new_y,new_z);
		}
	}
	

	cout << "ok" << endl;

	return EXIT_SUCCESS;
}