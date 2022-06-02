#include "image.h"

#include "ply.h"

#include <stdexcept>
#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <string>

#define PI 3.1415926

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

/*
int ply_example()
{
	std::vector<Vertex> vertices = { { 0, 0, 0, 1, 0, 0, 255, 0, 0 }, { 0, 1, 0, 1, 0, 0, 0, 255, 0 }, { 1, 0, 0, 1, 0, 0, 0, 0, 255 } };
	std::vector<Face> faces = { { 0, 1, 2 } };

	std::ofstream os("out.ply", std::ios_base::binary);
	write_ply_mesh(os, vertices, faces);
}
*/

int main(int argc, const char **argv)
{
// 	if (argc != 2) throw std::runtime_error("Invalid arguments");
// 	example(argv[1]);

	const char *path = argv[1];
	const char *out_path = argv[2];

	image<unsigned char> img = image_io::load(path);
	int width = img.width(), height = img.height();
	cout << width << " " << height << endl;

	int radius = width / 2;
	int r_2 = radius * radius;

	int new_width = 1000;//PI * width; // choice
	int new_height = height;
	

	int center_x = new_width / 2, center_z = new_height / 2;

	double rc = new_width / (2 * PI);
	//double n = 2 * rc;

	Eigen::Vector3d R = {0, 1, 0};
	//R = R / R.norm();

	image<unsigned char> output_img(new_width, new_height, 3);

	for (int i = 0; i < new_width; i++)
	{
		for (int j = 0; j < new_height; j++)
		{
			//int x = (i - center_x), z = (j - center_z);
			double angle = 2 * PI * (i / ( (double) new_width + 1));

			double a = - rc * sin(angle);
			double b = - rc * cos(angle);
			double c = - static_cast<double>(j) + new_height / 2;

			//printf("a=%lf, b=%lf, c=%lf", a, b, c);

			Eigen::Vector3d O = {a, b, c};
			O = O / O.norm();

			a = O(0);
			b = O(1);
			c = O(2);

			double new_y = sqrt( (1 - b) / 2);
			double new_x = -a / (2 * new_y);
			double new_z = -c / (2 * new_y);
			
			Eigen::Vector3d N = {new_x, new_y, new_z};
			N = (radius / N.norm()) * N;

			//printf("=======================================\n");

			//printf("i: %d, j: %d\n", i, j);

			// printf("newx: %lf, newy: %lf, newz: %lf, \n", new_x,new_y,new_z);

			//cout << "image coordinate:\n" << N << endl;

			//Eigen::Vector3d O = R - 2 * R.dot(N) * N / (N.norm());

			// t1 = a + x, t2 = b + y
			double t1 = O(0) + new_x, t2 = O(1) + new_y;
			//double scale = sqrt( r_2 / ( t1 * t1 + t2 * t2 ) );
			//scale /= O.norm(); // normalize original direction

			//double point_in_cyll = N + scale * O;

			//cout << N << endl;
			// cout << "a + x: " << t1 << " b + y" << t2 << endl;
			// cout << "v: " << scale << endl;
			// cout << O << endl;
			//cout << R << endl;



			int input_x = N(0) + width / 2;
			int input_y = N(2) + width / 2;

			int output_x = (i + new_width / 2) % new_width;


			//printf("x, y: (%d, %d)\n", input_x, input_y);

			unsigned char c_red = img.at2d(input_x, input_y, 0);
			output_img.at2d(output_x, j, 0) = c_red;

			unsigned char c_green = img.at2d(input_x, input_y, 1);
			output_img.at2d(output_x, j, 1) = c_green;

			unsigned char c_blue = img.at2d(input_x, input_y, 2);
			output_img.at2d(output_x, j, 2) = c_blue;
		}
	}
	
	image_io::save_png(output_img, out_path);

	cout << "ok" << endl;

	return EXIT_SUCCESS;
}