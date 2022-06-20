#include "image.h"

#include "ply.h"

#include <stdexcept>
#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <iomanip>

// #define FITTING_EXAMPLE

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

#define NUM_INPUTS 253

void load_images(const char* path, const char* filename, image_b* images) {

	for (int i = 0; i < NUM_INPUTS; ++i) {
		std::stringstream stream;
		stream << path << "/" << filename << "_" << std::setw(4) << std::setfill('0') << i << ".png";
		std::string file = stream.str();

		images[i] = image_io::load(file.c_str());
	}

}

void load_directions(const char* filename, Eigen::Vector3f* directions) {
	// open the .hdrgen file
	std::ifstream data;
	data.open(filename);
	if (!data.is_open()) {
		std::cout << "Could not open " << filename << "\n";
		throw std::runtime_error("Could not open directions file");
	}

	// open all images named in the file
	for (int i = 0; i < NUM_INPUTS; ++i) {

		std::string line;
		if(!std::getline(data, line)) {
			std::cout << "Not enough lines\n";
			throw std::runtime_error("Not enough lines");
		}

		std::istringstream stream(line);

		std::string num;
		float vec0, vec1, vec2;
		if (!(stream >> num >> vec0 >> vec1 >> vec2)) {
			std::cout << "Invalid vector\n";
			throw std::runtime_error("Invalid vector");
		}

		Eigen::Vector3f vector({vec0, vec1, vec2});

		directions[i] = vector;

	}
	data.close();
}

void calculateSolidAngles(const Eigen::Vector3f* directions, float* solid_angles) {
	
	const Eigen::Vector3f z_axis({ 0.f, 0.f, 1.f });
	for (int i = 0; i < NUM_INPUTS; ++i) {
		const Eigen::Vector3f direction = directions[i];

		const float enumerator = (direction.cross(z_axis)).norm();
		const float denominator = direction.norm(); // norm of z_axis is 1
	}

}

inline unsigned char getR_xy(const image_b* images, const int image, const int x, const int y, const int channel) {
	image_b current_image = images[image];

	return current_image.at2d(x, y, channel);
}

int main(int argc, const char **argv)
{
	const char* path = argv[1];
	const char* filename = argv[2];
	const char* directions_file = argv[3];

	std::vector<image_b> input_images(NUM_INPUTS);
	load_images(path, filename, input_images.data());

	std::vector<Eigen::Vector3f> directions(NUM_INPUTS);
	load_directions(directions_file, directions.data());


	return EXIT_SUCCESS;
}