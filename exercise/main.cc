#include "image.h"

#include "ply.h"

#include <stdexcept>
#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>

// #define FITTING_EXAMPLE

#ifdef FITTING_EXAMPLE
#include "ceres/ceres.h"
#endif

void example(const char *filename)
{
	std::cout << "Running example code..." << std::endl;

	image_b img_b = image_io::load(filename);
	std::cout << img_b.channels();
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

void ply_example()
{
	std::vector<Vertex> vertices = { { 0, 0, 0, 1, 0, 0, 255, 0, 0 }, { 0, 1, 0, 1, 0, 0, 0, 255, 0 }, { 1, 0, 0, 1, 0, 0, 0, 0, 255 } };
	std::vector<Face> faces = { { 0, 1, 2 } };

	std::ofstream os("out.ply", std::ios_base::binary);
	write_ply_mesh(os, vertices, faces);
}

image_b get_num_pixels(const char* path, const char* filename_base, const char* extension, unsigned int& image_height, unsigned int& image_width, unsigned int& image_channels, unsigned int& num_pixels) {
	std::string path_str(path);
	std::string base_str(filename_base);
	std::string ext_str(extension);
	std::string file_str = path_str + "\\" + base_str + "0" + ext_str;
	image_b image = image_io::load(file_str.data());
	

	return image;
}

std::vector<image_b> load_images(const char* path, const char* filename_base, std::vector<float>& times, unsigned int& num_images) {

	std::string path_str(path);
	std::string base_str(filename_base);
	std::string hdrgen = path_str + "\\" + base_str;
	std::ifstream data;
	data.open(hdrgen);

	std::vector<image_b> images;
	while (!data.eof()) {
		std::string filename;
		data >> filename;
		//std::cout << filename;
		const std::string file_str = path_str + "\\" + filename;
		std::cout << file_str << "\n";
		images.push_back(image_io::load(file_str.data()));
		float time;
		data >> time;
		times.push_back(time);
		++num_images;
	}
	data.close();
	return images;
}

void calculate_weights(const image_b image, const unsigned int num_pixels, float* weights) {
	for (int i = 0; i < num_pixels; ++i) {
		const float difference = static_cast<float>(image.at(i, 0)) - 127.5;
		const float exponent = -4 * ((difference * difference) / (127.5 * 127.5));
		weights[i] = std::exp(exponent);
	}
}

void calculate_weights_all_images(const std::vector<image_b> images, const unsigned int num_images, const unsigned int num_pixels, float* weights) {
	for (unsigned int i = 0; i < num_images; ++i) {
		calculate_weights(images[i], num_pixels, weights + i * num_pixels);
	}
}

void calculate_initial_i_m(float* i_m) {
	for (unsigned int i = 0; i < 255; ++i) {
		i_m[i] = i * (1.0 / 128.0);
	}
}

inline void calculate_i_m(const unsigned int cardinality, const std::tuple<unsigned int, unsigned int>* e_m, const float* times, const float* irradiances, float& i_m) {

	float sum = 0;

	for (unsigned int h = 0; h < cardinality; ++h) {

		unsigned int i = std::get<0>(e_m[h]);
		unsigned int j = std::get<1>(e_m[h]);
		sum += times[i] * irradiances[j];

	}

	i_m = sum / static_cast<float>(cardinality);
}

void calculate_i_m_values(const unsigned int* cardinalities, std::tuple<unsigned int, unsigned int>* e_m, const float* times, const float* irradiances, float* i_m) {

	std::tuple<unsigned int, unsigned int>* current_e_m = e_m;
	const size_t tuple_size = sizeof(std::tuple<unsigned int, unsigned int>);
	for (int i = 0; i < 255; ++i) {
		
		calculate_i_m(cardinalities[i], current_e_m, times, irradiances, i_m[i]);
		
		current_e_m += cardinalities[i] * tuple_size;

	}

}

void calculate_irradiances(const float* weights, const float* times, const float* i_m, std::vector<image_b> images, const unsigned int num_images, const unsigned int num_pixels, float* irradiance) {

	for (unsigned int h = 0; h < num_pixels; ++h) {
		float sum_1 = 0;
		float sum_2 = 0;
		for (unsigned int i = 0; i < num_images; ++i) {
			const unsigned int index = h + i * num_pixels;
			const unsigned int pixel_value = images[i].at(h);
			sum_1 += weights[index] * times[i] * i_m[pixel_value];

			sum_2 += weights[index] * times[i] * times[i];
		}

		irradiance[h] = sum_1 / sum_2;
	}
}

// calculates the objective function for unknown response function
float calculate_objective_function(const float* weights, const float* times, const float* i_values, const float* irradiances, const unsigned int num_images, const unsigned int num_pixels) {

	float result = 0;

	for (unsigned int i = 0; i < num_images; ++i) {

		for (unsigned int j = 0; j < num_pixels; ++j) {

			unsigned int index = i * num_pixels + j;
			float intermediate = i_values[index] - times[i] * irradiances[j];
			result += weights[index] * intermediate * intermediate;

		}

	}

	return result;

}

int main(int argc, const char **argv)
{
	if (argc != 3) throw std::runtime_error("Invalid arguments");
	const char* path = argv[1];
	const char* filename_base = argv[2];

	std::vector<float> times;
	unsigned int num_images = 0;

	std::vector<image_b> images = load_images(path, filename_base, times, num_images);

	const unsigned int image_height = images[0].height();
	const unsigned int image_width = images[0].width();
	const unsigned int image_channels = images[0].channels();
	const unsigned int num_pixels = image_height * image_width;

	std::cout << "height " << image_height << " width " << image_width << " channels " << image_channels << " pixels per image: " << num_pixels << "\n";

	std::vector<float> weights(num_pixels * num_images);

	calculate_weights_all_images(images, num_images, num_pixels, weights.data());

	float i_m[255];

	calculate_initial_i_m(i_m);

	std::vector<float> irradiances(num_pixels);

	calculate_irradiances(weights.data(), times.data(), i_m, images, num_images, num_pixels, irradiances.data());

	//example(argv[1]);

#ifdef FITTING_EXAMPLE
	fitting_example();
#endif

	//ply_example();

	return EXIT_SUCCESS;

	/*
	image_b image = image_io::load("C:\\Users\\migon\\Documents\\Capturing Reality\\Exercise 1\\HDR Sequence\\max0.ppm");
	for (unsigned int i = 0; i < image.width(); ++i) {
		for (unsigned int j = 0; j < image.height(); ++j) {
			unsigned char* ptr = image.ptr2d(i, j, 0);
			*ptr = 0;
		}
	}
	for (unsigned int i = 0; i < image.width(); ++i) {
		for (unsigned int j = 0; j < image.height(); ++j) {
			unsigned char* ptr = image.ptr2d(i, j, 1);
			*ptr = 0;
		}
	}
	image_io::save_png(image, "C:\\Users\\migon\\Documents\\Capturing Reality\\test.png");
	*/
}