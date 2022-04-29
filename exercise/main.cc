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
		if (filename.length() < 1 || data.eof())
			break;
		const std::string file_str = path_str + "\\" + filename;
		images.push_back(image_io::load(file_str.data()));
		float denominator;
		data >> denominator;
		float time = 1.0 / static_cast<float>(denominator);
		times.push_back(time);
		++num_images;
	}
	data.close();
	return images;
}

void fill_e_m(const std::vector<image_b> images, const unsigned int channel, const unsigned int num_images, const unsigned int num_pixels, unsigned int* cardinalities, std::vector<std::vector<std::tuple<unsigned int, unsigned int>>>& e_m) {

	for (unsigned int i = 0; i < num_images; ++i) {
		for (unsigned int j = 0; j < num_pixels; ++j) {
			const unsigned int pixel_value = images[i].at(j, channel);
			e_m[pixel_value].push_back({ i, j });
		}
	}

	for (unsigned int i = 0; i < 256; ++i) {
		cardinalities[i] = e_m[i].size();
	}

}

void fill_e_m_rgb(const std::vector<image_b> images, const unsigned int channels, const unsigned int num_images, const unsigned int num_pixels, unsigned int* cardinalities, std::vector<std::vector<std::tuple<unsigned int, unsigned int>>>& e_m) {
	for (unsigned int i = 0; i < channels; ++i) {
		fill_e_m(images, i, num_images, num_pixels, cardinalities + i * 256, )
	}
}

void calculate_weights(const image_b image, const unsigned int channel, const unsigned int num_pixels, float* weights) {
	for (int i = 0; i < num_pixels; ++i) {
		const unsigned int pixel_value = image.at(i, channel);
		if (pixel_value < 5 || pixel_value > 250) {
			weights[i] = 0;
			continue;
		}
		const float difference = static_cast<float>(pixel_value) - 127.5;
		const float exponent = -4 * ((difference * difference) / (127.5 * 127.5));
		weights[i] = std::exp(exponent);
	}
}

void calculate_weights_all_images(const std::vector<image_b> images, const unsigned int channel, const unsigned int num_images, const unsigned int num_pixels, float* weights) {
	for (unsigned int i = 0; i < num_images; ++i) {
		calculate_weights(images[i], channel, num_pixels, weights + i * num_pixels);
	}
}

void calculate_weights_all_images_rgb(const std::vector<image_b> images, const unsigned int channels, const unsigned int num_images, const unsigned int num_pixels, float* weights) {
	for (unsigned int i = 0; i < 3; ++i) {
		calculate_weights_all_images(images, i, num_images, num_pixels, weights + i * num_pixels);
	}
}

void calculate_initial_i_m(float* i_m) {
	for (unsigned int i = 0; i < 256; ++i) {
		i_m[i] = i * (1.0 / 128.0);
	}
}

void calculate_initial_i_m_rgb(float* i_m, const unsigned int channels) {
	for (unsigned int i = 0; i < channels; ++i) {
		calculate_initial_i_m(i_m + i * 256);
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

void calculate_i_m_values(const unsigned int* cardinalities, std::vector<std::vector<std::tuple<unsigned int, unsigned int>>> e_m, const float* times, const float* irradiances, float* i_m) {

	const size_t tuple_size = sizeof(std::tuple<unsigned int, unsigned int>);
	for (int i = 0; i < 256; ++i) {
		
		calculate_i_m(cardinalities[i], e_m[i].data(), times, irradiances, i_m[i]);

	}

}

void calculate_irradiances(const float* weights, const float* times, const float* i_m, std::vector<image_b> images, const unsigned int channel, const unsigned int num_images, const unsigned int num_pixels, float* irradiance) {

	for (unsigned int h = 0; h < num_pixels; ++h) {
		float sum_1 = 0;
		float sum_2 = 0;
		for (unsigned int i = 0; i < num_images; ++i) {
			const unsigned int index = h + i * num_pixels;
			const unsigned int pixel_value = images[i].at(h, channel);
			if (h == 200)
				std::cout << "pixel_value: " << pixel_value << " weight: " << weights[index] << " time: " << times[i] << " I_m: " << i_m[pixel_value] << "\n";
			sum_1 += weights[index] * times[i] * i_m[pixel_value];

			sum_2 += weights[index] * times[i] * times[i];
		}
		if (h == 200)
			std::cout << "sum1: " << sum_1 << " sum2: " << sum_2 << "\n";
		irradiance[h] = sum_1 / sum_2;
		if (h == 200)
			std::cout << "irradiance: " << irradiance[h] << "\n";
	}
}

void calculate_irradiances_rgb(const float* weights, const float* times, const float* i_m, std::vector<image_b> images, const unsigned int channels, const unsigned int num_images, const unsigned int num_pixels, float* irradiance) {
	for (unsigned int i = 0; i < channels; ++i) {
		calculate_irradiances(weights, times, i_m, images, i, num_images, num_pixels, irradiance + i * num_pixels);
	}
}

void scale_i_m(float* i_m) {
	float normal = i_m[128];
	for (unsigned int i = 0; i < 256; ++i) {
		i_m[i] /= normal;
	}
}

void assign_pixel_value_to_irradiances(const float* irradiances, const unsigned int num_pixels, const float min, const float max, const unsigned int channel, image_b& image) {
	for (unsigned int i = 0; i < num_pixels; ++i) {
		image.at(i, channel) = ((irradiances[i] - min) / (max - min)) * 255;
	}
}

void assign_pixel_value_to_irradiances_with_exposure(const float* irradiances, const unsigned int num_pixels, const float * i_m, const unsigned int channel, const float time, image_b& image) {
	for (unsigned int i = 0; i < num_pixels; ++i) {
		const float i_value = irradiances[i] * time;
		unsigned int pixel_value = 0;
		for (; pixel_value < 256; ++pixel_value) {
			if (i_value <= i_m[pixel_value])
				break;
		}
		image.at(i, channel) = pixel_value;
	}
}

// calculates the objective function for unknown response function
float calculate_objective_function(const float* weights, const float* times, const float* i_m, std::vector<image_b> images, const unsigned int channel, const float* irradiances, const unsigned int num_images, const unsigned int num_pixels) {

	float result = 0;

	for (unsigned int i = 0; i < num_images; ++i) {

		for (unsigned int j = 0; j < num_pixels; ++j) {

			unsigned int index = i * num_pixels + j;
			const unsigned int pixel_value = images[i].at(j, channel);
			float intermediate = i_m[pixel_value] - times[i] * irradiances[j];
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

	std::vector<float> weights(num_pixels * num_images * image_channels);


	calculate_weights_all_images_rgb(images, image_channels, num_images, num_pixels, weights.data());

	float i_m[256 * image_channels];

	calculate_initial_i_m_rgb(i_m, image_channels);

	std::vector<float> irradiances(num_pixels * image_channels);

	calculate_irradiances_rgb(weights.data(), times.data(), i_m, images, image_channels, num_images, num_pixels, irradiances.data());

	std::vector<unsigned int> cardinalities_red(256 * image_channels);

	std::vector<std::vector <std::vector<std::tuple<unsigned int, unsigned int>>>> e_m(image_channels);
	for(unsigned int i = 0; i < image_channels; ++i) {
		e_m[i] = std::vector<std::vector<std::tuple<unsigned int, unsigned int>>>(256);
	}

	fill_e_m_rgb(images, image_channels, num_images, num_pixels, cardinalities.data(), e_m);
	fill_e_m(images, 1, num_images, num_pixels, cardinalities_green, e_m_green);
	fill_e_m(images, 2, num_images, num_pixels, cardinalities_blue, e_m_blue);

	std::cout << "##################\n";

	for (unsigned int i = 0; i < 10; ++i) {

		calculate_i_m_values(cardinalities_red, e_m_red, times.data(), irradiances_red.data(), i_m_red);
		calculate_i_m_values(cardinalities_green, e_m_green, times.data(), irradiances_green.data(), i_m_green);
		calculate_i_m_values(cardinalities_blue, e_m_blue, times.data(), irradiances_blue.data(), i_m_blue);

		scale_i_m(i_m_red);
		scale_i_m(i_m_green);
		scale_i_m(i_m_blue);

		calculate_irradiances(weights_red.data(), times.data(), i_m_red, images, 0, num_images, num_pixels, irradiances_red.data());
		calculate_irradiances(weights_green.data(), times.data(), i_m_green, images, 1, num_images, num_pixels, irradiances_green.data());
		calculate_irradiances(weights_blue.data(), times.data(), i_m_blue, images, 2, num_images, num_pixels, irradiances_blue.data());

		//calculate_objective_function(weights.data(), times.data(), i_m, images, irradiances.data(), num_images, num_pixels);

	}

	float max_red = -10000000000.0;
	float min_red = 10000000000.0;
	float max_green = -10000000000.0;
	float min_green = 10000000000.0;
	float max_blue = -10000000000.0;
	float min_blue = 10000000000.0;
	for (unsigned int j = 0; j < num_pixels; ++j) {
		if (max_red < irradiances_red[j])
			max_red = irradiances_red[j];
		if (min_red > irradiances_red[j])
			min_red = irradiances_red[j];
		if (max_green < irradiances_green[j])
			max_green = irradiances_green[j];
		if (min_green > irradiances_green[j])
			min_green = irradiances_green[j];
		if (max_blue < irradiances_blue[j])
			max_blue = irradiances_blue[j];
		if (min_blue > irradiances_blue[j])
			min_blue = irradiances_blue[j];
	}
	image_b output(image_width, image_height, 3);

	assign_pixel_value_to_irradiances(irradiances_red.data(), num_pixels, min_red, max_red, 0, output);
	assign_pixel_value_to_irradiances(irradiances_green.data(), num_pixels, min_green, max_green, 1, output);
	assign_pixel_value_to_irradiances(irradiances_blue.data(), num_pixels, min_blue, max_blue, 2, output);

	std::string out_str = "C:\\Users\\migon\\Documents\\Capturing Reality\\output_linear.png";
	image_io::save_png(output, out_str.data());

	for (unsigned int i = 0; i < num_images; ++i) {
		image_b output(image_width, image_height, 3);

		assign_pixel_value_to_irradiances_with_exposure(irradiances_red.data(), num_pixels, i_m_red, 0, times[i], output);
		assign_pixel_value_to_irradiances_with_exposure(irradiances_green.data(), num_pixels, i_m_green, 1, times[i], output);
		assign_pixel_value_to_irradiances_with_exposure(irradiances_blue.data(), num_pixels, i_m_blue, 2, times[i], output);

		std::string out_str = "C:\\Users\\migon\\Documents\\Capturing Reality\\output_" + std::to_string(i) + ".png";
		image_io::save_png(output, out_str.data());
	}

	/*image_b output(image_width, image_height, 3);

	assign_pixel_value_to_irradiances(irradiances_red.data(), num_pixels, min_red, max_red, 0, output);
	assign_pixel_value_to_irradiances(irradiances_green.data(), num_pixels, min_green, max_green, 1, output);
	assign_pixel_value_to_irradiances(irradiances_blue.data(), num_pixels, min_blue, max_blue, 2, output);

	std::string out_str = "C:\\Users\\migon\\Documents\\Capturing Reality\\output.png";
	image_io::save_png(output, out_str.data());*/

	std::cout << "red:\n";
	for (unsigned int j = 1; j < 256; ++j) {
		//if (i_m_red[j] < i_m_red[j - 1])
			std::cout << "i_" << j << ": " << i_m_red[j] << "\n";
	}
	std::cout << "green:\n";
	for (unsigned int j = 1; j < 256; ++j) {
		//if (i_m_green[j] < i_m_green[j - 1])
			std::cout << "i_" << j << ": " << i_m_green[j] << "\n";
	}
	std::cout << "blue:\n";
	for (unsigned int j = 1; j < 256; ++j) {
		//if (i_m_blue[j] < i_m_blue[j - 1])
			std::cout << "i_" << j << ": " << i_m_blue[j] << "\n";
	}

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