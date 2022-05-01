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

// function to simultaneously load all images and the corresponding exposure times
std::vector<image_b> load_images(const char* path, const char* hdrgen, std::vector<float>& times, unsigned int& num_images) {

	// open the .hdrgen file
	std::string file_str = std::string(path) + "\\" + std::string (hdrgen);
	std::ifstream data;
	data.open(file_str);
	if (!data.is_open()) {
		std::cout << "Could not open " << file_str << "\n";
		throw std::runtime_error("Could not open hdrgen file");
	}

	// open all images named in the file
	std::vector<image_b> images;
	while (!data.eof()) {

		// read image filename from file
		std::string filename;
		data >> filename;
		if (filename.length() < 1 || data.eof())
			break;
		const std::string file_str = std::string(path) + "\\" + filename;

		// load image
		try {
			images.push_back(image_io::load(file_str.data()));
		}
		catch (std::runtime_error) {
			std::cout << "Could not load " << file_str << "\n";
			throw std::runtime_error("Failed to load image\n");
		}

		// calculate exposure time from the given value
		float denominator;
		data >> denominator;
		float time = 1.0 / static_cast<float>(denominator);
		times.push_back(time);

		// count the total number of loaded images
		++num_images;
	}
	data.close();
	return images;
}

// function to fill the e_m sets (as std::vector)
void fill_e_m(const std::vector<image_b> images, const unsigned int channel, const unsigned int num_images, const unsigned int num_pixels, unsigned int* cardinalities, std::vector<std::vector<std::tuple<unsigned int, unsigned int>>>& e_m) {

	// push each pair of (image number, pixel index) to the corresponding e_m
	for (unsigned int i = 0; i < num_images; ++i) {
		for (unsigned int j = 0; j < num_pixels; ++j) {
			const unsigned int pixel_value = images[i].at(j, channel);
			e_m[pixel_value].push_back({ i, j });
		}
	}

	// save the sizes of each e_m
	for (unsigned int i = 0; i < 256; ++i) {
		cardinalities[i] = e_m[i].size();
	}

}
// function to calculate the weights for one image
void calculate_weights(const image_b image, const unsigned int channel, const unsigned int num_pixels, float* weights) {
	// calculate the weight for each pixel in the image
	for (int i = 0; i < num_pixels; ++i) {
		const unsigned int pixel_value = image.at(i, channel);
		// clamp the weights
		if (pixel_value < 5 || pixel_value > 250) {
			weights[i] = 0;
			continue;
		}
		const float difference = static_cast<float>(pixel_value) - 127.5;
		const float exponent = -4 * ((difference * difference) / (127.5 * 127.5));
		weights[i] = std::exp(exponent);
	}
}

// function to calculate the weights for all images
void calculate_weights_all_images(const std::vector<image_b> images, const unsigned int channel, const unsigned int num_images, const unsigned int num_pixels, float* weights) {
	for (unsigned int i = 0; i < num_images; ++i) {
		calculate_weights(images[i], channel, num_pixels, weights + i * num_pixels);
	}
}

// function to calculate initial I_m values as a linear function
void calculate_initial_i_m(float* i_m) {
	for (unsigned int i = 0; i < 255; ++i) {
		// each value is calculated such that I_128 is 1.0
		i_m[i] = i * (1.0 / 128.0);
	}
}

// function to calculate I_m value as specified in the paper
inline void calculate_i_m(const unsigned int cardinality, const std::tuple<unsigned int, unsigned int>* e_m, const float* times, const float* irradiances, float& i_m) {

	float sum = 0;

	for (unsigned int h = 0; h < cardinality; ++h) {
		unsigned int i = std::get<0>(e_m[h]);
		unsigned int j = std::get<1>(e_m[h]);
		sum += times[i] * irradiances[j];
	}

	i_m = sum / static_cast<float>(cardinality);

}

// function to calculate all 255 I_m values
void calculate_i_m_values(const unsigned int* cardinalities, std::vector<std::vector<std::tuple<unsigned int, unsigned int>>> e_m, const float* times, const float* irradiances, float* i_m) {

	for (int i = 0; i < 255; ++i) {
		calculate_i_m(cardinalities[i], e_m[i].data(), times, irradiances, i_m[i]);
	}

}

// function to calculate the irradiances
void calculate_irradiances(const float* weights, const float* times, const float* i_m, std::vector<image_b> images, const unsigned int channel, const unsigned int num_images, const unsigned int num_pixels, float* irradiance) {

	// go through all pixels
	for (unsigned int h = 0; h < num_pixels; ++h) {
		float sum_1 = 0;
		float sum_2 = 0;

		// for each pixel, go through all images
		for (unsigned int i = 0; i < num_images; ++i) {
			const unsigned int index = h + i * num_pixels;
			const unsigned int pixel_value = images[i].at(h, channel);

			if (pixel_value == 255) // weights are definetly 0
				continue;

			sum_1 += weights[index] * times[i] * i_m[pixel_value];

			sum_2 += weights[index] * times[i] * times[i];
		}
		// check to avoid division by 0
		// sum_1 should also be 0 in this case
		if (sum_2 == 0) {
			irradiance[h] = 0;
		}
		// otherwise calculate fraction as usual
		else {
			irradiance[h] = sum_1 / sum_2;
		}

	}
}

// function to scale all I_m values such that I_128 is 1.0
void scale_i_m(float* i_m) {
	float normal = i_m[128];
	for (unsigned int i = 0; i < 255; ++i) {
		i_m[i] /= normal;
	}
}

// function to assign pixel values to irradiances according to a linear function
void assign_pixel_value_to_irradiances(const float* irradiances, const unsigned int num_pixels, const float min, const float max, const unsigned int channel, image_b& image) {
	for (unsigned int i = 0; i < num_pixels; ++i) {
		// each value is calculated such that min is 0 and max is 255
		image.at(i, channel) = ((irradiances[i] - min) / (max - min)) * 255;
	}
}

// function to assign pixel values to irradiances based on provided exposure times
void assign_pixel_value_to_irradiances_with_exposure(const float* irradiances, const unsigned int num_pixels, const float * i_m, const unsigned int channel, const float time, image_b& image) {
	for (unsigned int i = 0; i < num_pixels; ++i) {
		const float i_value = irradiances[i] * time;
		unsigned int pixel_value = 0;

		// compare the i_value with each I_m value to determine the pixel value
		for (; pixel_value < 255; ++pixel_value) {
			if (i_value <= i_m[pixel_value])
				break;
		}

		image.at(i, channel) = pixel_value;
	}
}

// calculates the objective function for unknown response function
float calculate_objective_function(const float* weights, const float* times, const float* i_m, std::vector<image_b> images, const float* irradiances, const unsigned int num_images, const unsigned int num_pixels) {

	float result = 0;

	for (unsigned int i = 0; i < num_images; ++i) {

		for (unsigned int j = 0; j < num_pixels; ++j) {

			unsigned int index = i * num_pixels + j;
			const unsigned int pixel_value = images[i].at(j, 0);
			if (pixel_value == 255) // weights are definetly 0
				continue;
			float intermediate = i_m[pixel_value] - times[i] * irradiances[j]; 
			
			result += weights[index] * intermediate * intermediate;
			
		}

	}

	return result;

}

// function to output an image according to a linear function
inline void output_linear(const float * irradiances_red, const float * irradiances_green, const float* irradiances_blue, const unsigned int image_width, const unsigned int image_height, const unsigned int num_pixels) {

	// first determine minimal and maximal irradiances for each color
	float max_red = std::numeric_limits<float>::min();
	float min_red = std::numeric_limits<float>::max();
	float max_green = std::numeric_limits<float>::min();
	float min_green = std::numeric_limits<float>::max();
	float max_blue = std::numeric_limits<float>::min();
	float min_blue = std::numeric_limits<float>::max();
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

	// assign pixel values based on min and max
	assign_pixel_value_to_irradiances(irradiances_red, num_pixels, min_red, max_red, 0, output);
	assign_pixel_value_to_irradiances(irradiances_green, num_pixels, min_green, max_green, 1, output);
	assign_pixel_value_to_irradiances(irradiances_blue, num_pixels, min_blue, max_blue, 2, output);

	std::string out_str = "C:\\Users\\migon\\Documents\\Capturing Reality\\output_linear.png";
	image_io::save_png(output, out_str.data());
}

// function to output images with provided exposure times
inline void output_custom_times(const char * path, const char * custom_times, const float* irradiances_red, const float* irradiances_green, const float* irradiances_blue, const float* i_m_red, const float* i_m_green, const float* i_m_blue, const unsigned int image_width, const unsigned int image_height, const unsigned int num_pixels) {
	std::string file_str = std::string(path) + "\\" + std::string(custom_times);
	std::ifstream data;
	data.open(file_str);
	if (!data.is_open()) {
		std::cout << "Could not open " << custom_times << "\n";
		throw std::runtime_error("Could not open exposure times file");
	}
	// counter to name the images
	unsigned int counter = 0;
	while(!data.eof()) {
		image_b output(image_width, image_height, 3);

		float time; 
		data >> time;

		// assign pixel values according to the read exposure time
		assign_pixel_value_to_irradiances_with_exposure(irradiances_red, num_pixels, i_m_red, 0, time, output);
		assign_pixel_value_to_irradiances_with_exposure(irradiances_green, num_pixels, i_m_green, 1, time, output);
		assign_pixel_value_to_irradiances_with_exposure(irradiances_blue, num_pixels, i_m_blue, 2, time, output);

		std::string out_str = std::string(path) + "\\output_custom_time_" + std::to_string(counter++) + ".png";
		image_io::save_png(output, out_str.data());
		std::cout << "saved image with exposure time " << time << "at\n" << out_str << "\n";
	}
}

// function to output the response curve for all 3 colors in one file
inline void output_response_combined(const char* path, const float * i_m_red, const float * i_m_green, const float * i_m_blue) {

	std::ofstream curve_output(std::string(path) + "\\response_curve_combined.txt");
	if (curve_output.is_open()) {
		curve_output << "Red\n";
		for (unsigned int i = 0; i < 255; ++i) {
			std::string line = std::to_string(i) + " " + std::to_string(i_m_red[i]) + "\n";
			curve_output << line;
		}
		curve_output << "Green\n";
		for (unsigned int i = 0; i < 255; ++i) {
			std::string line = std::to_string(i) + " " + std::to_string(i_m_green[i]) + "\n";
			curve_output << line;
		}
		curve_output << "Blue\n";
		for (unsigned int i = 0; i < 255; ++i) {
			std::string line = std::to_string(i) + " " + std::to_string(i_m_blue[i]) + "\n";
			curve_output << line;
		}
	}
	else {
		std::cout << "Failed to write response curve\n";
	}
	curve_output.close();

}

// function to output the response curve for all 3 colors individually
inline void output_response_individual(const char* path, const float* i_m_red, const float* i_m_green, const float* i_m_blue) {

	std::ofstream red_output(std::string(path) + "\\curve_red.dat");
	if (red_output.is_open()) {
		for (unsigned int i = 0; i < 255; ++i) {
			std::string line = std::to_string(i) + "\t" + std::to_string(i_m_red[i]) + "\n";
			red_output << line;
		}
		red_output.close();
	}
	else {
		std::cout << "Failed to write response curve for red\n";
	}

	std::ofstream green_output(std::string(path) + "\\curve_green.dat");
	if (green_output.is_open()) {
		for (unsigned int i = 0; i < 255; ++i) {
			std::string line = std::to_string(i) + "\t" + std::to_string(i_m_green[i]) + "\n";
			green_output << line;
		}
		green_output.close();
	}
	else {
		std::cout << "Failed to write response curve for green\n";
	}

	std::ofstream blue_output(std::string(path) + "\\curve_blue.dat");
	if (blue_output.is_open()) {
		for (unsigned int i = 0; i < 255; ++i) {
			std::string line = std::to_string(i) + "\t" + std::to_string(i_m_blue[i]) + "\n";
			blue_output << line;
		}
		blue_output.close();
	}
	else {
		std::cout << "Failed to write response curve for blue\n";
	}
}

int main(int argc, const char **argv)
{
	if (argc < 3) throw std::runtime_error("Invalid arguments");
	const char* path = argv[1];
	const char* hdrgen = argv[2];

	// custom exposure times to generate images with new times
	const char* new_exposure_times;
	if (argc > 3) {
		new_exposure_times = argv[3];
	}

	// vector to collect the exposure times of the given images
	std::vector<float> times;
	unsigned int num_images = 0;

	// vector of loaded images
	std::vector<image_b> images;

	try {
		images = load_images(path, hdrgen, times, num_images);
	}
	catch (std::runtime_error) {
		std::cout << "Loading images failed\n";
		return EXIT_FAILURE;
	}

	// save dimensions and channels for images
	const unsigned int image_height = images[0].height();
	const unsigned int image_width = images[0].width();
	const unsigned int image_channels = images[0].channels();
	const unsigned int num_pixels = image_height * image_width;

	std::cout << "loaded " << num_images << " images\n";
	std::cout << "height " << image_height << " width " << image_width << " channels " << image_channels << " pixels per image: " << num_pixels << "\n";

	// the algorithm is simply run independently for the 3 rgb color channels
	std::vector<float> weights_red(num_pixels * num_images);
	std::vector<float> weights_green(num_pixels * num_images);
	std::vector<float> weights_blue(num_pixels * num_images);

	// weights and the e_m sets are calculated first, as these never change
	calculate_weights_all_images(images, 0, num_images, num_pixels, weights_red.data());
	calculate_weights_all_images(images, 1, num_images, num_pixels, weights_green.data());
	calculate_weights_all_images(images, 2, num_images, num_pixels, weights_blue.data());

	std::cout << "Finished calculating weights\n";

	unsigned int cardinalities_red[256] = { 0 }; // needs to be initialized with 0s
	unsigned int cardinalities_green[256] = { 0 };
	unsigned int cardinalities_blue[256] = { 0 };

	std::vector <std::vector<std::tuple<unsigned int, unsigned int>>> e_m_red(256);
	std::vector <std::vector<std::tuple<unsigned int, unsigned int>>> e_m_green(256);
	std::vector <std::vector<std::tuple<unsigned int, unsigned int>>> e_m_blue(256);

	fill_e_m(images, 0, num_images, num_pixels, cardinalities_red, e_m_red);
	fill_e_m(images, 1, num_images, num_pixels, cardinalities_green, e_m_green);
	fill_e_m(images, 2, num_images, num_pixels, cardinalities_blue, e_m_blue);

	std::cout << "Finished determining E_m sets\n";

	float i_m_red[255];
	float i_m_green[255];
	float i_m_blue[255];

	// initial values for I_m calculated as linear function
	calculate_initial_i_m(i_m_red);
	calculate_initial_i_m(i_m_green);
	calculate_initial_i_m(i_m_blue);

	std::cout << "Finished calculating initial I_m values\n";

	std::vector<float> irradiances_red(num_pixels);
	std::vector<float> irradiances_green(num_pixels);
	std::vector<float> irradiances_blue(num_pixels);

	// irradiances calculated with the initial I_m values
	calculate_irradiances(weights_red.data(), times.data(), i_m_red, images, 0, num_images, num_pixels, irradiances_red.data());
	calculate_irradiances(weights_green.data(), times.data(), i_m_green, images, 1, num_images, num_pixels, irradiances_green.data());
	calculate_irradiances(weights_blue.data(), times.data(), i_m_blue, images, 2, num_images, num_pixels, irradiances_blue.data());

	std::cout << "Finished calculating initial irradiances\n";

	std::cout << "Entering loop\n";

	// threshold per pixel
	float threshold_value = .01;
	// threshold adjusted for the total number of pixels
	float threshold = threshold_value * static_cast<float>(num_pixels * num_images);

	// limited to 5 iterations
	for (unsigned int i = 0; i < 10; ++i) {

		std::cout << "Iteration " << i << "\n";

		// calculate I_m values
		calculate_i_m_values(cardinalities_red, e_m_red, times.data(), irradiances_red.data(), i_m_red);
		calculate_i_m_values(cardinalities_green, e_m_green, times.data(), irradiances_green.data(), i_m_green);
		calculate_i_m_values(cardinalities_blue, e_m_blue, times.data(), irradiances_blue.data(), i_m_blue);

		std::cout << "Finished calculating I_m values\n";

		// scale I_m values such that I_128 = 1.0
		scale_i_m(i_m_red);
		scale_i_m(i_m_green);
		scale_i_m(i_m_blue);

		std::cout << "Finished scaling I_m values\n";

		// calculate irradiances
		calculate_irradiances(weights_red.data(), times.data(), i_m_red, images, 0, num_images, num_pixels, irradiances_red.data());
		calculate_irradiances(weights_green.data(), times.data(), i_m_green, images, 1, num_images, num_pixels, irradiances_green.data());
		calculate_irradiances(weights_blue.data(), times.data(), i_m_blue, images, 2, num_images, num_pixels, irradiances_blue.data());

		std::cout << "Finished calculating irradiances\n";

		// calculate objective function
		float result_red = calculate_objective_function(weights_red.data(), times.data(), i_m_red, images, irradiances_red.data(), num_images, num_pixels);
		float result_green = calculate_objective_function(weights_green.data(), times.data(), i_m_green, images, irradiances_green.data(), num_images, num_pixels);
		float result_blue = calculate_objective_function(weights_blue.data(), times.data(), i_m_blue, images, irradiances_blue.data(), num_images, num_pixels);

		std::cout << "Threshold is " << threshold_value << " per pixel, total is " << threshold << "\n";
		std::cout << "Finished calculating objective function for red, result: " << result_red << "\n";
		std::cout << "Finished calculating objective function for green, result: " << result_green << "\n";
		std::cout << "Finished calculating objective function for blue, result: " << result_blue << "\n";

		// check if result of objective function is below threshold adjusted for total number of pixels
		if (result_red < threshold && result_green < threshold && result_blue < threshold)
			break; 
		
	}

	// output the 3 response curves within a single file
	output_response_combined(path, i_m_red, i_m_green, i_m_blue);

	// output each curve individually
	output_response_individual(path, i_m_red, i_m_green, i_m_blue);

	// output an image where irradiances are linearly mapped to pixel values
	output_linear(irradiances_red.data(), irradiances_green.data(), irradiances_blue.data(), image_width, image_height, num_pixels);

	// output replications of the original images
	for (unsigned int i = 0; i < num_images; ++i) {
		image_b output(image_width, image_height, 3);

		// assign pixel values according to the original exposure times
		assign_pixel_value_to_irradiances_with_exposure(irradiances_red.data(), num_pixels, i_m_red, 0, times[i], output);
		assign_pixel_value_to_irradiances_with_exposure(irradiances_green.data(), num_pixels, i_m_green, 1, times[i], output);
		assign_pixel_value_to_irradiances_with_exposure(irradiances_blue.data(), num_pixels, i_m_blue, 2, times[i], output);

		std::string out_str = std::string(path) + "\\output_" + std::to_string(i) + ".png";
		image_io::save_png(output, out_str.data());
		std::cout << "saved image with exposure time "<< times[i] << "at\n" << out_str << "\n";
	}

	// output images with custom exposure times
	if (argc > 3)
		output_custom_times(path, new_exposure_times, irradiances_red.data(), irradiances_green.data(), irradiances_blue.data(), i_m_red, i_m_green, i_m_blue, image_width, image_height, num_pixels);
}