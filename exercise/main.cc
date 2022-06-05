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

std::vector<std::string> getFilenames(const char* path) {
	// open the .hdrgen file
	std::string file_str = std::string(path) + "/hdrgen.txt";
	std::ifstream data;
	data.open(file_str);
	if (!data.is_open()) {
		std::cout << "Could not open " << file_str << "\n";
		throw std::runtime_error("Could not open hdrgen file");
	}

	std::vector<std::string> filenames;

	while (!data.eof()) {
		std::string filename;
		data >> filename;
		filenames.push_back(filename);
	}

	return filenames;
}

image<unsigned char> convertToPanoramaTwoCameras(image<unsigned char> input_front, image<unsigned char> input_back) {
	int front_width = input_front.width(), front_height = input_front.height();
	int back_width = input_back.width(), back_height = input_back.height();
	cout << "front: " << front_width << " " << front_height << endl;
	cout << "back: " << back_width << " " << back_height << endl;
	if (front_width != back_width || front_height != back_height)
		cout << "Image dimensions don't match\n";

	int radius_front = front_width / 2;
	int radius_back = back_width / 2;

	int width_max = std::max(front_width, back_width);
	int height_max = std::max(front_height, back_height);

	int output_width = width_max * PI;
	int output_height = height_max;

	int center_x = output_width / 2, center_z = output_height / 2;

	double radius_cylinder = output_width / (2 * PI);

	Eigen::Vector3d ray_front = { 0, 1, 0 };
	Eigen::Vector3d ray_back = { 0, -1, 0 };

	image<unsigned char> output_img(output_width, output_height, 3);

	for (int i = 0; i < output_width; i++)
	{
		for (int j = 0; j < output_height; j++)
		{
			double angle = 2 * PI * (i / ((double)output_width + 1));

			double a = -radius_cylinder * sin(angle);
			double b = -radius_cylinder * cos(angle);
			double c = -static_cast<double>(j) + output_height / 2;

			// if b > 0, then the point is behind the sphere in the front image
			bool back = b > 0.f;

			// the radius for the imaged sphere, use front as standard and switch
			// if the point is behind front sphere
			int radius_sphere = radius_front;

			// the image used to get the pixel values, use front as standard and switch
			// if the point is behind front sphere
			image<unsigned char>* input_image = &input_front;

			// reverse the vector if the point is behind the sphere
			// this way, it should act as the proper vector for the back image
			if (back) {
				a = -a;
				b = -b;
				// don't reverse c or the back half is upside down

				radius_sphere = radius_back;

				input_image = &input_back;
			}

			Eigen::Vector3d ray_cylinder = { a, b, c };
			ray_cylinder = ray_cylinder / ray_cylinder.norm();

			a = ray_cylinder(0);
			b = ray_cylinder(1);
			c = ray_cylinder(2);

			double normal_y = sqrt((1 - b) / 2);
			double normal_x;
			double normal_z;

			// if the y-component of the normal is 0, then the point 
			// on the cylinder should be exactly behind the center of the sphere
			if (normal_y != 0.f) {
				normal_x = -a / (2 * normal_y);
				normal_z = -c / (2 * normal_y);
			}
			// set x and z arbitrarily at the very edge of the sphere on the input image
			else {
				normal_x = 1.f;
				normal_z = 0.f;
			}

			Eigen::Vector3d normal_sphere = { normal_x, normal_y, normal_z };
			normal_sphere = (radius_sphere / normal_sphere.norm()) * normal_sphere;

			int input_x = normal_sphere(0) + front_width / 2;
			int input_y = normal_sphere(2) + front_width / 2;

			int output_x = (i + output_width / 2) % output_width;

			unsigned char c_red = input_image->at2d(input_x, input_y, 0);
			output_img.at2d(output_x, j, 0) = c_red;

			unsigned char c_green = input_image->at2d(input_x, input_y, 1);
			output_img.at2d(output_x, j, 1) = c_green;

			unsigned char c_blue = input_image->at2d(input_x, input_y, 2);
			output_img.at2d(output_x, j, 2) = c_blue;
		}
	}

	return output_img;
}

image<unsigned char> convertToPanorama(image<unsigned char> input_img) {
	int input_width = input_img.width(), input_height = input_img.height();
	cout << input_width << " " << input_height << endl;

	int radius_sphere = input_width / 2;

	int output_width = input_width * PI;//PI * width; // choice
	int output_height = input_height;

	int center_x = output_width / 2, center_z = output_height / 2;

	double radius_cylinder = output_width / (2 * PI);
	//double n = 2 * rc;

	image<unsigned char> output_img(output_width, output_height, 3);

	for (int i = 0; i < output_width; i++)
	{
		for (int j = 0; j < output_height; j++)
		{
			//int x = (i - center_x), z = (j - center_z);
			double angle = 2 * PI * (i / ((double)output_width + 1));

			double a = -radius_cylinder * sin(angle);
			double b = -radius_cylinder * cos(angle);
			double c = -static_cast<double>(j) + output_height / 2;

			//printf("a=%lf, b=%lf, c=%lf", a, b, c);

			Eigen::Vector3d ray_cylinder = { a, b, c };
			ray_cylinder = ray_cylinder / ray_cylinder.norm();

			a = ray_cylinder(0);
			b = ray_cylinder(1);
			c = ray_cylinder(2);

			double normal_y = sqrt((1 - b) / 2);
			double normal_x;
			double normal_z;

			// if the y-component of the normal is 0, then the point 
			// on the cylinder should be exactly behind the center of the sphere
			if (normal_y != 0.f) {
				normal_x = -a / (2 * normal_y);
				normal_z = -c / (2 * normal_y);
			} 
			// set x and z arbitrarily at the very edge of the sphere on the input image
			else {
				normal_x = 1.f;
				normal_z = 0.f;
			}

			Eigen::Vector3d normal_sphere = { normal_x, normal_y, normal_z };
			normal_sphere = (radius_sphere / normal_sphere.norm()) * normal_sphere;

			int input_x = normal_sphere(0) + input_width / 2;
			int input_y = normal_sphere(2) + input_width / 2;

			int output_x = (i + output_width / 2) % output_width;

			unsigned char c_red = input_img.at2d(input_x, input_y, 0);
			output_img.at2d(output_x, j, 0) = c_red;

			unsigned char c_green = input_img.at2d(input_x, input_y, 1);
			output_img.at2d(output_x, j, 1) = c_green;

			unsigned char c_blue = input_img.at2d(input_x, input_y, 2);
			output_img.at2d(output_x, j, 2) = c_blue;
		}
	}

	return output_img;
}

int main(int argc, const char **argv)
{
// 	if (argc != 2) throw std::runtime_error("Invalid arguments");
// 	example(argv[1]);

	const char *path = argv[1];
	const char *out_path = argv[2];

	bool sequence = false;
	bool two_images = false;
	if (argc > 3) {
		std::string argv3 = argv[3];
		sequence = argv3 == "-s";
		two_images = argv3 == "-t";
	}

	if (!sequence && !two_images) {
		image<unsigned char> output_img = convertToPanorama(image_io::load(path));

		image_io::save_png(output_img, out_path);
	}
	else if (sequence) {
		std::vector<std::string> files = getFilenames(path);

		for (unsigned int i = 0; i < files.size(); ++i) {
			std::string in_filename = std::string(path) + "/" + files[i];
			
			image<unsigned char> output_img = convertToPanorama(image_io::load(in_filename.data()));

			std::string out_filename = std::string(out_path) + "/" + files[i];
			image_io::save_jpeg(output_img, out_filename.data());
		}
	}
	else if (two_images) {
		const char* path_second = argv[4];

		image<unsigned char> input_front = image_io::load(path);
		image<unsigned char> input_back = image_io::load(path_second);

		image<unsigned char> output_img = convertToPanoramaTwoCameras(input_front, input_back);

		image_io::save_png(output_img, out_path);
	}

	cout << "ok" << endl;

	return EXIT_SUCCESS;
}