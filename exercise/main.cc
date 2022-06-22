#include "image.h"

#include "ply.h"

#include <stdexcept>
#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <experimental/filesystem>
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
		stream << path << "/" << filename << "_" << std::setw(3) << std::setfill('0') << i << ".png";
		std::string file = stream.str();


		// std::cout << "FILENAME: " << file << std::endl;
		// std::cout << "path: " << path << std::endl;

		images[i] = image_io::load(file.c_str());
	}

}

void load_directions(const char* filename, Eigen::Vector3d* directions) {
	// open the .hdrgen file
	std::ifstream data;

	//std::cout << "FILENAME: " << filename << std::endl;

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
		double vec0, vec1, vec2;
		if (!(stream >> num >> vec0 >> vec1 >> vec2)) {
			std::cout << "Invalid vector\n";
			throw std::runtime_error("Invalid vector");
		}

		Eigen::Vector3d vector({vec0, vec1, vec2});

		directions[i] = vector;

	}
	data.close();
}

void calculateSolidAngles(const Eigen::Vector3d* directions, double* solid_angles) {
	
	const Eigen::Vector3d z_axis({ 0.f, 0.f, 1.f });
	for (int i = 0; i < NUM_INPUTS; ++i) {
		const Eigen::Vector3d direction = directions[i];

		const double enumerator = (direction.cross(z_axis)).norm();
		const double denominator = direction.norm(); // norm of z_axis is 1

		const double sine_phi = enumerator / denominator;
		solid_angles[i] = sine_phi;
	}

}

void determineEnvironmentMap(const image_b image, const Eigen::Vector3d* directions, Eigen::Vector3d* environment_map, const Eigen::Matrix3d displacement = Eigen::Matrix3d::Identity()) {
	int input_width = image.width(), input_height = image.height();

	int radius_sphere = input_width / 2;

	for (int i = 0; i < NUM_INPUTS; i++)
	{

		Eigen::Vector3d direction = displacement * directions[i];

		double a = direction(0);
		double b = direction(1);
		double c = direction(2);

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

		double c_red = image.at2d(input_x, input_y, 0);

		double c_green = image.at2d(input_x, input_y, 1);

		double c_blue = image.at2d(input_x, input_y, 2);

		environment_map[i] = { c_red, c_green, c_blue };
	}
}

inline unsigned char getR_xy(const image_b* images, const int image, const int x, const int y, const int channel) {
	image_b current_image = images[image];

	return current_image.at2d(x, y, channel);
}

std::vector<image_b> reluminate_images(std::vector<image_b> input_images, std::vector<Eigen::Vector3d> env_map, std::vector<double> solid_angles)
{
	std::vector<image_b> reluminated_images(NUM_INPUTS);

	int width = input_images[0].width();
	int height = input_images[0].height();

	for(int i = 0; i < NUM_INPUTS; i++)
	{
		image_b reluminated_image(width, height, 3);

		//std::cout << "solid_angle: " << solid_angle << "\n";

		for (int j = 0; j < width; j++)
		{
			for (int k = 0; k < height; k++)
			{
				double r = 0.0, g = 0.0, b = 0.0;

				for (int h = 0; h < NUM_INPUTS; ++h) {
					const image_b input_image = input_images[h];
					const double solid_angle = solid_angles[h];
					const Eigen::Vector3d normalized_map = env_map[h] * solid_angle;

					const double r_d = input_image.at2d(j, k, 0);
					const double g_d = input_image.at2d(j, k, 1);
					const double b_d = input_image.at2d(j, k, 2);

					// std::cout << "r: " << (int) r << "\n";
					// std::cout << "g: " << (int) g << "\n";
					// std::cout << "b: " << (int) b << "\n\n";

					r += r_d * normalized_map(0);
					g += g_d * normalized_map(1);
					b += b_d * normalized_map(2);
				}
				
				reluminated_image.at2d(j, k, 0) = static_cast<unsigned char>(r);
				reluminated_image.at2d(j, k, 1) = static_cast<unsigned char>(g);
				reluminated_image.at2d(j, k, 2) = static_cast<unsigned char>(b);

			}			
		}		

		reluminated_images.push_back(reluminated_image);
	}

	return reluminated_images;
}

int main(int argc, const char **argv)
{
	const char* path = argv[1];
	const char* filename = argv[2];
	const char* directions_file = argv[3];
	const char* environment_file = argv[4];
	//const char* out_path = argv[3];

	// std::cout << "FILENAME: " << filename << std::endl;
	// std::cout << "path: " << path << std::endl;
	// std::cout << "directions_file: " << directions_file << std::endl;

	std::vector<image_b> input_images(NUM_INPUTS);
	load_images(path, filename, input_images.data());

	std::vector<Eigen::Vector3d> directions(NUM_INPUTS);
	load_directions(directions_file, directions.data());

	std::vector<double> solid_angles(NUM_INPUTS);
	calculateSolidAngles(directions.data(), solid_angles.data());

	image_b environment_image = image_io::load(environment_file);
	std::vector<Eigen::Vector3d> environment_map(NUM_INPUTS);
	determineEnvironmentMap(environment_image, directions.data(), environment_map.data());

	// std::stringstream stream;
	// stream << path << "/out/" << filename << "_" << std::setw(3) << std::setfill('0') << 0 << "_out.png";
	// std::string out_file = stream.str();

	

	std::vector<image_b> reluminated_images = reluminate_images(input_images, environment_map, solid_angles);
	


	//image_io::save_png(reluminated_images[0], out_file.c_str());

	for (int i = 0; i < NUM_INPUTS; ++i) {
		std::stringstream stream;
		stream << path << "/out/" << filename << "_" << std::setw(3) << std::setfill('0') << i << ".png";
		std::string out_file = stream.str();

		//images[i] = image_io::load(file.c_str());
		image_io::save_png(reluminated_images[i], out_file.c_str());

		std::cout << out_file << "\n";
	}



	return EXIT_SUCCESS;
}