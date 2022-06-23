#include "image.h"

#include "ply.h"

#include <stdexcept>
#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>

#define NUM_INPUTS 253
#define NUM_OUTPUTS 360
#define PI 3.14159265851979323
#define TWO_PI 6.28318530717958647

void load_images(const char* path, const char* filename, image_b* images, const int naming_scheme = 3) {

	for (int i = 0; i < NUM_INPUTS; ++i) {
		std::stringstream stream;
		stream << path << "/" << filename << "_" << std::setw(naming_scheme) << std::setfill('0') << i << ".png";
		std::string file = stream.str();

		// std::cout << "FILENAME: " << file << std::endl;
		// std::cout << "path: " << path << std::endl;

		images[i] = image_io::load(file.c_str());
		std::cout << "Loaded image " << (i + 1) << " of " << NUM_INPUTS << "\n";
	}
	std::cout << "\n";

}

void save_image(const char* path, const char* filename, const image_b &image, const int num) {
	const int naming_scheme = ceil(log10(NUM_OUTPUTS));

	std::stringstream stream;
	stream << path << "/" << filename << "_reluminated_" << std::setw(naming_scheme) << std::setfill('0') << num << ".png";
	std::string file = stream.str();
	image_io::save_png(image, file.c_str());

	std::cout << "Saved image as " << file << "\n";
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
	std::cout << "Loaded directions\n\n";
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

	std::cout << "Calculated solid angles\n";
}

inline Eigen::Matrix3d rotationMatrixZ(const double angle) {
	const double sinZ = sin(angle);
	const double cosZ = cos(angle);
	Eigen::Matrix3d rot_mat;
	rot_mat << 
		cosZ, -sinZ, 0.0,
		sinZ, cosZ, 0.0,
		0.0, 0.0, 1.0;
	return rot_mat;
}

inline double toUnitInterval(const unsigned char value) {
	const double value_d = static_cast<double>(value);
	return value_d / 255.f;
}

void determineEnvironmentMap(const image_b image, const Eigen::Vector3d* directions, const double* solid_angles, Eigen::Vector3d* environment_map, const Eigen::Matrix3d displacement = Eigen::Matrix3d::Identity()) {
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

		double c_red = toUnitInterval(image.at2d(input_x, input_y, 0));

		double c_green = toUnitInterval(image.at2d(input_x, input_y, 1));

		double c_blue = toUnitInterval(image.at2d(input_x, input_y, 2));

		const double solid_angle = solid_angles[i];

		environment_map[i] = { c_red * solid_angle, c_green * solid_angle, c_blue * solid_angle };
		//environment_map[i] = {c_red, c_green, c_blue};
	}
}

inline unsigned char getR_xy(const image_b* images, const int image, const int x, const int y, const int channel) {
	image_b current_image = images[image];

	return current_image.at2d(x, y, channel);
}

void reluminate_images(const std::vector<image_b> input_images, const image_b env_img, const std::vector<Eigen::Vector3d> directions, const char* out_path, const char* filename, std::vector<double> solid_angles)
{
	int width = input_images[0].width();
	int height = input_images[0].height();

	//std::vector<image_b> reluminated_images(NUM_INPUTS, image_b(width, height, 3));
	image_b reluminated_image(width, height, 3);

	for(int i = 0; i < NUM_OUTPUTS; i++)
	{
		//image_b reluminated_image(width, height, 3);

		std::vector<Eigen::Vector3d> env_map(NUM_INPUTS);
		const double angle = (static_cast<double>(i) / static_cast<double>(NUM_OUTPUTS)) * TWO_PI;
		const Eigen::Matrix3d rot_matrix = rotationMatrixZ(angle);

		// determine NORMALIZED environment map
		determineEnvironmentMap(env_img, directions.data(), solid_angles.data(), env_map.data(), rot_matrix);

		std::cout << "Determined evnironment map " << (i + 1) << " of " << NUM_OUTPUTS << "\n";

		//std::cout << "solid_angle: " << solid_angle << "\n";

		std::chrono::high_resolution_clock::time_point begin, end;
		begin = std::chrono::steady_clock::now();


		for (int j = 0; j < width; j++)
		{
			for (int k = 0; k < height; k++)
			{
				double r = 0.0, g = 0.0, b = 0.0;

				for (int h = 0; h < NUM_INPUTS; ++h) {
					const image_b* input_image = &input_images[h];

					const double r_d = toUnitInterval(input_image->at2d(j, k, 0));
					const double g_d = toUnitInterval(input_image->at2d(j, k, 1));
					const double b_d = toUnitInterval(input_image->at2d(j, k, 2));

					// std::cout << "r: " << (int) r << "\n";
					// std::cout << "g: " << (int) g << "\n";
					// std::cout << "b: " << (int) b << "\n\n";

					const Eigen::Vector3d map_vector = env_map[i];

					r += r_d * map_vector(0);
					g += g_d * map_vector(1);
					b += b_d * map_vector(2);
				}
				
				if (r > 255.f)
					printf("(%i, %i) red value is %f\n", j, k, r);
				if (g > 255.f)
					printf("(%i, %i) green value is %f\n", j, k, g);
				if (b > 255.f)
					printf("(%i, %i) blue value is %f\n", j, k, b);

				reluminated_image.at2d(j, k, 0) = static_cast<unsigned char>(r);
				reluminated_image.at2d(j, k, 1) = static_cast<unsigned char>(g);
				reluminated_image.at2d(j, k, 2) = static_cast<unsigned char>(b);

			}			
			/*end = std::chrono::steady_clock::now();
			std::cout << "Image " << i << " line " << j << " of " << (width - 1) << ") took "
				<< std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "ns\n";
			begin = end;*/
		}		

		//reluminated_images.push_back(reluminated_image);
		save_image(out_path, filename, reluminated_image, i);

		end = std::chrono::steady_clock::now();
		std::cout << "Done with image " << (i + 1) << " of " << NUM_OUTPUTS << " took "
			<< std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "ns\n";
		begin = end;

		//std::cout << "Done with image " << (i + 1) << " of " << NUM_OUTPUTS << "\n"; 
	}

	//return reluminated_images;
}

int main(int argc, const char **argv)
{
	if (argc < 6) {
		std::cout << "Not enough arguments\n";
	}

	const char* path = argv[1];
	const char* filename = argv[2];
	const char* directions_file = argv[3];
	const char* environment_file = argv[4];
	const char* out_path = argv[5];
	int naming_scheme;
	if(argc > 6)
		naming_scheme = std::stoi(std::string(argv[6]));


	// std::cout << "FILENAME: " << filename << std::endl;
	// std::cout << "path: " << path << std::endl;
	// std::cout << "directions_file: " << directions_file << std::endl;

	std::vector<image_b> input_images(NUM_INPUTS);
	if(argc > 5)
		load_images(path, filename, input_images.data(), naming_scheme);
	else
		load_images(path, filename, input_images.data());

	std::vector<Eigen::Vector3d> directions(NUM_INPUTS);
	load_directions(directions_file, directions.data());

	std::vector<double> solid_angles(NUM_INPUTS);
	calculateSolidAngles(directions.data(), solid_angles.data());

	const image_b environment_image = image_io::load(environment_file);

	// std::stringstream stream;
	// stream << path << "/out/" << filename << "_" << std::setw(3) << std::setfill('0') << 0 << "_out.png";
	// std::string out_file = stream.str();

	

	//std::vector<image_b> reluminated_images = 
	reluminate_images(input_images, environment_image, directions, out_path, filename, solid_angles);
	


	//image_io::save_png(reluminated_images[0], out_file.c_str());

	/*for (int i = 0; i < NUM_INPUTS; ++i) {
		std::stringstream stream;
		stream << path << "/out/" << filename << "_" << std::setw(3) << std::setfill('0') << i << ".png";
		std::string out_file = stream.str();

		//images[i] = image_io::load(file.c_str());
		image_io::save_png(reluminated_images[i], out_file.c_str());

		std::cout << out_file << "\n";
	}*/



	return EXIT_SUCCESS;
}