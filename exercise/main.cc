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
#define GAMMA 2.2
#define PI 3.14159265851979323
#define TWO_PI 6.28318530717958647

typedef Eigen::Matrix<unsigned char, 3, 1> RGBVector;

inline double custom_pow(double value) {
	union {
		double d;
		int x[2];
	} u = { value };
	u.x[1] = (int)(GAMMA * (u.x[1] - 1072632447) + 1072632447);
	u.x[0] = 0;
	return u.d;
}

inline double toUnitInterval(const unsigned char value) {
	const double value_d = static_cast<double>(value);
	return value_d / 255.0;
}

inline double toUnitInterval_gamma_correction(const unsigned char value) {
	const double value_d = static_cast<double>(value);
	const double unit_value = value_d / 255.0;
	const double gamma_corrected = std::pow(unit_value, GAMMA);
	//if (gamma_corrected - custom_pow(unit_value) > 0.00001) {
	//	std::cout << "Error: " << (abs(gamma_corrected - custom_pow(unit_value))) << "\n";
	//}
	return gamma_corrected;
}

inline double toByteValues(const double value) {
	const double intermediate = value * 255.f;
	return static_cast<unsigned char>(intermediate);

}

std::pair<int, int> get_dimensions(const char* path, const char* filename, const int naming_scheme = 3) {
	std::stringstream stream;
	stream << path << "/" << filename << "_" << std::setw(naming_scheme) << std::setfill('0') << 0 << ".png";
	std::string file = stream.str();

	const image_b image = image_io::load(file.c_str());

	return std::pair<int, int>(image.width(), image.height());
}

inline void normalize_color(const Eigen::Vector3d *intensity, unsigned char &r, unsigned char &g, unsigned char &b) {
	r = static_cast<unsigned char>(static_cast<double>(r) * (1.0 / (*intensity)(0)));
	g = static_cast<unsigned char>(static_cast<double>(g) * (1.0 / (*intensity)(1)));
	b = static_cast<unsigned char>(static_cast<double>(b) * (1.0 / (*intensity)(2)));

	/*const double norm = intensity->norm();
	const double normalizer = (1.0 / norm);

	r = static_cast<unsigned char>(static_cast<double>(r) * normalizer);
	g = static_cast<unsigned char>(static_cast<double>(g) * normalizer);
	b = static_cast<unsigned char>(static_cast<double>(b) * normalizer);*/
}

void load_reflectance_functions(const char* path, const char* filename, const Eigen::Vector3d* intensities, const unsigned char* matte, std::vector<RGBVector>* r_xy, const int naming_scheme = 3) {
	int width = 0, height = 0;
	std::chrono::high_resolution_clock::time_point begin, end;
	begin = std::chrono::steady_clock::now();
	for (int i = 0; i < NUM_INPUTS; ++i) {
		std::stringstream stream;
		stream << path << "/" << filename << "_" << std::setw(naming_scheme) << std::setfill('0') << i << ".png";
		std::string file = stream.str();

		// std::cout << "FILENAME: " << file << std::endl;
		// std::cout << "path: " << path << std::endl;

		const image_b image = image_io::load(file.c_str());
		if (!width) {
			width = image.width();
			height = image.height();
		}

		const Eigen::Vector3d* intensity = &intensities[i];

		for (int j = 0; j < width; ++j) {
			for (int k = 0; k < height; ++k) {
				if (matte[k * width + j]) {
					continue;
				}
				unsigned char r = image.at2d(j, k, 0);
				unsigned char g = image.at2d(j, k, 1);
				unsigned char b = image.at2d(j, k, 2);

				normalize_color(intensity, r, g, b);

				const double r_corrected = toUnitInterval_gamma_correction(r);
				const double g_corrected = toUnitInterval_gamma_correction(g);
				const double b_corrected = toUnitInterval_gamma_correction(b);

				r = toByteValues(r_corrected);
				g = toByteValues(g_corrected);
				b = toByteValues(b_corrected);

				r_xy[k * width + j][i] = { r, g, b };
				//std::cout << r:xy[k * width + k]
			}
		}
		//std::cout << "Loaded function " << (i + 1) << " of " << NUM_INPUTS << "\n";
	}

	end = std::chrono::steady_clock::now();
	std::cout << "Loaded reflectance functions took "
		<< std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "ns\n";
	//std::cout << "Loaded reflectance functions\n";
}

void save_image(const char* path, const char* filename, const image_b &image, const int num) {
	const int naming_scheme = ceil(log10(NUM_OUTPUTS));

	std::stringstream stream;
	stream << path << "/" << filename << "_reluminated_" << std::setw(naming_scheme) << std::setfill('0') << num << ".png";
	std::string file = stream.str();
	image_io::save_png(image, file.c_str());

	//std::cout << "Saved image as " << file << "\n";
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

void load_intensities(const char* filename, Eigen::Vector3d* intensities) {
	// open the .hdrgen file
	std::ifstream data;

	//std::cout << "FILENAME: " << filename << std::endl;

	data.open(filename);
	if (!data.is_open()) {
		std::cout << "Could not open " << filename << "\n";
		throw std::runtime_error("Could not open intensities file");
	}

	// open all images named in the file
	for (int i = 0; i < NUM_INPUTS; ++i) {

		std::string line;
		if (!std::getline(data, line)) {
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

		Eigen::Vector3d vector({ vec0, vec1, vec2 });

		intensities[i] = vector;

	}
	data.close();
	std::cout << "Loaded intensities\n\n";
}

void load_matte(const char* filename, unsigned char* matte) {
	const image_b matte_image = image_io::load(filename);
	int size = matte_image.pixels();
	for (int i = 0; i < size; ++i) {
		matte[i] = matte_image.at(i);
	}
}

void calculateSolidAngles(const Eigen::Vector3d* directions, double* solid_angles) {
	
	const Eigen::Vector3d z_axis({ 0.f, 0.f, 1.f });
	for (int i = 0; i < NUM_INPUTS; ++i) {
		const Eigen::Vector3d direction = -directions[i];

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

void determineEnvironmentMap(const image_b image, const Eigen::Vector3d* directions, const double* solid_angles, Eigen::Vector3d* environment_map, const Eigen::Matrix3d displacement = Eigen::Matrix3d::Identity()) {
	int input_width = image.width(), input_height = image.height();

	int radius_sphere = input_width / 2;

	std::vector<Eigen::Vector3d> directions_turned(NUM_INPUTS);

	for (int i = 0; i < NUM_INPUTS; i++)
	{
		directions_turned[i] = -(displacement * directions[i]);
	}

	const Eigen::Vector3d camera_ray({ 0.0, -1.0, 0.0 });

	std::vector<int> pixel_counter(NUM_INPUTS, 0);

	for (int i = 0; i < input_width; ++i) {
		for (int j = 0; j < input_height; ++j) {
			int x_int = -radius_sphere + i;
			int z_int = radius_sphere - j;
			if (x_int * x_int + z_int * z_int > radius_sphere * radius_sphere) {
				continue;
			}
			const double x = static_cast<double>(x_int);
			const double z = static_cast<double>(z_int);
			const double temp = static_cast<double>(radius_sphere * radius_sphere) - (x * x + z * z);
			const double y = std::sqrt(abs(temp));

			Eigen::Vector3d normal({ x, y, z });
			normal.normalize();

			const Eigen::Vector3d direction = -(camera_ray - 2 * (camera_ray.dot(normal)) * normal).normalized();

			double closest_distance = 1000.0;
			int closest_index = 0;

			for (int k = 0; k < NUM_INPUTS; ++k) {
				const Eigen::Vector3d direction_turned = directions_turned[k];
				const double distance = abs((direction_turned - direction).norm());
				if (distance < closest_distance) {
					closest_distance = distance;
					closest_index = k;
				}
			}

			//std::cout << "Pixel " << i << ", " << j << " direction was:\n" << direction << "\n" 
			//	<< "Closest was direction " << closest_index << ":\n" << directions_turned[closest_index] << "\n";

			double r = toUnitInterval(image.at2d(i, j, 0));
			double g = toUnitInterval(image.at2d(i, j, 1));
			double b = toUnitInterval(image.at2d(i, j, 2));

			const Eigen::Vector3d color({ r, g, b });
			
			/*if (color.norm() > environment_map[closest_index].norm()) {
				environment_map[closest_index] = color;
			}*/
			environment_map[closest_index] += color;
			pixel_counter[closest_index]++;
		}
	}

	for (int i = 0; i < NUM_INPUTS; ++i) {
		//std::cout << "Before:\n" << environment_map[i] << "\n\n";
		environment_map[i] /= static_cast<double>(pixel_counter[i]);
		environment_map[i] *= solid_angles[i];
		//std::cout << "After:\n" << environment_map[i] << "\n\n";
		if (pixel_counter[i] == 0) {
			std::cout << "Direction " << i << "had no closest pixel\n";
		}
		std::cout << environment_map[i] << "\n\n";
	}
	//std::cout << environment_map[0] << "\n";

	/*for (int i = 0; i < NUM_INPUTS; i++)
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
		if (normal_y != 0.0) {
			normal_x = -a / (2 * normal_y);
			normal_z = -c / (2 * normal_y);
		}
		// set x and z arbitrarily at the very edge of the sphere on the input image
		else {
			normal_x = 1.0;
			normal_z = 0.0;
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
		//std::cout << environment_map[i] << "\n";
		//environment_map[i] = {c_red, c_green, c_blue};
	}*/
}

image<unsigned char> convertToPanorama(image<unsigned char> input_img, const Eigen::Vector3d* directions) {
	int input_width = input_img.width(), input_height = input_img.height();

	int radius_sphere = input_width / 2;

	int output_width = input_width * PI;//PI * width; // choice
	int output_height = input_height * 8;

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
	int counter = 0;
	bool got = false;
	for (int i = 0; i < NUM_INPUTS; ++i) {

		Eigen::Vector3d direction = -directions[i];

		for (int j = 0; j < output_width; ++j) {
			for (int k = 0; k < output_height; ++k) {
				double angle = 2 * PI * (j / ((double)output_width + 1));

				double a = -radius_cylinder * sin(angle);
				double b = -radius_cylinder * cos(angle);
				double c = -static_cast<double>(k) + output_height / 2;

				Eigen::Vector3d ray_cylinder = { a, b, c };
				ray_cylinder = ray_cylinder / ray_cylinder.norm();

				if ((direction - ray_cylinder).norm() < 0.05) {

					if (!got) {
						got = true;
						++counter;
					}
					int output_x = (j + output_width / 2) % output_width;
					int output_y = k;

					output_img.at2d(output_x, output_y, 0) = 255;

					output_img.at2d(output_x, output_y, 1) = 255;

					output_img.at2d(output_x, output_y, 2) = 255;
					//std::cout << "aoiasfioahiaf\n";
				}
				//printf("a=%lf, b=%lf, c=%lf", a, b, c);

			}
		}
		got = false;

	}
	std::cout << counter << "\n";
	return output_img;
}

void reluminate_images_alt(const std::vector<std::vector<RGBVector>>& reflectance_functions, const image_b env_img, const std::vector<Eigen::Vector3d> directions, const unsigned char* matte, const char* out_path, const char* filename, const int width, const int height, std::vector<double> solid_angles)
{
	//std::vector<image_b> reluminated_images(NUM_INPUTS, image_b(width, height, 3));
	image_b reluminated_image(width, height, 3);

	std::chrono::high_resolution_clock::time_point begin, end;
	for (int i = 0; i < NUM_OUTPUTS; i++)
	{
		//image_b reluminated_image(width, height, 3);

		std::vector<Eigen::Vector3d> env_map(NUM_INPUTS, Eigen::Vector3d({0.0, 0.0, 0.0}));
		const double angle = (static_cast<double>(i) / static_cast<double>(NUM_OUTPUTS)) * TWO_PI;
		const Eigen::Matrix3d rot_matrix = rotationMatrixZ(angle);

		// determine NORMALIZED environment map
		begin = std::chrono::steady_clock::now();
		determineEnvironmentMap(env_img, directions.data(), solid_angles.data(), env_map.data(), rot_matrix);
		end = std::chrono::steady_clock::now();

		std::cout << "Determined evnironment map " << (i + 1) << " of " << NUM_OUTPUTS << "\n";
		std::cout << "Took " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "ns\n";

		//std::cout << env_map[0] << "\n";
		//std::cout << "solid_angle: " << solid_angle << "\n";

		begin = std::chrono::steady_clock::now();

		for (int j = 0; j < width; j++)
		{
			for (int k = 0; k < height; k++)
			{
				if (matte[k * width + j]) {
					continue;
				}
				double r = 0.0, g = 0.0, b = 0.0;

				for (int h = 0; h < NUM_INPUTS; ++h) {
					const auto pixel_index = k * width + j;
					const RGBVector r_xy = reflectance_functions[pixel_index][h];

					const double r_d = toUnitInterval(r_xy(0));
					const double g_d = toUnitInterval(r_xy(1));
					const double b_d = toUnitInterval(r_xy(2));

					const Eigen::Vector3d map_vector = env_map[h];

					r += r_d * map_vector(0);
					g += g_d * map_vector(1);
					b += b_d * map_vector(2);

					//if(j == 850 && k == 548)
					//	std::cout << h << ": " << r << ", " << g << ", " << b << "\n";
				}

				if (r > 1.0)
					r = 1.0;
				if (g > 1.0)
					g = 1.0;
				if (b > 1.0)
					b = 1.0;

				//std::cout << "r: " << r << "\ng: " << g << "\nb: " << b << "\n\n";

				reluminated_image.at2d(j, k, 0) = toByteValues(r);// / static_cast<double>(NUM_INPUTS));
				reluminated_image.at2d(j, k, 1) = toByteValues(g);// / static_cast<double>(NUM_INPUTS));
				reluminated_image.at2d(j, k, 2) = toByteValues(b);// / static_cast<double>(NUM_INPUTS));

			}
			/*end = std::chrono::steady_clock::now();
			std::cout << "Image " << i << " line " << j << " of " << (width - 1) << ") took "
				<< std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "ns\n";
			begin = end;*/
		}
		//begin = std::chrono::steady_clock::now();
		//reluminated_images.push_back(reluminated_image);
		save_image(out_path, filename, reluminated_image, i);
		//end = std::chrono::steady_clock::now();
		//std::cout << "Saving took " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "ns\n";

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
	if (argc < 8) {
		std::cout << "Not enough arguments\n";
	}

	const char* path = argv[1];
	const char* filename = argv[2];
	const char* directions_file = argv[3];
	const char* environment_file = argv[4];
	const char* out_path = argv[5];
	const char* intensities_file = argv[6];
	const char* matte_file = argv[7];
	int naming_scheme = 3;
	if(argc > 8)
		naming_scheme = std::stoi(std::string(argv[8]));


	// std::cout << "FILENAME: " << filename << std::endl;
	// std::cout << "path: " << path << std::endl;
	// std::cout << "directions_file: " << directions_file << std::endl;

	//std::vector<image_b> input_images(NUM_INPUTS);
	/*if (argc > 7)
		load_images(path, filename, input_images.data(), naming_scheme);
	else
		load_images(path, filename, input_images.data());

	const int width = input_images[0].width();
	const int height = input_images[0].height();
	std::vector<std::vector<RGBVector>> reflectance_functions(width * height, std::vector<RGBVector>(253));
	calculateR_xy(input_images.data(), reflectance_functions.data());*/

	std::vector<Eigen::Vector3d> directions(NUM_INPUTS);
	load_directions(directions_file, directions.data());

	//image_b output = convertToPanorama(image_io::load(environment_file), directions.data());
	//image_io::save(output, "C:/Users/migon/Documents/Capturing Reality/AAA.png");

	std::vector<Eigen::Vector3d> intensities(NUM_INPUTS);
	load_intensities(intensities_file, intensities.data());

	const std::pair<int, int> dimensions = get_dimensions(path, filename, naming_scheme);
	const int width = dimensions.first, height = dimensions.second;
	const int size = width * height;

	std::vector<unsigned char> matte(size);
	load_matte(matte_file, matte.data());

	std::vector<std::vector<RGBVector>> reflectance_functions(size, std::vector<RGBVector>(NUM_INPUTS));

	load_reflectance_functions(path, filename, intensities.data(), matte.data(), reflectance_functions.data(), naming_scheme);

	//normalize_colors(input_images.data(), intensities.data());

	std::vector<double> solid_angles(NUM_INPUTS);
	calculateSolidAngles(directions.data(), solid_angles.data());

	const image_b environment_image = image_io::load(environment_file);

	// std::stringstream stream;
	// stream << path << "/out/" << filename << "_" << std::setw(3) << std::setfill('0') << 0 << "_out.png";
	// std::string out_file = stream.str();

	

	//std::vector<image_b> reluminated_images = 
	//reluminate_images(input_images, environment_image, directions, out_path, filename, solid_angles);
	reluminate_images_alt(reflectance_functions, environment_image, directions, matte.data(), out_path, filename, width, height, solid_angles);
	


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

// UNUSED
/*
void load_images(const char* path, const char* filename, image_b* images, const int naming_scheme = 3) {

	for (int i = 0; i < NUM_INPUTS; ++i) {
		std::stringstream stream;
		stream << path << "/" << filename << "_" << std::setw(naming_scheme) << std::setfill('0') << i << ".png";
		std::string file = stream.str();

		// std::cout << "FILENAME: " << file << std::endl;
		// std::cout << "path: " << path << std::endl;

		images[i] = image_io::load(file.c_str());
		//std::cout << "Loaded image " << (i + 1) << " of " << NUM_INPUTS << "\n";
	}
	std::cout << "Loaded images\n";

}

void calculateR_xy(const image_b* images, std::vector<RGBVector>* r_xy) {
	int width = images[0].width();
	int height = images[0].height();

	for (int i = 0; i < width; ++i) {
		for (int j = 0; j < height; ++j) {
			for (int k = 0; k < NUM_INPUTS; ++k) {

				const unsigned char r = (&images[k])->at2d(i, j, 0);
				const unsigned char g = (&images[k])->at2d(i, j, 1);
				const unsigned char b = (&images[k])->at2d(i, j, 2);

				r_xy[j * width + i][k] = { r, g, b };
			}
		}
	}
}
*/

/*
void reluminate_images(const std::vector<image_b> &input_images, const image_b env_img, const std::vector<Eigen::Vector3d> directions, const char* out_path, const char* filename, std::vector<double> solid_angles)
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

					const Eigen::Vector3d map_vector = env_map[i];

					r += r_d * map_vector(0);
					g += g_d * map_vector(1);
					b += b_d * map_vector(2);

				}

				if (r > 0.f)
					printf("(%i, %i) red value is %f\n", j, k, r);
				if (g > 0.f)
					printf("(%i, %i) green value is %f\n", j, k, g);
				if (b > 0.f)
					printf("(%i, %i) blue value is %f\n", j, k, b);

				reluminated_image.at2d(j, k, 0) = toByteValues(r / static_cast<double>(NUM_INPUTS));
				reluminated_image.at2d(j, k, 1) = toByteValues(g / static_cast<double>(NUM_INPUTS));
				reluminated_image.at2d(j, k, 2) = toByteValues(b / static_cast<double>(NUM_INPUTS));

			}
			end = std::chrono::steady_clock::now();
			std::cout << "Image " << i << " line " << j << " of " << (width - 1) << ") took "
				<< std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "ns\n";
			begin = end;
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
*/

/*
void load_reflectance_functions(const char* path, const char* filename, std::vector<RGBVector>* r_xy, const int naming_scheme = 3) {
	int width = 0, height = 0;
	for (int i = 0; i < NUM_INPUTS; ++i) {
		std::stringstream stream;
		stream << path << "/" << filename << "_" << std::setw(naming_scheme) << std::setfill('0') << i << ".png";
		std::string file = stream.str();

		// std::cout << "FILENAME: " << file << std::endl;
		// std::cout << "path: " << path << std::endl;

		const image_b image = image_io::load(file.c_str());
		if (!width) {
			width = image.width();
			height = image.height();
		}

		for (int j = 0; j < width; ++j) {
			for (int k = 0; k < height; ++k) {

				const unsigned char r = image.at2d(j, k, 0);
				const unsigned char g = image.at2d(j, k, 1);
				const unsigned char b = image.at2d(j, k, 2);

				r_xy[k * width + j][i] = { r, g, b };
				//std::cout << r:xy[k * width + k]
			}
		}
		//std::cout << "Loaded function " << (i + 1) << " of " << NUM_INPUTS << "\n";
	}
	std::cout << "Loaded reflectance functions\n";
}
*/

/*
void normalize_colors(image_b* images, const Eigen::Vector3d* intensities) {
	int width = 0, height = 0;
	for (int i = 0; i < NUM_INPUTS; ++i) {
		image_b* image = &images[i];
		if (!width) {
			width = image->width();
			height = image->height();
		}
		const Eigen::Vector3d* intensity = &intensities[i];
		const double norm = intensity->norm();
		const double normalizer = (1.0 / norm);
		for (int j = 0; j < width; ++j) {
			for (int k = 0; k < height; ++k) {
				image->at2d(j, k, 0) = static_cast<unsigned char>(static_cast<double>(image->at2d(j, k, 0)) * normalizer);
				image->at2d(j, k, 1) = static_cast<unsigned char>(static_cast<double>(image->at2d(j, k, 1)) * normalizer);
				image->at2d(j, k, 2) = static_cast<unsigned char>(static_cast<double>(image->at2d(j, k, 2)) * normalizer);
			}
		}
	}
	std::cout << "Normalized colors\n";
}
*/

/*
void determineEnvironmentMap_old(const image_b image, const Eigen::Vector3d* directions, const double* solid_angles, Eigen::Vector3d* environment_map, const Eigen::Matrix3d displacement = Eigen::Matrix3d::Identity()) {
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
		if (normal_y != 0.0) {
			normal_x = -a / (2 * normal_y);
			normal_z = -c / (2 * normal_y);
		}
		// set x and z arbitrarily at the very edge of the sphere on the input image
		else {
			normal_x = 1.0;
			normal_z = 0.0;
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
		//std::cout << environment_map[i] << "\n";
		//environment_map[i] = {c_red, c_green, c_blue};
	}
}
*/