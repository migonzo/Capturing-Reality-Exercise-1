#include "image.h"

#include "ply.h"

#include <stdexcept>
#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>

struct camera_info {
	double focal_length;
	double pixel_aspect;
	Eigen::Vector2d principal_point;
	Eigen::Matrix3d rotation;
	Eigen::Vector3d translation;
};

std::vector<camera_info> load_camera_infos(const char* path, const unsigned int num) {
	std::vector<camera_info> infos(num);
	for (int i = 0; i < num; ++i) {
		std::stringstream stream;
		stream << path << "/" << "view_" << std::setw(4) << std::setfill('0') << i << "-cam.txt";
		std::string file_str = stream.str();
		std::ifstream data;
		data.open(file_str);
		if (!data.is_open()) {
			std::cout << "Could not open " << file_str << "\n";
			throw std::runtime_error("Could not open camera info file");
		}

		float focal, aspect, p_point_0, p_point_1, rot_00, rot_01, rot_02, rot_10, rot_11, rot_12, rot_20, rot_21, rot_22, trans_0, trans_1, trans_2;
		if (!(data >> focal >> aspect
			>> p_point_0 >> p_point_1
			>> rot_00 >> rot_01 >> rot_02
			>> rot_10 >> rot_11 >> rot_12
			>> rot_20 >> rot_21 >> rot_22
			>> trans_0 >> trans_1 >> trans_2
			)) 
		{
			std::cout << "Invalid camera info file\n";
			throw std::runtime_error("Invalid camera info file");
		}

		Eigen::Vector2d p_point(p_point_0, p_point_1);
		Eigen::Matrix3d rot;
		rot << rot_00, rot_01, rot_02, rot_10, rot_11, rot_12, rot_20, rot_21, rot_22;
		Eigen::Vector3d trans(trans_0, trans_1, trans_2);

		camera_info info{ focal, aspect, p_point, rot, trans };

		infos.push_back(info);
	}

	return infos;
}

std::vector<image_f> load_depth_images(const char* path, const unsigned int num) {
	std::vector<image_f> depth_images(num);
	for (int i = 0; i < num; ++i) {
		std::stringstream stream;
		stream << path << "/" << "view_" << std::setw(4) << std::setfill('0') << i << "-L2.pfm";
		std::string file_str = stream.str();
		
		depth_images.push_back(image_io::load<float>(file_str.c_str()));
	}

	return depth_images;
}

inline Eigen::Vector3d to_world_coordinates(const Eigen::Vector2d x_pixel, const camera_info info, const double depth) {
	const Eigen::Vector2d x_dist = (x_pixel - info.principal_point) / info.focal_length;
	const Eigen::Vector3d x_camera = x_dist.normalized() * depth;
	return info.rotation.transpose() * (x_camera - info.translation);
}

std::vector<Eigen::Vector3d> create_3d_points(image_f const * images, camera_info const * infos, const unsigned int num_images, const unsigned int num_points) {
	std::vector<Eigen::Vector3d> points(num_points);
	for (int i = 0; i < num_images; ++i) {
		const image_f image = images[i];
		const camera_info info = infos[i];

		for (int j = 0; j < image.width(); ++j) {
			for (int h = 0; h < image.height(); ++h) {
				const double depth = image.at2d(j, h);
				const Eigen::Vector2d x_pixel(j, h);
				points.push_back(to_world_coordinates(x_pixel, info, depth));
			}
		}
	}

	return points;
}

inline Eigen::Vector3d rotate_to_lcs(const Eigen::Vector3d x, const Eigen::Matrix3d rot, const Eigen::Vector3d pos) {
	return rot * (x - pos);
}

inline double basis_f(const Eigen::Vector3d x) {

}

inline double implicit_F(const std::vector<Eigen::Vector3d>& xs) {

}

int main(int argc, const char **argv)
{
	const char* path = argv[1];
	const std::vector<camera_info> infos = load_camera_infos(path);

	return EXIT_SUCCESS;
}