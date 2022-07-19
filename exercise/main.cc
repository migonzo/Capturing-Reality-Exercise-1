#include "image.h"

#include "ply.h"

#include <stdexcept>
#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>

#define SIGMA 1.0

struct camera_info {
	double focal_length;
	double pixel_aspect;
	Eigen::Vector2d principal_point;
	Eigen::Matrix3d rotation;
	Eigen::Vector3d translation;
};

std::vector<camera_info> load_camera_infos(const char* path, const unsigned int num) {
	std::vector<camera_info> infos;
	infos.reserve(num);
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

		std::cout << focal << " " << aspect << "\n" << p_point << rot << trans << "\n";
	}

	return infos;
}

std::vector<image_f> load_depth_images(const char* path, const unsigned int num) {
	std::vector<image_f> depth_images;
	depth_images.reserve(num);
	for (int i = 0; i < num; ++i) {
		std::stringstream stream;
		stream << path << "/" << "view_" << std::setw(4) << std::setfill('0') << i << "-depth-L2.pfm";
		std::string file_str = stream.str();
		
		depth_images.push_back(image_io::load<float>(file_str.c_str()));
	}

	return depth_images;
}

inline Eigen::Vector3d to_world_coordinates(const Eigen::Vector2d x_pixel, const camera_info info, const double depth) {
	const Eigen::Vector2d x_dist = (x_pixel - info.principal_point) / info.focal_length;
	Eigen::Vector3d x_dist_3d(x_dist(0), x_dist(1), 1.0);
	x_dist_3d.normalize();
	//std::cout << info.focal_length << "\n";
	const Eigen::Vector3d x_camera = x_dist_3d * depth;
	// maybe wrong
	//const Eigen::Vector3d x_camera(x_dist(0) * depth, x_dist(1) * depth, depth);
	return info.rotation.transpose() * (x_camera - info.translation);
}

inline Eigen::Vector3d get_normal(const Eigen::Vector3d x_1, const Eigen::Vector3d x_2, const Eigen::Vector3d y_1, const Eigen::Vector3d y_2) {
	const Eigen::Vector3d x_diff = x_2 - x_1;
	const Eigen::Vector3d y_diff = y_2 - y_1;
	return x_diff.cross(y_diff).normalized();
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> create_3d_points(image_f const * images, camera_info const * infos, const unsigned int num_images, const unsigned int num_points) {
	std::vector<Eigen::Vector3d> points;
	points.reserve(num_points);
	std::vector<Eigen::Vector3d> normals;
	normals.reserve(num_points);
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

		std::cout << "Done with points for image " << i << "\n";

		for (int j = 0; j < image.width(); ++j) {
			for (int h = 0; h < image.height(); ++h) {
				Eigen::Vector3d x1, x2, y1, y2;
				if (j > 0) {
					x1 = points[j - 1 + h * image.width()];
				}
				else {
					x1 = points[j + h * image.width()];
				}
				if (j < image.width() - 1) {
					x2 = points[j + 1 + h * image.width()];
				}
				else {
					x2 = points[j + h * image.width()];
				}

				if (h > 0) {
					y1 = points[j + (h - 1) * image.width()];
				}
				else {
					y1 = points[j + h * image.width()];
				}
				if (h < image.height() - 1) {
					y2 = points[j + (h + 1) * image.width()];
				}
				else {
					y2 = points[j + h * image.width()];
				}

				get_normal(x1, x2, y1, y2);
			}
		}

		std::cout << "Done with image " << i << "\n";
	}

	return std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>(points, normals);
}

inline Eigen::Matrix3d get_rotation_matrix(const Eigen::Vector3d normal) {
	const Eigen::Vector3d x_unit = Eigen::Vector3d::UnitX();
	const Eigen::Vector3d v = normal.cross(x_unit);
	const double sine = v.norm();
	const double cosine = normal.dot(x_unit);
	Eigen::Matrix3d cross_matrix;
	cross_matrix << 
		0.0, -v(2), v(1),
		v(2), 0.0, -v(0),
		-v(1), v(0), 0.0;
	const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity() + cross_matrix + cross_matrix * cross_matrix * ((1.0 - cosine) / (sine * sine));

	return rotation;
}

inline std::vector<Eigen::Matrix3d> get_rotation_matrices(const std::vector<Eigen::Vector3d> normals) {
	std::vector<Eigen::Matrix3d> rotation_matrices;
	rotation_matrices.reserve(normals.size());
	for (const auto& normal : normals) {
		rotation_matrices.push_back(get_rotation_matrix(normal));
	}

	return rotation_matrices;
}

inline Eigen::Vector3d rotate_to_lcs(const Eigen::Vector3d x, const Eigen::Matrix3d rot, const Eigen::Vector3d pos) {
	return rot * (x - pos);
}

inline double basis_f(const Eigen::Vector3d x) {
	const double first = x(0) / (std::pow(SIGMA, 4) * 2 * EIGEN_PI);
	const double second = std::exp(-(1.0 / (2 * SIGMA * SIGMA)) * x.squaredNorm());

	return first * second;
}

inline double implicit_F(const Eigen::Vector3d x, const std::vector<Eigen::Matrix3d>& rotations, const std::vector<Eigen::Vector3d>& points) {
	double result = 0.0;
	for (int i = 0; i < rotations.size(); ++i) {
		const Eigen::Vector3d x_i = rotate_to_lcs(x, rotations[i], points[i]);
		result += basis_f(x_i);
	}
}

std::vector<double> iterate_voxels(
	const std::vector<Eigen::Vector3d> points, 
	const std::vector<Eigen::Matrix3d> rotations, 
	const double min_x, const double min_y, const double min_z,
	const double max_x, const double max_y, const double max_z,
	const int dim_x, const int dim_y, const int dim_z) 
{
	std::vector<double> result;
	result.reserve(dim_x * dim_y * dim_z);

	const double step_x = (max_x - min_x) / static_cast<double>(dim_x);
	const double step_y = (max_y - min_y) / static_cast<double>(dim_y);
	const double step_z = (max_z - min_z) / static_cast<double>(dim_z);

	const double start_x = min_x + step_x / 2.0;
	const double start_y = min_y + step_y / 2.0;
	const double start_z = min_z + step_z / 2.0;

	for (int i = 0; i < dim_x; ++i) {
		for (int j = 0; j < dim_y; ++j) {
			for (int k = 0; k < dim_z; ++k) {
				const Eigen::Vector3d x(start_x + i * step_x, start_y + j * step_y, start_z + k * step_z);
				result.push_back(implicit_F(x, rotations, points));
			}
		}
	}

	return result;
}

int main(int argc, const char **argv)
{
	const char* path = argv[1];
	const std::vector<camera_info> infos = load_camera_infos(path, 1);
	const std::vector<image_f> images = load_depth_images(path, 1);
	std::cout << "Loaded depth images\n";
	const int img_size = images[0].size();

	const std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> result = create_3d_points(images.data(), infos.data(), 1, 1 * img_size);
	std::cout << "Created points and normals\n";
	const std::vector<Eigen::Vector3d> points = result.first;
	double min_x, min_y, min_z;
	min_x = min_y = min_z = 1000000000.0;
	double max_x, max_y, max_z;
	max_x = max_y = max_z = -1000000000.0;
	for (int i = 0; i < points.size(); ++i) {
		const Eigen::Vector3d test = points[i];
		min_x = test(0) < min_x ? test(0) : min_x;
		min_y = test(1) < min_y ? test(1) : min_y;
		min_z = test(2) < min_z ? test(2) : min_z;
		max_x = test(0) > max_x ? test(0) : max_x;
		max_y = test(1) > max_y ? test(1) : max_y;
		max_z = test(2) > max_z ? test(2) : max_z;
	}
	std::cout << min_x << " " << min_y << " " << min_z << "\n" << max_x << " " << max_y << " " << max_z << "\n";

	const std::vector<Eigen::Vector3d> normals = result.second;

	const std::vector<Eigen::Matrix3d> matrices = get_rotation_matrices(normals);
	const std::vector<double> voxel_grid = iterate_voxels(points, matrices, min_x, min_y, min_z, max_x, max_y, max_z, 100, 100, 100);

	return EXIT_SUCCESS;
}