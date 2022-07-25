#include "image.h"

#include "ply.h"

#include <stdexcept>
#include <cstdlib>
#include <iostream>
#include <ios>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <omp.h>

#define SIGMA 0.1

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

inline Eigen::Vector3d to_world_coordinates(
	const Eigen::Vector2d& x_pixel, 
	const camera_info& info, 
	const double depth, 
	const Eigen::Vector2d& dimensions
) {
	// subtract principal point (multiplied with dimensions so it's in the center of the image)
	Eigen::Vector2d x_dist = x_pixel - info.principal_point.cwiseProduct(dimensions);

	// divide result by focal length
	x_dist = x_dist / info.focal_length;

	// mirror point
	x_dist = x_dist.cwiseProduct(Eigen::Vector2d(-1.0, 1.0));

	// construct point on image plane that's dimensions(0) far away from camera origin
	// dimensions(0) was chosen because the results looked best with it
	Eigen::Vector3d x_dist_3d(x_dist(0), x_dist(1), dimensions(0));

	// normalize so we can get a vector with proper length
	x_dist_3d.normalize();

	// get vector that has length of depth
	const Eigen::Vector3d x_camera = x_dist_3d * depth;
	
	// invert camera transformation
	return info.rotation.transpose() * (x_camera - info.translation);
}

inline Eigen::Vector3d get_normal(const Eigen::Vector3d& x_1, const Eigen::Vector3d& x_2, const Eigen::Vector3d& y_1, const Eigen::Vector3d& y_2) {
	const Eigen::Vector3d x_diff = x_2 - x_1;
	const Eigen::Vector3d y_diff = y_2 - y_1;
	return x_diff.cross(y_diff).normalized();
}

std::vector<Eigen::Vector3d> create_3d_points(
	image_f const * images, 
	camera_info const * infos, 
	const unsigned int num_images, 
	const unsigned int num_points, 
	std::vector<Eigen::Vector3d>& normals, 
	std::vector<double>& scales,
	const bool scale,
	const bool no_normals
) {
	std::vector<Eigen::Vector3d> points;
	points.reserve(num_points);
	if(!no_normals)
		normals.reserve(num_points);
	if(scale)
		scales.reserve(num_points);
	for (int i = 0; i < num_images; ++i) {
		const image_f image = images[i];
		const camera_info info = infos[i];
		const Eigen::Vector2d dimensions(image.width(), image.height());

		for (int j = 0; j < image.width(); ++j) {
			for (int h = 0; h < image.height(); ++h) {
				const double depth = image.at2d(j, h);
				if (depth == 0.0)
					continue;
				const Eigen::Vector2d x_pixel(j, h);
				const Eigen::Vector3d x_point = to_world_coordinates(x_pixel, info, depth, dimensions);
				if (!(x_point(0) > -2.0 && x_point(0) < 2.0
					&& x_point(1) > -2.0 && x_point(1) < 4.0
					&& x_point(2) > -6.0 && x_point(2) < 10.0))
					continue;
				points.push_back(x_point);

				if (scale) {
					// calculate footprint
					const Eigen::Vector2d x_0_pixel(j, h);
					const Eigen::Vector2d x_1_pixel(j + 1, h);
					const Eigen::Vector3d x_0 = to_world_coordinates(x_0_pixel, info, depth, dimensions);
					const Eigen::Vector3d x_1 = to_world_coordinates(x_1_pixel, info, depth, dimensions);
					const double footprint = 4.0 * (x_0 - x_1).norm();
					scales.push_back(footprint);
				}
			}
		}

		std::cout << "Done with points for image " << i << "\n";
		if (!no_normals) {
			for (int j = 0; j < image.width(); ++j) {
				for (int h = 0; h < image.height(); ++h) {
					const double depth = image.at2d(j, h);
					if (depth == 0.0)
						continue;
					const Eigen::Vector2d x_test(j, h);
					const Eigen::Vector3d x_point = to_world_coordinates(x_test, info, depth, dimensions);
					if (!(x_point(0) > -2.0 && x_point(0) < 2.0
						&& x_point(1) > -2.0 && x_point(1) < 4.0
						&& x_point(2) > -6.0 && x_point(2) < 10.0))
						continue;
					Eigen::Vector3d x1, x2, y1, y2;
					if (j > 0) {
						const Eigen::Vector2d x_pixel(j - 1, h);
						x1 = to_world_coordinates(x_pixel, info, image.at2d(j - 1, h), dimensions);
					}
					else {
						x1 = points[j + h * image.width()];
					}
					if (j < image.width() - 1) {
						const Eigen::Vector2d x_pixel(j + 1, h);
						x2 = to_world_coordinates(x_pixel, info, image.at2d(j + 1, h), dimensions);
					}
					else {
						x2 = points[j + h * image.width()];
					}

					if (h > 0) {
						const Eigen::Vector2d x_pixel(j, h - 1);
						y1 = to_world_coordinates(x_pixel, info, image.at2d(j, h - 1), dimensions);
					}
					else {
						y1 = points[j + h * image.width()];
					}
					if (h < image.height() - 1) {
						const Eigen::Vector2d x_pixel(j, h + 1);
						y2 = to_world_coordinates(x_pixel, info, image.at2d(j, h + 1), dimensions);
					}
					else {
						y2 = points[j + h * image.width()];
					}

					normals.push_back(-get_normal(x1, x2, y1, y2));
				}
			}
		}

		std::cout << "Done with image " << i << "\n";
	}

	return points;
}

inline Eigen::Vector3d get_normal_unstructured(const std::vector<Eigen::Vector3d>& Nbhd) {
	Eigen::Vector3d o_i = Eigen::Vector3d::Zero();
	for (const auto& y : Nbhd)
		o_i += y;
	o_i /= Nbhd.size();
	Eigen::Matrix3d CV = Eigen::Matrix3d::Zero();
	for (const auto& y : Nbhd) {
		const Eigen::Vector3d temp = y - o_i;
		const Eigen::Matrix3d outer_product = temp * temp.transpose();
		CV += outer_product;
	}
	Eigen::EigenSolver<Eigen::Matrix3d> solver(CV);
	const Eigen::Matrix3d evs = solver.eigenvectors().real();
	const Eigen::Vector3d ev_3(evs.col(2));

	return -ev_3;
}

inline void move_up(std::vector<Eigen::Vector3d>& closest, std::vector<double>& distances, int start, int k) {
#pragma unroll
	for (int i = k - 2; i >= start; --i) {
			closest[i+1] = closest[i];
			distances[i+1] = distances[i];
	}
}

inline void get_normals_unstructured(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& normals) {
	const int k = 5;
	normals.resize(points.size());
#pragma omp parallel for num_threads(16)
	for (int i = 0; i < points.size(); ++i) {
		// find k closest points
		std::vector<Eigen::Vector3d> closest(k);
		std::vector<double> distances(k);
		const Eigen::Vector3d point = points[i];
		int num = 0;
		for (const auto& p_i : points) {
#pragma unroll
			const double distance = (p_i - point).norm();
			for (int j = 0; j < k; ++j) {
				if (num - 1 < j) {
					closest[j] = p_i;
					++num;
					break;
				}
				if (distance < distances[j]) {
					move_up(closest, distances, j, k);
					break;
				}
			}
		}
		normals[i] = get_normal_unstructured(closest);
	}
	std::cout << "Done with normals\n";
}

inline Eigen::Matrix3d get_rotation_matrix(const Eigen::Vector3d& normal) {
	const Eigen::Vector3d x_unit = Eigen::Vector3d::UnitX();
	const Eigen::Vector3d v = normal.cross(x_unit);
	if (v.isZero())
		return Eigen::Matrix3d::Identity();
	const double sine = v.norm();
	const double cosine = normal.dot(x_unit);
	if (fabs(cosine) < 0.0001)
		return -Eigen::Matrix3d::Identity();
	Eigen::Matrix3d cross_matrix;
	cross_matrix << 
		0.0, -v(2), v(1),
		v(2), 0.0, -v(0),
		-v(1), v(0), 0.0;
	const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity() + cross_matrix + cross_matrix * cross_matrix * ((1.0 - cosine) / (sine * sine));

	return rotation;
}

inline std::vector<Eigen::Matrix3d> get_rotation_matrices(const std::vector<Eigen::Vector3d>& normals) {
	std::vector<Eigen::Matrix3d> rotation_matrices;
	rotation_matrices.reserve(normals.size());
	for (const auto& normal : normals) {
		rotation_matrices.push_back(get_rotation_matrix(normal));
	}

	return rotation_matrices;
}

inline double get_weight(const Eigen::Vector3d& X) {
	const double x = X(0);
	double w_x = 0.0;
	if (x >= -3 * SIGMA && x < 0.0) {
		w_x = (1.0 / 9.0) * ((x * x) / (SIGMA * SIGMA)) + (2.0 / 3.0) * (x / SIGMA) + 1.0;
	}
	else if (x < 3 * SIGMA) {
		w_x = (2.0 / 27.0) * ((x * x * x) / (SIGMA * SIGMA * SIGMA)) - (1.0 / 3.0) * ((x * x) / (SIGMA * SIGMA)) + 1.0;
	}
	else {
		return 0.0;
	}
	double w_yz = 0.0;
	const double r = sqrt(X(1)*X(1) + X(2)*X(2));
	if (r < 3 * SIGMA) {
		w_yz = (2.0 / 27.0) * ((r * r * r) / (SIGMA * SIGMA * SIGMA)) - (1.0 / 3.0) * ((r * r) / (SIGMA * SIGMA)) + 1.0;
	}
	else {
		return 0.0;
	}

	return w_x * w_yz;
}

inline double get_weight_scaled(const Eigen::Vector3d& X, const double scale) {
	const double x = X(0);
	double w_x = 0.0;
	if (x >= -3 * scale && x < 0.0) {
		w_x = (1.0 / 9.0) * ((x * x) / (scale * scale)) + (2.0 / 3.0) * (x / scale) + 1.0;
	}
	else if (x < 3 * scale) {
		w_x = (2.0 / 27.0) * ((x * x * x) / (scale * scale * scale)) - (1.0 / 3.0) * ((x * x) / (scale * scale)) + 1.0;
	}
	else {
		return 0.0;
	}
	double w_yz = 0.0;
	const double r = sqrt(X(1) * X(1) + X(2) * X(2));
	if (r < 3 * scale) {
		w_yz = (2.0 / 27.0) * ((r * r * r) / (scale * scale * scale)) - (1.0 / 3.0) * ((r * r) / (scale * scale)) + 1.0;
	}
	else {
		return 0.0;
	}

	return w_x * w_yz;
}

inline Eigen::Vector3d rotate_to_lcs(const Eigen::Vector3d& x, const Eigen::Matrix3d& rot, const Eigen::Vector3d& pos) {
	return rot * (x - pos);
}

inline double get_W(const Eigen::Vector3d& x, const std::vector<Eigen::Matrix3d>& rotations, const std::vector<Eigen::Vector3d>& points) {
	double W = 0.0;
//#pragma omp parallel for num_threads(64)
	for (int i = 0; i < points.size(); ++i) {
		W += get_weight(rotate_to_lcs(x, rotations[i], points[i]));
	}

	return W;
}

inline double get_W_scaled(const Eigen::Vector3d& x, const std::vector<Eigen::Matrix3d>& rotations, const std::vector<Eigen::Vector3d>& points, const std::vector<double>& scales) {
	double W = 0.0;
	//#pragma omp parallel for num_threads(64)
	for (int i = 0; i < points.size(); ++i) {
		W += get_weight_scaled(rotate_to_lcs(x, rotations[i], points[i]), scales[i]);
	}

	return W;
}

inline double basis_f(const Eigen::Vector3d& x) {
	const double first = x(0) / (std::pow(SIGMA, 4) * 2 * EIGEN_PI);
	const double second = std::exp(-(1.0 / (2 * SIGMA * SIGMA)) * x.squaredNorm());

	return first * second;
}

inline double basis_f_scaled(const Eigen::Vector3d& x, const double scale) {
	const double first = x(0) / (std::pow(scale, 4) * 2 * EIGEN_PI);
	const double second = std::exp(-(1.0 / (2 * scale * scale)) * x.squaredNorm());

	return first * second;
}

inline double implicit_F(const Eigen::Vector3d& x, const std::vector<Eigen::Matrix3d>& rotations, const std::vector<Eigen::Vector3d>& points) {
	const double W = get_W(x, rotations, points);
	if (W == 0.0)
		return 10000.0;
	double result = 0.0;
//#pragma omp parallel for num_threads(64)
	for (int i = 0; i < rotations.size(); ++i) {
		const Eigen::Vector3d x_i = rotate_to_lcs(x, rotations[i], points[i]);
		const double weight = get_weight(x_i);
		if (weight > 0.0) {
			result += weight * basis_f(x_i);
		}
	}
	return result / W;
}

inline double implicit_F_scaled(const Eigen::Vector3d& x, const std::vector<Eigen::Matrix3d>& rotations, const std::vector<Eigen::Vector3d>& points, const std::vector<double>& scales) {
	const double W = get_W_scaled(x, rotations, points, scales);
	if (W == 0.0)
		return 10000.0;
	double result = 0.0;
	//#pragma omp parallel for num_threads(64)
	for (int i = 0; i < rotations.size(); ++i) {
		const Eigen::Vector3d x_i = rotate_to_lcs(x, rotations[i], points[i]);
		const double weight = get_weight_scaled(x_i, scales[i]);
		if (weight > 0.0) {
			result += weight * basis_f_scaled(x_i, scales[i]);
		}
	}
	return result / W;
}

void write_point_cloud(const char* path, const std::vector<Eigen::Vector3d> vertices, char* filename = nullptr);
void write_point_cloud_normals(const char* path, const std::vector<Eigen::Vector3d> vertices, const std::vector<Eigen::Vector3d> normals, char* filename = nullptr);

std::vector<double> iterate_voxels(
	const std::vector<Eigen::Vector3d> points, 
	const std::vector<Eigen::Matrix3d> rotations, 
	const std::vector<double> scales,
	const double min_x, const double min_y, const double min_z,
	const double max_x, const double max_y, const double max_z,
	const int divisor,
	const bool scale) 
{

	const double step = 1.0 / static_cast<double>(divisor);

	const double start_x = std::floor(min_x) + step / 2.0;
	const double start_y = std::floor(min_y) + step / 2.0;
	const double start_z = std::floor(min_z) + step / 2.0;

	const int start_i = std::floor(min_x);
	const int start_j = std::floor(min_y);
	const int start_k = std::floor(min_z);

	const int end_i = std::ceil(max_x);
	const int end_j = std::ceil(max_y);
	const int end_k = std::ceil(max_z);

	const int range_i = (end_i - start_i) * divisor;
	const int range_j = (end_j - start_j) * divisor;
	const int range_k = (end_k - start_k) * divisor;

	std::vector<double> result(range_i * range_j * range_k);
	std::vector<Eigen::Vector3d> result_points(range_i * range_j * range_k, Eigen::Vector3d::Zero());
#pragma omp parallel for //num_threads(16)
	for (int i = 0; i < range_i; ++i) {
		if (i == 0)
			std::cout << omp_get_num_threads() << "\n";
		for (int j = 0; j < range_j; ++j) {
			for (int k = 0; k < range_k; ++k) {
				const Eigen::Vector3d x(start_x + i * step, start_y + j * step, start_z + k * step);
				double res;
				if(scale)
					res = implicit_F_scaled(x, rotations, points, scales);
				else
					res = implicit_F(x, rotations, points);
				result[i * range_j * range_k + j * range_k + k] = res;
				if (res < 0.0)
					result_points[i * range_j * range_k + j * range_k + k] = x;
			}
		}
		std::cout << i + 1 << "/" << range_i << "\n\n";
	}
	std::cout << "Grid dimensions: " << range_i << " " << range_j << " " << range_k << "\n";
	write_point_cloud("C:/Users/migon/Documents/Capturing Reality/Exercise 4/achteck-new2-part1", result_points, "point_cloud_grid");
	return result;
}

void write_point_cloud(const char* path, const std::vector<Eigen::Vector3d> vertices, char* filename) {
	std::vector<Vertex_pc> vertices_pc;
	vertices_pc.reserve(vertices.size());
	for (auto v : vertices) {
		vertices_pc.push_back(Vertex_pc{ static_cast<float>(v(0)), static_cast<float>(v(1)), static_cast<float>(v(2)) });
	}
	if (!filename)
		filename = "point_cloud";
	std::ofstream pc_output(std::string(path) + "\\" + filename + ".ply", std::ios_base::binary);
	write_ply_point_cloud(pc_output, vertices_pc);
	std::cout << "Wrote point cloud\n";
}

void write_point_cloud_normals(const char* path, const std::vector<Eigen::Vector3d> vertices, const std::vector<Eigen::Vector3d> normals, char* filename) {
	std::vector<Vertex_pc_n> vertices_pc_n;
	vertices_pc_n.reserve(vertices.size());
	for (int i = 0; i < vertices.size(); ++i) {
		vertices_pc_n.push_back(Vertex_pc_n{ static_cast<float>(vertices[i](0)), static_cast<float>(vertices[i](1)), static_cast<float>(vertices[i](2)),
			static_cast<float>(normals[i](0)), static_cast<float>(normals[i](1)), static_cast<float>(normals[i](2)) });
	}
	if (!filename)
		filename = "point_cloud";
	std::ofstream pc_output(std::string(path) + "\\" + filename + ".ply", std::ios_base::binary);
	write_ply_point_cloud_normals(pc_output, vertices_pc_n);
	std::cout << "Wrote point cloud\n";
}

void write_grid_file(const char* path, const std::vector<double>& grid) {
	std::ofstream grid_output(std::string(path) + "\\grid.txt", std::ios::binary);
	if (grid_output.is_open()) {
		for (const double& v : grid) {
			//grid_output << std::to_string(v) << "\n";
			float v_f = v;
			grid_output.write(reinterpret_cast<char*>(&v_f), sizeof(float));
		}
	}
	else {
		std::cout << "Failed to write grid\n";
	}
}

int main(int argc, const char **argv)
{
	const char* path = argv[1];
	const int num_images = std::stoi(std::string(argv[2]));
	bool scale, hoppe;
	if (argc > 3)
		scale = std::stoi(argv[3]);
	if (argc > 4)
		hoppe = std::stoi(argv[4]);
	const std::vector<camera_info> infos = load_camera_infos(path, num_images);
	const std::vector<image_f> images = load_depth_images(path, num_images);
	std::cout << "Loaded depth images\n";
	const int img_size = images[0].size();

	std::vector<Eigen::Vector3d> normals;
	std::vector<double> scales;
	const std::vector<Eigen::Vector3d> points = create_3d_points(images.data(), infos.data(), num_images, num_images * img_size, normals, scales, scale, hoppe);
	std::cout << "Created points\n";

	if (hoppe)
		get_normals_unstructured(points, normals);
	write_point_cloud_normals(path, points, normals);
	const std::vector<Eigen::Matrix3d> matrices = get_rotation_matrices(normals);
	const std::vector<double> voxel_grid = iterate_voxels(points, matrices, scales, -2.0, -4.0, 6.0, 2.0, 2.0, 10.0, 64, scale);
	write_grid_file(path, voxel_grid);

	return EXIT_SUCCESS;
}