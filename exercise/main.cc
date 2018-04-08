#include "image.h"

#include <stdexcept>
#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>

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

int main(int argc, const char **argv)
{
	if (argc != 2) throw std::runtime_error("Invalid arguments");

	example(argv[1]);

	return EXIT_SUCCESS;
}