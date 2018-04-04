#include "image.h"

#include <iostream>
#include <Eigen/Dense>

int main()
{
	image<unsigned char> img = image_io::load_jpeg("in.jpg");
	float r, g, b;
	Eigen::Vector2d v(669.5, 689.217);
	r = img.lin_at(v, 0);
	g = img.lin_at(v, 1);
	b = img.lin_at(v, 2);
	std::cout << r << " " << g << " " << b << std::endl;
	image_io::save_jpeg(img, "./test.jpg");

	image<float> cp(img.width(), img.height(), 1);
	for (int i = 0; i < img.pixels(); ++i) {
		cp[i] = img.at(i) / 255.f;
	}
	image_io::save_exr(cp, "./saveexr.exr");
	image_io::save_bpm<float>(cp, "./saveexr.pfm");

	image<float> ex = image_io::load_exr("test.exr");
	image_io::save_exr(ex, "./tes2t.exr");
	image<unsigned char> cp2(ex.width(), ex.height(), 1);
	for (int i = 0; i < ex.pixels(); ++i) {
		cp2[i] = std::min(std::max(ex[i] * 255.f, 0.f), 255.f);
	}
	image_io::save_jpeg(cp2, "./exr.jpg");
	image_io::save_bpm(cp2, "./exr.pgm");


	image<unsigned char> p1 = image_io::load_bpm("p1.pbm");
	image<unsigned char> p2 = image_io::load_bpm("p2.pgm");
	image<unsigned char> p3 = image_io::load_bpm("p3.ppm");
	image<unsigned char> p4 = image_io::load_bpm("p4.pbm");
	image<unsigned char> p5 = image_io::load_bpm("p5.pgm");
	image<unsigned char> p6 = image_io::load_bpm("p6.ppm");
	image<float> pf = image_io::load_bpm<float>("saveexr.pfm");

	image_io::save_bpm(p1, "dup_p1.pgm");
	image_io::save_bpm(p2, "dup_p2.pgm");
	image_io::save_bpm(p3, "dup_p3.ppm");
	image_io::save_bpm(p4, "dup_p4.pgm");
	image_io::save_bpm(p5, "dup_p5.pgm");
	image_io::save_bpm(p6, "dup_p6.ppm");
	image_io::save(pf, "dup_pf.exr");

// 	image_io::load("");
}