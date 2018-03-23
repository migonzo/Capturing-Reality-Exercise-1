#include "image.h"

#include <stdlib.h>
#include <stdio.h>
#include <png.h>
#include <jpeglib.h>
#include <setjmp.h>
#include <stdexcept>

namespace image_io {

image<unsigned char> load_png(const char *filename)
{
	FILE *fp = fopen(filename, "rb");
	if (!fp) throw std::runtime_error("Cannot open PNG file");

	png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if (!png) {
		fclose(fp);
		throw std::runtime_error("Could not read PNG version");
	}

	png_infop info = png_create_info_struct(png);
	if (!info) {
		png_destroy_read_struct(&png, &info, NULL);
		fclose(fp);
		throw std::runtime_error("Could not read PNG info");
	}

	if (setjmp(png_jmpbuf(png))) {
		png_destroy_read_struct(&png, &info, NULL);
		fclose(fp);
		throw std::runtime_error("Internal libpng error");
	}

	png_init_io(png, fp);
	png_read_info(png, info);

	image<unsigned char> image(png_get_image_width(png, info), png_get_image_height(png, info), png_get_channels(png, info));

	png_byte color_type = png_get_color_type(png, info);
	png_byte bit_depth = png_get_bit_depth(png, info);
	if (bit_depth == 16) png_set_strip_16(png);
	if (color_type == PNG_COLOR_TYPE_PALETTE) png_set_palette_to_rgb(png);
	if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8) png_set_expand_gray_1_2_4_to_8(png);
	if (png_get_valid(png, info, PNG_INFO_tRNS)) png_set_tRNS_to_alpha(png);

	png_read_update_info(png, info);

	std::vector<png_bytep> row_pointers(image.height());
	for (std::size_t y = 0; y < image.height(); ++y) {
		row_pointers[y] = (png_byte*)image.data(0, y);
	}
	png_read_image(png, row_pointers.data());

	png_destroy_read_struct(&png, &info, NULL);
	fclose(fp);

	return image;
}

void save_png(const image<unsigned char> &image, const char *filename)
{
	FILE *fp = fopen(filename, "wb");
	if (!fp) throw std::runtime_error("Cannot open PNG file");

	png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if (!png) {
		fclose(fp);
		throw std::runtime_error("Cannot create PNG write struct");
	}

	png_infop info = png_create_info_struct(png);
	if (!info) {
		png_destroy_write_struct(&png, &info);
		fclose(fp);
		throw std::runtime_error("Cannot create PNG info");
	}

	if (setjmp(png_jmpbuf(png))) {
		png_destroy_write_struct(&png, &info);
		fclose(fp);
		throw std::runtime_error("Internal libpng error");
	}

	png_init_io(png, fp);

	png_byte color_type;
	switch (image.channels()) {
		case 1: color_type = PNG_COLOR_TYPE_GRAY; break;
		case 2: color_type = PNG_COLOR_TYPE_GRAY_ALPHA; break;
		case 3: color_type = PNG_COLOR_TYPE_RGB; break;
		case 4: color_type = PNG_COLOR_TYPE_RGB_ALPHA; break;
		default: throw std::runtime_error("Cannot determine image color type");
	}
	png_set_IHDR(png, info, image.width(), image.height(), 8, color_type, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
	png_write_info(png, info);

	std::vector<png_bytep> row_pointers(image.height());
	for (std::size_t y = 0; y < image.height(); ++y) {
		row_pointers[y] = (png_byte*)image.data(0, y);
	}

	png_write_image(png, row_pointers.data());
	png_write_end(png, NULL);

	png_destroy_write_struct(&png, &info);
	fclose(fp);
}

void jpg_error_handler(j_common_ptr)
{
	throw std::runtime_error("JPEG format not recognized");
}
void jpg_message_handler(j_common_ptr, int msg_level)
{
	if (msg_level < 0) throw std::runtime_error("JPEG data corrupt");
}

image<unsigned char> load_jpeg(const char *filename)
{
	FILE *fp = fopen(filename, "rb");
	if (!fp) throw std::runtime_error("Cannot open JPEG file");

	jpeg_decompress_struct cinfo;
	jpeg_error_mgr jerr;
	cinfo.err = jpeg_std_error(&jerr);
	jerr.error_exit = &jpg_error_handler;
	jerr.emit_message = &jpg_message_handler;

	jpeg_create_decompress(&cinfo);
	jpeg_stdio_src(&cinfo, fp);
	jpeg_read_header(&cinfo, TRUE);

	image<unsigned char> image(cinfo.image_width, cinfo.image_height, cinfo.out_color_space == JCS_RGB ? 3 : 1);

	jpeg_start_decompress(&cinfo);

	unsigned char *data = image.data();
	while (cinfo.output_scanline < cinfo.output_height) {
		jpeg_read_scanlines(&cinfo, &data, 1);
		data += image.channels() * cinfo.output_width;
	}

	jpeg_finish_decompress(&cinfo);
	jpeg_destroy_decompress(&cinfo);
	fclose(fp);

	return image;
}

void save_jpeg(const image<unsigned char> &image, const char *filename, int quality)
{
	if (image.channels() != 1 && image.channels() != 3) throw std::runtime_error("Invalid number of channels");

	FILE *fp = std::fopen(filename, "wb");
	if (!fp) throw std::runtime_error("Cannot open JPEG file");

	jpeg_compress_struct cinfo;
	jpeg_error_mgr jerr;

	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);
	jpeg_stdio_dest(&cinfo, fp);

	cinfo.image_width = image.width();
	cinfo.image_height = image.height();
	cinfo.input_components = image.channels();
	cinfo.in_color_space = image.channels() == 1 ? JCS_GRAYSCALE : JCS_RGB;

	jpeg_set_defaults(&cinfo);
	jpeg_set_quality(&cinfo, quality, TRUE);
	jpeg_start_compress(&cinfo, TRUE);

	while (cinfo.next_scanline < cinfo.image_height) {
		JSAMPROW row_pointer = (JSAMPROW)image.data(0, cinfo.next_scanline);
		jpeg_write_scanlines(&cinfo, &row_pointer, 1);
	}

	jpeg_finish_compress(&cinfo);
	jpeg_destroy_compress(&cinfo);
	fclose(fp);
}

}