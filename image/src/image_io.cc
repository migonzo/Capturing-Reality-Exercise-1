#include "image.h"

#include <stdexcept>
#include <type_traits>
#include <fstream>
#include <string>
#include <bitset>
#include <limits>
#include <cstdio>

#ifdef WITH_PNG
#include <png.h>
#endif
#ifdef WITH_JPEG
#include <jpeglib.h>
#endif
#ifdef WITH_EXR
#include <ImfOutputFile.h>
#include <ImfInputFile.h>
#include <ImfChannelList.h>
#include <ImfArray.h>
#endif

#undef max

namespace image_io {

#ifdef WITH_PNG
image<unsigned char> load_png(const char *filename)
{
	FILE *fp = std::fopen(filename, "rb");
	if (!fp) throw std::runtime_error("Cannot open PNG file");

	png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
	if (!png) {
		std::fclose(fp);
		throw std::runtime_error("Could not read PNG version");
	}

	png_infop info = png_create_info_struct(png);
	if (!info) {
		png_destroy_read_struct(&png, &info, nullptr);
		std::fclose(fp);
		throw std::runtime_error("Could not read PNG info");
	}

	if (setjmp(png_jmpbuf(png))) {
		png_destroy_read_struct(&png, &info, nullptr);
		std::fclose(fp);
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
		row_pointers[y] = (png_byte*)image.ptr2d(0, y);
	}
	png_read_image(png, row_pointers.data());

	png_destroy_read_struct(&png, &info, nullptr);
	std::fclose(fp);

	return image;
}

void save_png(const image<unsigned char> &image, const char *filename)
{
	FILE *fp = std::fopen(filename, "wb");
	if (!fp) throw std::runtime_error("Cannot open PNG file");

	png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
	if (!png) {
		std::fclose(fp);
		throw std::runtime_error("Cannot create PNG write struct");
	}

	png_infop info = png_create_info_struct(png);
	if (!info) {
		png_destroy_write_struct(&png, &info);
		std::fclose(fp);
		throw std::runtime_error("Cannot create PNG info");
	}

	if (setjmp(png_jmpbuf(png))) {
		png_destroy_write_struct(&png, &info);
		std::fclose(fp);
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
		row_pointers[y] = (png_byte*)image.ptr2d(0, y);
	}

	png_write_image(png, row_pointers.data());
	png_write_end(png, nullptr);

	png_destroy_write_struct(&png, &info);
	std::fclose(fp);
}
#endif

#ifdef WITH_JPEG
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
	FILE *fp = std::fopen(filename, "rb");
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
	std::fclose(fp);

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
		JSAMPROW row_pointer = (JSAMPROW)image.ptr2d(0, cinfo.next_scanline);
		jpeg_write_scanlines(&cinfo, &row_pointer, 1);
	}

	jpeg_finish_compress(&cinfo);
	jpeg_destroy_compress(&cinfo);
	std::fclose(fp);
}
#endif

#ifdef WITH_EXR
const char *exr_channel_names[4][4] = { { "Y", "", "", "" }, { "Y", "A", "", "" }, { "R", "G", "B", "" }, { "R", "G", "B", "A" } };

image<float> load_exr(const char *filename)
{
	Imf::InputFile file(filename);
	const Imf::Header &header = file.header();
	enum Channels { R = 1, G = 2, B = 4, A = 8, Y = 16 };
	int channels = 0;
	for (Imf::ChannelList::ConstIterator it = header.channels().begin(); it != header.channels().end(); ++it) {
		switch (it.name()[0]) {
		case 'R': channels |= R; break;
		case 'G': channels |= G; break;
		case 'B': channels |= B; break;
		case 'A': channels |= A; break;
		case 'Y': channels |= Y; break;
		default: throw std::runtime_error("Unsupported channel");
		}
	}
	std::size_t nc = 0;
	if (channels & R && channels & G && channels & B && channels & A) nc = 4;
	else if (channels & R && channels & G && channels & B) nc = 3;
	else if (channels & Y && channels & A) nc = 2;
	else if (channels & Y) nc = 1;
	else throw std::runtime_error("Unsupported channels");

	Imath::Box2i dw = header.dataWindow();
	image<float> image(dw.max.x - dw.min.x + 1, dw.max.y - dw.min.y + 1, nc);

	Imf::FrameBuffer fb;
	for (std::size_t c = 0; c < nc; ++c) {
		fb.insert(exr_channel_names[nc][c], Imf::Slice(Imf::FLOAT, (char*)((char*)image.data() + sizeof(float) * c), sizeof(float) * nc, sizeof(float) * image.width() * nc, 1, 1, 0.0));
	}

	file.setFrameBuffer(fb);
	file.readPixels(dw.min.y, dw.max.y);

	return image;
}

void save_exr(const image<float> &image, const char *filename)
{
	if (image.channels() < 1 || image.channels() > 4) throw std::runtime_error("Invalid number of channels");

	Imf::FrameBuffer fb;
	Imf::Header header(image.width(), image.height());
	std::size_t nc = image.channels();
	for (std::size_t c = 0; c < nc; ++c) {
		fb.insert(exr_channel_names[nc][c], Imf::Slice(Imf::FLOAT, (char*)((char*)image.data() + sizeof(float) * c), sizeof(float) * nc, sizeof(float) * image.width() * nc, 1, 1, 0.0));
		header.channels().insert(exr_channel_names[nc][c], Imf::Channel(Imf::FLOAT));
	}

	Imf::OutputFile outFile(filename, header);
	outFile.setFrameBuffer(fb);
	outFile.writePixels(image.height());
}
#endif

template <typename T>
image<T> load_bpm(const char *filename)
{
	std::ifstream is(filename);
	std::string magic;
	is >> magic;
	std::size_t channels;
	enum Header { FLOAT = 1, BIT = 2, ASCII = 4, RGB = 8 };
	int header = 0;
	if (magic == "Pf") {
		header |= FLOAT;
	} else if (magic == "PF") {
		header |= FLOAT;
		header |= RGB;
	} else if (magic == "P1") {
		header |= BIT;
		header |= ASCII;
	} else if (magic == "P2") {
		header |= ASCII;
	} else if (magic == "P3") {
		header |= RGB;
		header |= ASCII;
	} else if (magic == "P4") {
		header |= BIT;
	} else if (magic == "P5") {
	} else if (magic == "P6") {
		header |= RGB;
	}

	if (!!(header & FLOAT) != std::is_floating_point<T>::value) throw std::runtime_error("Cannot load a floating point file to a char image and vice versa");

	float scale = 1, f_val;
	std::size_t i_val, w, h;
	is >> w >> h;
	if (!(header & BIT)) is >> scale;
	scale = 1. / scale;
	if (!(header & FLOAT)) scale *= 255;

	image<T> image(w, h, header & RGB ? 3 : 1);

	is.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	if (!(header & ASCII)) {
		if (header & BIT) {
			std::vector<std::bitset<8>> row((image.width() + 7) / 8);
			for (std::size_t y = 0; y < image.height(); ++y) {
				is.read((char*)row.data(), row.size());
				for (std::size_t x = 0; x < image.width(); ++x) {
					image(x, y) = !row[x >> 3][7 - (x & 7)];
				}
			}
		} else {
			is.read((char*)image.data(), image.bytes());
		}
	} else if (header & FLOAT) {
		for (std::size_t i = 0; i < image.size(); ++i) {
			is >> f_val;
			image[i] = f_val;
		}
	} else {
		for (std::size_t i = 0; i < image.size(); ++i) {
			is >> i_val;
			image[i] = header & BIT ? !i_val : i_val;
		}
	}

	if (!(header & FLOAT)) {
		for (std::size_t i = 0; i < image.size(); ++i) {
			image[i] *= scale;
		}
	}

	return image;
}

template image<unsigned char> load_bpm<unsigned char>(const char *filename);
template image<float> load_bpm<float>(const char *filename);

template <typename T>
void save_bpm(const image<T> &image, const char *filename)
{
	std::ofstream os(filename, std::ios_base::binary);
	if (std::is_floating_point<T>::value && image.channels() == 1) os << "Pf\n";
	else if (std::is_floating_point<T>::value && image.channels() == 3) os << "PF\n";
	else if (image.channels() == 1) os << "P5\n";
	else if (image.channels() == 3) os << "P6\n";
	else throw std::runtime_error("Unsupported number of channels");

	os << image.width() << " " << image.height() << "\n";
	if (std::is_floating_point<T>::value) os << "1\n";
	else os << "255\n";
	os.write((const char*)image.data(), image.bytes());
}

template void save_bpm<unsigned char>(const image<unsigned char> &image, const char *filename);
template void save_bpm<float>(const image<float> &image, const char *filename);

enum Extension { PNG = 0x706e67, JPG = 0x6a7067, JPEG = 0x6a706567, EXR = 0x657872, PBM = 0x70626d, PGM = 0x70676d, PPM = 0x70706d, PFM = 0x70666d };

Extension get_extension(const char *filename)
{
	int dot = -1;
	for (int i = 0; filename[i]; ++i) {
		if (filename[i] == '.') dot = i;
	}
	uint32_t ext = 0;
	for (int i = dot + 1; filename[i]; ++i) {
		ext <<= 8;
		ext |= (uint8_t)::tolower(filename[i]);
	}
	return (Extension)ext;
}

template <>
image<unsigned char> load(const char *filename)
{
	switch (get_extension(filename)) {
#ifdef WITH_PNG
	case PNG:
		return load_png(filename);
#endif
#ifdef WITH_JPEG
	case JPG:
	case JPEG:
		return load_jpeg(filename);
#endif
	case PBM:
	case PGM:
	case PPM:
		return load_bpm(filename);
	default:
		throw std::runtime_error("Unsupported file extension");
	}
}
template <>
image<float> load(const char *filename)
{
	switch (get_extension(filename)) {
#ifdef WITH_EXR
	case EXR:
		return load_exr(filename);
#endif
	case PFM:
		return load_bpm<float>(filename);
	default:
		throw std::runtime_error("Unsupported file extension");
	}
}

template <>
void save(const image<unsigned char> &image, const char *filename)
{
	switch (get_extension(filename)) {
#ifdef WITH_PNG
	case PNG:
		return save_png(image, filename);
#endif
#ifdef WITH_JPEG
	case JPG:
	case JPEG:
		return save_jpeg(image, filename);
#endif
	case PBM:
	case PGM:
	case PPM:
		return save_bpm(image, filename);
	default:
		throw std::runtime_error("Unsupported file extension");
	}
}
template <>
void save(const image<float> &image, const char *filename)
{
	switch (get_extension(filename)) {
#ifdef WITH_EXR
	case EXR:
		return save_exr(image, filename);
#endif
	case PFM:
		return save_bpm<float>(image, filename);
	default:
		throw std::runtime_error("Unsupported file extension");
	}
}

}