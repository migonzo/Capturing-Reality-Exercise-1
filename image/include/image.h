#pragma once

#include <cstddef>
#include <vector>

template <typename T>
struct image {
	image(std::size_t width, std::size_t height = 1, std::size_t channels = 1) : w(width), h(height), nc(channels), _data(w * h * nc)
	{}
	image() : image(0, 0, 0)
	{}

	std::size_t width() const
	{
		return w;
	}
	std::size_t height() const
	{
		return h;
	}
	std::size_t channels() const
	{
		return nc;
	}
	std::size_t pixels() const
	{
		return width() * height();
	}
	std::size_t size() const
	{
		return pixels() * channels();
	}
	std::size_t bytes() const
	{
		return size() * sizeof(T);
	}

	void resize(std::size_t width, std::size_t height = 1, std::size_t channels = 1)
	{
		_data.resize(w * h * nc);
	}

	template <typename U>
	void resize(const image<U> &other)
	{
		resize(other.width(), other.height(), other.channels);
	}

	// raw indexing, ignoring channels
	T *data(std::size_t idx = 0)
	{
		return _data.data() + idx;
	}
	const T *data(std::size_t idx = 0) const
	{
		return _data.data() + idx;
	}

	// 1D indexing of the image matrix
	T *ptr(std::size_t idx, std::size_t c = 0)
	{
		return data(idx * nc + c);
	}
	const T *ptr(std::size_t idx, std::size_t c = 0) const
	{
		return data(idx * nc + c);
	}

	// 2D indexing of the image matrix
	T *ptr2d(std::size_t x, std::size_t y, std::size_t c = 0)
	{
		return data(y * w * nc + x * nc + c);
	}
	const T *ptr2d(std::size_t x, std::size_t y, std::size_t c = 0) const
	{
		return data(y * w * nc + x * nc + c);
	}

	// raw indexing of the image matrix
	T &operator[](std::size_t idx)
	{
		return *data(idx);
	}
	const T &operator[](std::size_t idx) const
	{
		return *data(idx);
	}

	// 1D indexing of the image matrix
	T &at(std::size_t idx, std::size_t c = 0)
	{
		return *ptr(idx, c);
	}
	const T &at(std::size_t idx, std::size_t c = 0) const
	{
		return *ptr(idx, c);
	}

	// 2D indexing of the image matrix
	T &at2d(std::size_t x, std::size_t y, std::size_t c = 0)
	{
		return *ptr2d(x, y, c);
	}
	const T &at2d(std::size_t x, std::size_t y, std::size_t c = 0) const
	{
		return *ptr2d(x, y, c);
	}
	T &operator()(std::size_t x, std::size_t y, std::size_t c = 0)
	{
		return *ptr2d(x, y, c);
	}
	const T &operator()(std::size_t x, std::size_t y, std::size_t c = 0) const
	{
		return *ptr2d(x, y, c);
	}

	template <typename U = float>
	U at_lin(float x, float y, std::size_t c = 0) const
	{
		std::size_t xl = x, yl = y;
		std::size_t xu = xl + 1, yu = yl + 1;
		float ax = x - xl, ay = y - yl;

		// the user has to take care about the other boundaries
		if (xu == w) xu = w - 1;
		if (yu == h) yu = h - 1;

		U p0 = (*this)(xl, yl, c), p1 = (*this)(xu, yl, c), p2 = (*this)(xl, yu, c), p3 = (*this)(xu, yu, c);
		U l0 = p0 * (1.f - ax) + p1 * ax;
		U l1 = p2 * (1.f - ax) + p3 * ax;
		return l0 * (1.f - ay) + l1 * ay;
	}

	// vector variants
	template <typename V>
	T &at_v(const V &v, std::size_t c = 0)
	{
		return (*this)(v[0], v[1], c);
	}
	template <typename V>
	const T &at_v(const V &v, std::size_t c = 0) const
	{
		return (*this)(v[0], v[1], c);
	}
	template <typename U = float, typename V>
	U at_v_lin(const V &v, std::size_t c = 0) const
	{
		return at_lin(v[0], v[1], c);
	}

private:
	std::size_t w, h, nc;
	std::vector<T> _data;
};

typedef image<unsigned char> image_b;
typedef image<float> image_f;

namespace image_io {

#ifdef WITH_PNG
image_b load_png(const char *filename);
void save_png(const image_b &image, const char *filename);
#endif

#ifdef WITH_JPEG
image_b load_jpeg(const char *filename);
void save_jpeg(const image_b &image, const char *filename, int quality = 80);
#endif

#ifdef WITH_EXR
image_f load_exr(const char *filename);
void save_exr(const image_f &image, const char *filename);
#endif

template <typename T = unsigned char> image<T> load_pbm(const char *filename);
template <typename T = unsigned char> void save_pbm(const image<T> &image, const char *filename);

template <typename T = unsigned char> image<T> load(const char *filename);
template <typename T = unsigned char> void save(const image<T> &image, const char *filename);

// deprecated
template <typename T = unsigned char> image<T> load_bpm(const char *filename)
{
	return load_pbm(filename);
}
template <typename T = unsigned char> void save_bpm(const image<T> &image, const char *filename)
{
	save_pbm(image, filename);
}

}

namespace image_manip {

image_f grayscale(const image_b &src);

}