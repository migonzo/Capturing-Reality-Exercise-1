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

	void resize(std::size_t width, std::size_t height = 1, std::size_t channels = 1)
	{
		_data.resize(w * h * nc);
	}

	T *data(std::size_t idx = 0)
	{
		return _data.data() + idx;
	}
	const T *data(std::size_t idx = 0) const
	{
		return _data.data() + idx;
	}

	T *data(std::size_t x, std::size_t y, std::size_t c = 0)
	{
		return data(y * w * nc + x * nc + c);
	}
	const T *data(std::size_t x, std::size_t y, std::size_t c = 0) const
	{
		return data(y * w * nc + x * nc + c);
	}

	T &operator[](std::size_t idx)
	{
		return *data(idx);
	}
	const T &operator[](std::size_t idx) const
	{
		return *data(idx);
	}

	T &operator()(std::size_t x, std::size_t y = 0, std::size_t c = 0)
	{
		return *data(x, y, c);
	}
	const T &operator()(std::size_t x, std::size_t y = 0, std::size_t c = 0) const
	{
		return *data(x, y, c);
	}

	template <typename U = float>
	U bilin(float x, float y, std::size_t c = 0) const
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
	template <typename U = float, typename V>
	U bilin(const V &v, std::size_t c = 0) const
	{
		return bilin(v[0], v[1], c);
	}

private:
	std::size_t w, h, nc;
	std::vector<T> _data;
};

namespace image_io {

image<unsigned char> load_png(const char *filename);
image<unsigned char> load_jpeg(const char *filename);
image<float> load_exr(const char *filename);

void save_png(const image<unsigned char> &image, const char *filename);
void save_jpeg(const image<unsigned char> &image, const char *filename, int quality = 80);
void save_exr(const image<float> &image, const char *filename);

}