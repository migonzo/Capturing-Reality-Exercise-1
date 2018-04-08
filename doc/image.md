# image
This class provides basic functionality for handling, loading and storing functionality for images. Supported file formats are PNG, JPEG and EXR.

## image class
The image class is a templatized for the native data type. The width, height and number of channels is variable during the run time. There are aliases for byte images `image_b` and for floating point images `image_f`.

The constructor takes an image width and optionally a height and the number of channels:
```
image_b img(512, 128, 3); // RGB image of size 512x128
```

## Accessing the pixels
Pixels can be accessed using `operator[]` for 1D indexing and `operator()` for 2D (+channel) indexing:
```
unsigned char green_at_12_34 = img(12, 34, 1);
```

The `lin_at` method already implements bi-linear interpolation:
```
unsigned char green_interp = img.lin_at(12.34f, 56.78f, 1);
```

You can also use any array-like type that implements `operator[]`, like `Eigen::Vector2d` or `std::vector`:
```
Eigen::Vector2d v(12.34, 56.78);
unsigned char green_interp = img.lin_at(v, 1);
```

## Loading and saving image files
Images can be loaded using the `image_io::load` function:
```
image_b img = image_io::load("file.png");
```

Afterwards, they can be saved using the `image_io::save` function:
```
image_io::save(img, "bla.jpg");
```

## image.h
There are some other basic functions in the `image/include/image.h` file. Feel free to take a deeper look.