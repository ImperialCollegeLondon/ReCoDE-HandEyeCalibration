# Introduction to OpenCV Library
OpenCV is a powerful library for applications that involve operations on images and computer vision algorithm. A compresive version of function/class documentation can be referred to [OpenCV documentation](https://docs.opencv.org/4.x/). In this markdown file, we only provide descriptions of basic functions that were used in this project. The following header files need to be included.

```cpp
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
```

## Read and write images <br />

In OpenCV, all images are stored as matrix object `cv::Mat`. To read images, we first declare image objects `cv::Mat img_depth, img_IR;` and use the built-in function `cv::imread` to read images from a local path. <br />

```c
img_IR = cv::imread(IR_file_path);
```

The size of the image is `img_IR.rows` $\times$ `img_IR.cols`.

When we need to save an image `img_overlay` to a given path, we first create a string object for the path `img_overlay_filename`, then use the built-in OpenCV function `cv::write` 
```cpp
cv::imwrite(img_overlay_filename, img_overlay);
```

## Contour Detection
