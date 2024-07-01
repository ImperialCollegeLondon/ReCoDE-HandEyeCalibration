# Introduction to OpenCV Library
OpenCV is a powerful C++ library for applications that involve operations on images and computer vision algorithms. A comprehensive version of its function/class documentation can be found at [OpenCV documentation](https://docs.opencv.org/4.x/). In this section, we only provide descriptions of the basic functions that were used in this project.

The following header files need to be included:

```cpp
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
```

## Read and write images


In OpenCV, all images are stored as matrix objects `cv::Mat`. To read images, we first declare image objects `cv::Mat img_depth, img_IR;` and use the built-in function `cv::imread` to read images from a local path.

```cpp
img_IR = cv::imread(IR_file_path);
```

The size of the image is `img_IR.rows` $\times$ `img_IR.cols`.

When we need to save an image `img_overlay` to a given path, we first create a string object for the path `img_overlay_filename`, then use the built-in OpenCV function `cv::write` 
```cpp
cv::imwrite(img_overlay_filename, img_overlay);
```

## Create a new image

To create a depth mask in function `FeatureDetection::ReadPointCloudPCD`, we first need to declare an empty binary image with the size of `n_rows` $\times$ `n_cols`:

```cpp
img = cv::Mat(n_rows, n_cols, CV_8UC1, cv::Scalar(0)); 
```

`CV_8UC1` decides that the image we just created only has one channel, and each pixel stores an 8-digit unsigned integer. Then we start to read in a point cloud representing the current frame and assign 255 (white) to pixels that have non-zero depth values, otherwise 0 (black).

```cpp
    for(size_t i=0; i < n_rows; i++)
    {
        for(size_t j=0; j< n_cols; j++)
        {
            int index = i * n_cols + j;
            double pixel_depth = cloud->points[index].z;
            if(pixel_depth != 0.0)
            {
                img.at<uchar>(i,j) = 255;
            }
        }
    }
```

We use `.at` to assign values to a certain pixel with position `(i,j)`. We use the template operator `<uchar>` because we have declared a single-channeled black and white image.

## Contour Detection

A contour is described by a list of points on the image as `std::vector<cv::Point>`. Therefore, all detected contours on an image are stored as `std::vector<std::vector<cv::Point>>`. We use the built-in fuction `cv::findContours` to detect contours of markers in a depth mask frame. 

```cpp
cv::findContours(image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
```

After that, we further sort the list of detected contours in descending order of contour areas by calling

```cpp
std::sort(contours.begin(), contours.end(),[](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2)
              {return cv::contourArea(c1,false) > cv::contourArea(c2, false);}); 
```

As we iterate through the list of detected contours, we want to draw each contour on the image, and hence we use the built-in function `cv:drawContours`

```cpp
cv::drawContours(image_with_contour, contours, i, cv::Scalar(255), -1, 8,hierarchy);
```
The option value `-1` represents that we want to fill in the contour with colour white (`cv::Scalar(255)`).
