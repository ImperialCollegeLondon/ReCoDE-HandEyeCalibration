# Introduction to Point Cloud Library (PCL)

A comprehensive description of Point Cloud Library's functions and classes can be found at [PCL documentation](https://pointclouds.org/documentation/). In this section, we only provide descriptions of basic function implementations that were used in this project. We utilised PCL for generating depth masks and obtaining the 3D position of surface points on the markers.

The following header files need to be included:
```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
```

## Initialisation of point clouds

A point cloud is a container that stores points that represent a 3D geometric object. To describe these points, usually their positions, normal directions and colour information are used. We only leverage their 3D positions in this project.
 
```cpp
using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;
```

For each recorded frame, we create a point cloud to store surface points captured by the depth sensing camera.

```cpp
PointCloudT::Ptr frame_cloud(new PointCloudT);
```

## Read and Write

For a given recorded `.pcd` file, we use the built-in function `pcl::io::loadPCDFile` to read and store information to a point cloud. 

```cpp
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_path, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }
