<!-- Your Project title, make it sound catchy! -->

# Hand-eye Calibration for Medical Robots

<!-- Provide a short description to your project -->

## Description

Hand-eye calibration is a well-studied topic in the field of robotics. Applications that involve the use cameras typically require conducting hand-eye calibration beforehand. This project presents a pipeline for conducting hand-eye calibration using a bespoke marker for a medical robot (the first generation da Vinci research kit). The experimental setup is shown in Fig 1.

| ![Setup](https://github.com/ImperialCollegeLondon/ReCoDE-HandEyeCalibration/blob/main/docs/Pics_for_demo/Setup.jpg?raw=true) |
|:--:|
| Fig 1. Experimental setup |

The method presented in this project aims to conduct registration between two sets of point cloud using singular value decomposition (SVD). This project aims to instruct students to understand basic knowledge of hand-eye calibration and grasp essential skills in using computer vision libraries such as OpenCV and Point Cloud library in C++. Hand-eye calibration results can be visualised through 2D back projections. Fig 2 and Fig 3 display the back projection of a surgical tool shaft after accurate and inaccurate hand-eye calibrations, respectively. 

| ![AfterCalib](https://github.com/ImperialCollegeLondon/ReCoDE-HandEyeCalibration/blob/main/docs/Pics_for_demo/AccurateCalib.gif?raw=true "AfterCalib") | ![BeforeCalib](https://github.com/ImperialCollegeLondon/ReCoDE-HandEyeCalibration/blob/main/docs/Pics_for_demo/InaccurateCalib.gif?raw=true "BeforeCalib") |
|:--:|:--:|
| Fig 2. Overlay with accurate calibration | Fig 3. Overlay without inaccurate calibration |

<!-- What should the students going through your exemplar learn -->

## Learning Outcomes

- Understand what hand-eye calibration is and prevalent school of thoughts in solving this problem.
- Develop basic skills in writing an object-oriented project in C++.
- Develop basic skills in using computer vision libraries, such as OpenCV and Point Cloud Library. 

<!-- How long should they spend reading and practising using your Code.
Provide your best estimate -->

| Task       | Time    |
| ---------- | ------- |
| Reading    | 3 hours |
| Practising | 2 hours |

## Requirements

<!--
If your exemplar requires students to have a background knowledge of something
especially this is the place to mention that.

List any resources you would recommend to get the students started.

If there is an existing exemplar in the ReCoDE repositories link to that.
-->

### Academic
- Basic knowledge on robotics (eg. Denavit–Hartenberg (DH) parameters, forward kinematics)
- Hand-eye calibration and fundamental knowledge on computer vision (eg. camera intrinsic matrix, extrinsic matrix, image projection)
- Linear algebra (eg. Singular Value Decomposition (SVD))
Detailed information can be referred to `docs/Background.md`
<!-- List the system requirements and how to obtain them, that can be as simple
as adding a hyperlink to as detailed as writting step-by-step instructions.
How detailed the instructions should be will vary on a case-by-case basis.

Here are some examples:

- 50 GB of disk space to hold Dataset X
- Anaconda
- Python 3.11 or newer
- Access to the HPC
- PETSc v3.16
- gfortran compiler
- Paraview
-->

### System
- A C++ toolchain along with the necessary development libraries:
    - Eigen library (Eigen 3)
    - OpenCV library (OpenCV 4.6)
    - Point cloud library (PCL 1.11)
<!-- Instructions on how the student should start going through the exemplar.

Structure this section as you see fit but try to be clear, concise and accurate
when writing your instructions.

For example:
Start by watching the introduction video,
then study Jupyter notebooks 1-3 in the `intro` folder
and attempt to complete exercise 1a and 1b.

Once done, start going through through the PDF in the `main` folder.
By the end of it you should be able to solve exercises 2 to 4.

A final exercise can be found in the `final` folder.

Solutions to the above can be found in `solutions`.
-->

## Getting Started
Background knowledge on robotics and computer vision can be referred to `docs/Background.md`. The workflow of this project can be referred to `docs/Code_Overview.md`. `docs/EigenLibraryIntro.md`, `docs/OpenCVLibraryIntro.md` and `docs/PointCloudLibraryIntro.md` contain information on the basic usage of these C++ libraries, and explanations on related fuctions written in this project using these libraries. `docs/input.md` provides information of input files used in this project. 
<!-- An overview of the files and folder in the exemplar.
Not all files and directories need to be listed, just the important
sections of your project, like the learning material, the code, the tests, etc.

A good starting point is using the command `tree` in a terminal(Unix),
copying its output and then removing the unimportant parts.

You can use ellipsis (...) to suggest that there are more files or folders
in a tree node.

-->

## Project Structure

```log
.
├── input
│   ├── IR
│   ├── RGB
│   ├── pcd
│   ├── Acusense_RGB_K.txt
│   ├── ExtrinsicMat.txt
│   ├── q_history.txt
│   ├── pos_act.txt
│   └── pos_des.txt
├── output
├── src
│   ├── main.cc
│   ├── FeatureDetection.cc
│   └── utils.cc
├── include
│   ├── FeatureDetection.hpp
│   └── utils.hpp
├── CMakeLists.txt 
├── docs
│   ├── Background.md
│   ├── Code_Overview.md
│   ├── EigenLibraryIntro.md
│   ├── FeatureDetection.md
│   ├── OpenCVLibrary.md
│   ├── PointClourLibraryIntro.md
│   └── input.hpp
├── README.md
├── LICENSE.md
├── mkdocs.yml
├── doxide.yaml
└── requirements.txt 
```

<!-- Change this to your License. Make sure you have added the file on GitHub -->

## License

This project is licensed under the [BSD-3-Clause license](LICENSE.md)
