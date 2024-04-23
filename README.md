<!-- Your Project title, make it sound catchy! -->

# Hand-eye Calibration for Medical Robots

<!-- Provide a short description to your project -->

## Description

Hand-eye calibration is a well-studied topic in the field of robotics. Applications that involve the use cameras typically require conducting hand-eye calibration beforehand. This project presents a pipeline for conducting hand-eye calibration using a bespoke marker for a medical robot. The method presented in this project hinges on conducting registration between two sets of point cloud using singular value decomposition (SVD). This project aims to instruct students to understand basic knowledge of hand-eye calibration and grasp essential skills in using computer vision libraries such as OpenCV and Point Cloud library in C++. 

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
| Practising | 3 hours |

## Requirements

<!--
If your exemplar requires students to have a background knowledge of something
especially this is the place to mention that.

List any resources you would recommend to get the students started.

If there is an existing exemplar in the ReCoDE repositories link to that.
-->

### Academic

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
├── examples
│   ├── ex1
│   └── ex2
├── src
|   ├── file1.py
|   ├── file2.cpp
|   ├── ...
│   └── data
├── app
├── docs
├── main
└── test
```

<!-- Change this to your License. Make sure you have added the file on GitHub -->

## License

This project is licensed under the [BSD-3-Clause license](LICENSE.md)
