# Introduction to Eigen Library

Eigen library is a useful library for linear algebra operations. A comprehensive documentation of Eigen classes and functions is listed on [Eigen library documentation](https://eigen.tuxfamily.org/dox/). This markdown file only provides descriptions for basic implementations that were used in this project. 

## Basic matrix operation
To declare a matrix with `n_rows` and `n_cols` populated with double numbers, we use the following command
```cpp
Eigen::MatrixXd mat(n_rows, n_cols);
```
When `n_cols` reduces to 1, `mat` reduces to a one-dimensional vector. In this case, we can directly declare it as a vector using the following command.
```cpp
Eigen::VectorXd mat(n_rows);
```
By convention, all declared one-dimensional vectors are column vectors, and if we want to convert them into row vectors by using
```cpp
Eigen::VectorXd mat_row = mat.transpose();
```

## Singular Value Decomposition (SVD)
By leveraging built-in functions, we can implement linear algebra algorithms on matrices. We have created the function `SVD` in `utils.cc` to decompose an `n_rows` $\times$ `n_cols` matrix `A` into `A=USV'`. 
```cpp
void SVD(const Eigen::MatrixXd &A, Eigen::MatrixXd &U, Eigen::MatrixXd &S, Eigen::MatrixXd &V)
```
We first create an SVD object by using the following command
```cpp
Eigen::JacobiSVD<Eigen::MatrixXd> svd_solver(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
```
From mathematical knowledge, we know that `U` is a `n_rows` $\times$ `n_rows` matrix, `S` is a `n_rows` $\times$ `n_cols` sparse matrix with singular values along its main diagonal, and `V` is a `n_cols` $\times$ `n_cols` matrix. 
```cpp
    U.resize(A.rows(), A.rows());
    S.resize(A.rows(), A.cols());
    S.setZero(); 
    V.resize(A.cols(), A.cols());
```

Then we obtain `U` and `V` through
```cpp
    U = svd_solver.matrixU();
    V = svd_solver.matrixV();
```
Finally, we populate matrix `S` by first calculating the singular values followed by arranging these values along its main its diagonal.
```cpp
    Eigen::VectorXd sigma_list = svd_solver.singularValues();
    for(size_t i=0; i < sigma_list.size(); i++)
    {
        S(i,i) = sigma_list[i];
    }
```

## Find hand-eye transformation matrix using SVD
The function `SVD_rigid_transform` in `utils.cc` aims to find the hand-eye transformation matrix through SVD operation. 
```cpp
Eigen::Matrix4d SVD_rigid_transform(const Eigen::MatrixXd &pts1, const Eigen::MatrixXd &pts2)
```
The inputs for this function are two list of 3D points expressed in two different coordinate frames, and the output is a 4 $\times$ 4 homogeneous transformation matrix between the two coordinate systems. We first need to verify that the two matrices are of the same size, that is with known correspondences. 
```cpp
    int n_pts1 = pts1.rows(), n_pts2 = pts2.rows();
    if(n_pts1 != n_pts2)
    {
        throw std::invalid_argument("pts1 size doesn't match with pts2");
    }
```
Next, we copy the values of two input matrices to two new matrices for further matrix operation. We first declare two empty matrices with known sizes and then copy matrix components row by row. We use the operator `<<` to populate matrix components. 
```cpp
    Eigen::MatrixXd pts1_mat(n_pts1, 3), pts2_mat(n_pts2, 3);
    for(size_t i=0; i<n_pts1; i++)
    {
        pts1_mat.row(i) << pts1(i,0), pts1(i,1), pts1(i,2);
        pts2_mat.row(i) << pts2(i,0), pts2(i,1), pts2(i,2);
    }
```

Then we try to find the central position for both sets of points and then create two lists of points representing their offsets from the central positions
```cpp
    Eigen::VectorXd mean_1 = pts1_mat.colwise().mean(), 
                    mean_2 = pts2_mat.colwise().mean();
    pts1_mat.rowwise() -= mean_1.transpose();
    pts2_mat.rowwise() -= mean_2.transpose();
```
The covariance matrix is constructed as 
```cpp
auto H = pts1_mat.transpose() * pts2_mat;
```
We apply SVD algorithm on `H`, and we have
```cpp
    Eigen::MatrixXd U,S,V; // 3*3 matrices
    SVD(H,U,S,V);
```
The rotation and translatoin components of hand-eye transformation are 
```cpp
    auto R = V * U.transpose();
    auto t = -R * mean_1 + mean_2;
```
Finally, we can construct the 4 $\times$ 4 homogeneous hand-eye transformation matrix as
```
    Eigen::Matrix4d T_12 = Eigen::Matrix4d::Identity();
    T_12.topLeftCorner(3,3) = R.topLeftCorner(3,3);
    T_12.topRightCorner(3,1) = t.leftCols(1);
    return T_12;
```
Commands `topLeftCorner`, `topRightCorner` and `leftCols` are used for matrix block operation.

## IO operation
The function `ReadMatrixFromTxt` is used for storing values in a `.txt` file into a matrix.
```cpp
void ReadMatrixFromTxt(const std::string path, Eigen::MatrixXd &output)
```
We declare a file streamer `file` which iterates through each row of the `.txt` file and stores row components to a string object `line`. 
```cpp
    std::ifstream file(path);
    std::string line, word;
    std::vector<double> row_vec;
    std::vector<std::vector<double>> matrix_input;
    if(!file) 
    {
        std::cout<<"reference file cannot be read in"<<std::endl;
        return;
    }
```
We then parse row-wise components by text splitter, space in this project. 
```cpp
    int row_count = 0, n_cols;
    while(std::getline(file, line))
    {
        std::stringstream ss(line);
        int count = 0;
        while(std::getline(ss, word, ' '))
        {
            row_vec.push_back(std::stod(word));
            count++;
        }
        row_count++;
        matrix_input.push_back(row_vec);
        n_cols = row_vec.size();
        row_vec.clear();
    }
```
Finally, we copy the elements in `matrix_input` to `output`
```cpp
    output.resize(row_count, n_cols);
    for(size_t i=0; i<row_count; i++)
    {
        for(size_t j=0; j<n_cols; j++)
        {
            output(i,j) = matrix_input[i][j];
        }
    }
```
