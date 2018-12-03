#pragma once
#include<pcl\point_types.h>
#include<pcl\filters\voxel_grid.h>
#include<pcl\filters\statistical_outlier_removal.h>
//必须放在最前面，不知道为什么放在opencv后面就会无法使用statistical_outlier_removal这个方法（幺蛾子事件）

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <limits>
#include<opencv2/opencv.hpp>    
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<vector>

using namespace std;

#define PI 3.1415926f

enum dof_type
{
	x_axis_rotate, y_axis_rotate, z_axis_rotate,
	x_axis_trans, y_axis_trans, z_axis_trans
};

enum shape_type
{
	x_axis_scale, y_axis_scale, z_axis_scale,
	x_axis_trans_, y_axis_trans_, z_axis_trans_
};

//关节参数
const int num_thetas = 26;
const int num_shape_thetas = 20;
const int num_joints = 21;
typedef Eigen::Matrix<float, num_thetas, 1> Thetas;


//矩阵定义
typedef Eigen::Matrix<float, 2, 3> Matrix_2x3;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Matrix_MxN;
typedef Eigen::Matrix<float, 2, Eigen::Dynamic> Matrix_2xN;
typedef Eigen::Matrix<float, 3, Eigen::Dynamic> Matrix_3xN;
typedef Eigen::Matrix<float, Eigen::Dynamic, 2> Matrix_Nx2;
typedef Eigen::Matrix<float, Eigen::Dynamic, 3> Matrix_Nx3;
typedef Eigen::VectorXf Vector_N;
typedef Eigen::Vector4f Vector4;

/// Nan for the default type
inline float nan() { return std::numeric_limits<float>::quiet_NaN(); }
inline float inf() { return std::numeric_limits<float>::max(); }

/// Linear system lhs*x=rhs
struct LinearSystem {
	Matrix_MxN Jt_J; // J^T*J
	Vector_N Jt_e; // J^T*r
	LinearSystem() {}
	LinearSystem(int n) {
		Jt_J = Matrix_MxN::Zero(n, n);
		Jt_e = Vector_N::Zero(n);
	}
};

struct Collision
{
	int id;
	int adjscent_id[2];
	Eigen::Vector4f init_Position;    //局部坐标系
	Eigen::Vector4f updata_Position;    //世界坐标系

	float init_radius;
	float updata_radius;
	int joint_index;
};

struct TrackingError {
	float error_3D;
	float error_2D;
	static TrackingError zero() { return{ 0, 0 }; }
};

//所有从输入可以得到的数据
struct InputDataForTrack
{
	int index;//数据序号

	cv::Mat depth;
	cv::Mat Handsegment;
	cv::Mat Handsegment_inv;

	pcl::PointCloud<pcl::PointXYZ> pointcloud_from_depth;
	pcl::PointCloud<pcl::PointXYZ> pointcloud_filtered;
	pcl::PointCloud<pcl::PointXYZ> pointcloud_downsample;

	float params[num_thetas];
	int *idxs_img;
	int *inv_idxs_img;

	Eigen::Matrix<float, num_joints, 3> joint_read;
};