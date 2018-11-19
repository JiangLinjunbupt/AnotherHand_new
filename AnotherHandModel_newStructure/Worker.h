#pragma once
#include"HandModel.h"
#include"Energy_Fitting.h"
#include"Energy_Limited.h"
#include"Energy_Collision.h"
#include"Energy_Temporal.h"
#include"Energy_Damping.h"
#include"TrackingMonitor.h"

class Worker {

public:
	struct Settings {
		int termination_max_iters = 1;
		int termination_max_rigid_iters = 1;
	} _settings;
	Settings*const settings = &_settings;


public:

	HandModel * model;
	TrackingError tracking_error;
	energy::Energy_Fitting E_fitting;
	energy::Energy_Limited E_limits;
	energy::Energy_Collision E_collision;
	energy::Energy_Temporal E_temporal;
	energy::Energy_Damping E_damping;
	TrackingMonitor monitor;

public:
	//相关目标点变量
	//拟合关节点
	Matrix_Nx3 Target_joints;
	void save_target_joints();
	void load_target_joints();
	Matrix_Nx3 Target_vertices;
	void save_target_vertices();
	void load_target_vertices();

	//拟合点云数据
	pcl::PointCloud<pcl::PointXYZ> Downsample_pointcloud;
	float Downsample_pointcloud_center_x;
	float Downsample_pointcloud_center_y;
	float Downsample_pointcloud_center_z;
	int *idx_img;

public:
	Worker(HandModel * model);
	~Worker() 
	{
		delete model;
		delete idx_img;
	}
	void fetch_Input(string fileName);
	void track(int iter, bool with_glove);
	void track_shape(int iter, bool with_glove);
	bool track_till_convergence(bool with_glove, bool shapeTracking);
};