#pragma once
#include"Energy_Fitting.h"
#include"Energy_Limited.h"
#include"Energy_Collision.h"
#include"Energy_Temporal.h"
#include"Energy_Damping.h"
#include"TrackingMonitor.h"
#include"HandModel.h"
#include"MyPointCloud.h"
#include"DistanceTransform.h"

class Worker {

public:
	struct Settings {
		int termination_max_iters = 1;
		int termination_max_rigid_iters = 1;
	} _settings;
	Settings*const settings = &_settings;

public:

	Camera *camera;
	HandModel * model;
	TrackingError tracking_error;
	energy::Energy_Fitting E_fitting;
	energy::Energy_Limited E_limits;
	energy::Energy_Collision E_collision;
	energy::Energy_Temporal E_temporal;
	energy::Energy_Damping E_damping;
	TrackingMonitor monitor;
	MyPointCloud mypointcloud;
	DistanceTransform distance_transform;

public:
	//相关目标点变量
	//拟合关节点
	Matrix_Nx3 Target_joints;
	void save_target_joints();
	void load_target_joints();
	Matrix_Nx3 Target_vertices;
	void save_target_vertices();
	void load_target_vertices();

	void save_DatasetParams(int itr);

	//所有从输入可以得到的数据
	InputDataForTrack input_data_for_track;

public:
	Worker(HandModel * input_model, Camera *input_camera);
	~Worker() 
	{
		delete model;
	}

	void track(int iter, bool with_glove);
	void track_shape(int iter, bool with_glove);
	bool track_till_convergence(bool with_glove, bool shapeTracking);




	void fetch_Input(int index);

	void fetch_KinectInput(int index);
	void fetch_DatasetMSRA_14Inpute(int index);
	void fetch_DatasetMSRA_15Inpute(int index);
};