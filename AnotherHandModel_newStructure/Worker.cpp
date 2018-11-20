#include "Worker.h"

Worker::Worker(HandModel * input_model) {
	this->model = input_model;
	E_fitting.init(input_model);
	E_limits.init(input_model);
	//E_collision.init(input_model);
	//E_temporal.init(input_model);

	Target_joints = Eigen::MatrixXf::Zero(model->NumofJoints, 3);
	Target_vertices = Eigen::MatrixXf::Zero(model->NumofVertices, 3);
	Downsample_pointcloud.points.clear();
	Downsample_pointcloud_center_x = 0.0f;
	Downsample_pointcloud_center_y = 0.0f;
	Downsample_pointcloud_center_z = 0.0f;
	idx_img = new int[424 * 512];

}

bool Worker::track_till_convergence(bool with_glove,bool shapeTracking) {

	for (int i = 0; i < settings->termination_max_iters; ++i) {
		if(shapeTracking)  track_shape(i, with_glove);
		else track(i, with_glove);
		//tracking_error_optimization[i] = tracking_error;
	}

	return monitor.is_failure_frame(tracking_error.error_3D, tracking_error.error_2D, E_fitting.settings->fit2D_enable);
}

void Worker::track(int iter, bool with_glove) {
	bool eval_error = (iter == settings->termination_max_iters - 1);                 //最后一代： 估计误差
	bool rigid_only = (iter < settings->termination_max_rigid_iters);                //termination_max_rigid_iters = 1；也就是说：第一代是rigid_only


	///--- Optimization phases	
	LinearSystem system(num_thetas);
	//eval_error = true;
	E_fitting.track(system, Downsample_pointcloud, idx_img);
	//E_collision.track(system);
	//E_temporal.track(system);
	E_limits.track(system, with_glove);
	E_damping.track(system);
	/*if (rigid_only)
		energy::Energy::rigid_only(system);*/

	//--- Solve 
	Eigen::VectorXf delta_thetas = energy::Energy::solve(system);

	for (int i = 0; i < this->model->NumberofParams; i++)
		this->model->Params[i] += delta_thetas(i);

	this->model->Updata(this->model->Params, this->model->Shape_Params);
	//E_temporal.update(current_frame.id, _thetas);
}

void Worker::track_shape(int iter, bool with_glove)
{
	bool eval_error = (iter == settings->termination_max_iters - 1);                 //最后一代： 估计误差
	bool rigid_only = (iter < settings->termination_max_rigid_iters);                //termination_max_rigid_iters = 1；也就是说：第一代是rigid_only


	///--- Optimization phases	
	LinearSystem system(num_shape_thetas);
	//eval_error = true;
	E_fitting.track_Shape_2(system, Downsample_pointcloud);
	//E_fitting.track_Shape(system, Target_vertices, iter);
	//E_fitting.track_Shape_Joints(system, Target_joints, rigid_only, eval_error, tracking_error.error_3D, tracking_error.error_2D, iter);
	//E_collision.track(system);
	//E_temporal.track(system);
	E_limits.track_shape(system);
	E_damping.track_shape(system);
	/*if (rigid_only)
	energy::Energy::rigid_only(system);*/


	/*system.Jt_J.col(0).setZero();
	system.Jt_J.row(0).setZero();
	system.Jt_e.row(0).setZero();*/

	//int end = 4;
	//for (int c = 0; c < end; ++c)
	//	system.Jt_J.col(c).setZero();
	//for (int r = 0; r < end; ++r)
	//{
	//	system.Jt_J.row(r).setZero();
	//	system.Jt_e.row(r).setZero();
	//}
	//int start = 5;
	//for (int c = start; c < num_shape_thetas; ++c)
	//	system.Jt_J.col(c).setZero();
	//for (int r = start; r < num_shape_thetas; ++r)
	//{
	//	system.Jt_J.row(r).setZero();
	//	system.Jt_e.row(r).setZero();
	//}

	//--- Solve 
	Eigen::VectorXf delta_thetas = energy::Energy::solve(system);


	for (int i = 0; i < this->model->NumofShape_Params; i++)
		this->model->Shape_Params[i] += delta_thetas(i);

	this->model->Updata(this->model->Params, this->model->Shape_Params);
}

void Worker::save_target_joints()
{
	std::ofstream f;
	f.open(".\\test\\target_joints.txt", std::ios::out);
	for (int i = 0; i < model->NumofJoints; i++) {
		f << model->Joints[i].CorrespondingPosition(0) << "  " 
			<< model->Joints[i].CorrespondingPosition(1) << "  " 
			<< model->Joints[i].CorrespondingPosition(2) << endl;
	}
	f.close();
	printf("Save Target Joints succeed!!!\n");
}
void Worker::load_target_joints()
{
	std::ifstream f;
	f.open(".\\test\\target_joints.txt", std::ios::in);
	Target_joints = Eigen::MatrixXf::Zero(model->NumofJoints, 3);
	for (int i = 0; i < model->NumofJoints; i++) {
		f >> Target_joints(i, 0) >> Target_joints(i, 1) >> Target_joints(i, 2);
	}
	f.close();
	printf("Load Target Joints succeed!!!\n");
}

void Worker::save_target_vertices()
{
	std::ofstream f;
	f.open(".\\test\\target_vertices.txt", std::ios::out);
	for (int i = 0; i < model->NumofVertices; i++) {
		f << model->Vertices_update_(i,0) << "  "
			<< model->Vertices_update_(i, 1) << "  "
			<< model->Vertices_update_(i, 2) << endl;
	}
	f.close();
	printf("Save Target vertices succeed!!!\n");
}
void Worker::load_target_vertices()
{
	std::ifstream f;
	f.open(".\\test\\target_vertices.txt", std::ios::in);
	Target_vertices = Eigen::MatrixXf::Zero(model->NumofVertices, 3);
	for (int i = 0; i < model->NumofVertices; i++) {
		f >> Target_vertices(i, 0) >> Target_vertices(i, 1) >> Target_vertices(i, 2);
	}
	f.close();
	printf("Load Target vertices succeed!!!\n");
}
void Worker::fetch_Input(string fileName, string fileName2)
{
	//从文件中读取下采样后的深度值和距离变换后的idx_img
	std::ifstream f;
	f.open(fileName, std::ios::in);
	int NumofDownSamplePoint;
	f >> NumofDownSamplePoint;
	Downsample_pointcloud.points.resize(NumofDownSamplePoint);
	f >> Downsample_pointcloud_center_x >> Downsample_pointcloud_center_y >> Downsample_pointcloud_center_z;
	for (int i = 0; i < NumofDownSamplePoint; ++i) {
		pcl::PointXYZ p;
		float col_, row_, depth_;
		f >> col_ >> row_ >> depth_;
		Eigen::Vector3f pixel_to_p = model->camera->depth_to_world(row_, col_, depth_);
		p.x = pixel_to_p.x();
		p.y = pixel_to_p.y();
		p.z = pixel_to_p.z();
		Downsample_pointcloud.points[i] = p;
	}
	f.close();
	Downsample_pointcloud_center_x = -Downsample_pointcloud_center_x;
	Downsample_pointcloud_center_z = -Downsample_pointcloud_center_z;
	printf("Load Target DownSamplePoint succeed!!!\n");


	std::ifstream f2;
	f2.open(fileName2, std::ios::in);
	for (int i = 0; i < 424*512; ++i) {
		int t;
		f2 >> t;
		if (t == i) idx_img[i] = 255;
		else idx_img[i] = 0;
	}
	f2.close();
	printf("Load Target idx succeed!!!\n");

}