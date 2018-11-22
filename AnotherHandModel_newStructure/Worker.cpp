#include "Worker.h"

Worker::Worker(HandModel * input_model) {
	this->model = input_model;
	E_fitting.init(input_model);
	E_limits.init(input_model);
	//E_collision.init(input_model);
	//E_temporal.init(input_model);

	Target_joints = Eigen::MatrixXf::Zero(model->NumofJoints, 3);
	Target_vertices = Eigen::MatrixXf::Zero(model->NumofVertices, 3);
	
	mypointcloud.camera = input_model->camera;
	distance_transform.init(512, 424);
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
	E_fitting.track(system, input_data_for_track);
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
	E_fitting.track_Shape_2(system, input_data_for_track);
	//E_fitting.track_Shape(system, Target_vertices, iter);
	//E_fitting.track_Shape_Joints(system, Target_joints, rigid_only, eval_error, tracking_error.error_3D, tracking_error.error_2D, iter);
	//E_collision.track(system);
	//E_temporal.track(system);
	E_limits.track_shape(system);
	E_damping.track_shape(system);
	/*if (rigid_only)
	energy::Energy::rigid_only(system);*/

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

void Worker::fetch_Input(int index)
{
	string depth_img = ".\\test\\depth_" + to_string(index) + ".png";
	string handsegment = ".\\test\\Hand_" + to_string(index) + ".jpg";
	string params = ".\\test\\Params_" + to_string(index) + ".txt";

	input_data_for_track.depth_Kinect = cv::imread(depth_img, CV_LOAD_IMAGE_UNCHANGED);  //这里采用CV_LOAD_IMAGE_UNCHANGED或者CV_LOAD_IMAGE_ANYDEPTH这个模式才可以真确的读入，不然读入都是不正确的，可能和存储的深度值是16位有关系。
	input_data_for_track.Handsegment = cv::imread(handsegment, CV_LOAD_IMAGE_GRAYSCALE);

	cv::flip(input_data_for_track.Handsegment, input_data_for_track.Handsegment, 0);
	cv::bitwise_not(input_data_for_track.Handsegment, input_data_for_track.Handsegment_inv);

	ifstream f;
	f.open(params, ios::in);
	for (int i = 3; i < num_thetas; ++i) f >> input_data_for_track.params[i];
	for (int i = 4; i < num_thetas; ++i) input_data_for_track.params[i] = -input_data_for_track.params[i];


	int count = 0;
	int *sensor_indicator = new int[424 * 512];
	int num_sensor_points = 0;
	for (int row = 0; row < input_data_for_track.Handsegment.rows; ++row) {
		for (int col = 0; col < input_data_for_track.Handsegment.cols; ++col) {
			if (input_data_for_track.Handsegment.at<uchar>(row, col) != 255) continue;
			if (count % 2 == 0) {
				sensor_indicator[num_sensor_points++] = row * input_data_for_track.Handsegment.cols + col;
			}
			count++;
		}
	}

	distance_transform.exec(input_data_for_track.Handsegment.data, 125);
	std::copy(distance_transform.idxs_image_ptr(), distance_transform.idxs_image_ptr() + 424 * 512, input_data_for_track.idxs_img);

	distance_transform.exec(input_data_for_track.Handsegment_inv.data, 125);
	std::copy(distance_transform.idxs_image_ptr(), distance_transform.idxs_image_ptr() + 424 * 512, input_data_for_track.inv_idxs_img);

	mypointcloud.DepthMatToPointCloud(sensor_indicator, num_sensor_points, input_data_for_track);
	delete[] sensor_indicator;
}