#include "Worker.h"

string generate_filename(int index)
{
	int str_100, str_10, str_1, tmp;
	tmp = index;
	str_1 = tmp % 10;  tmp /= 10;
	str_10 = tmp % 10; tmp /= 10;
	str_100 = tmp % 10; tmp /= 10;

	return "\\000" + to_string(str_100) + to_string(str_10) + to_string(str_1) + "_depth.bin";
}

Worker::Worker(HandModel * input_model,Camera *input_camera):model(input_model),camera(input_camera){

	E_fitting.init(model);
	E_limits.init(model);
	//E_collision.init(input_model);
	//E_temporal.init(input_model);

	Target_joints = Eigen::MatrixXf::Zero(model->NumofJoints, 3);
	Target_vertices = Eigen::MatrixXf::Zero(model->NumofVertices, 3);
	
	mypointcloud.camera = camera;

	int camera_width = camera->width();
	int camera_height = camera->height();

	input_data_for_track.idxs_img = new int[camera_width*camera_height];
	input_data_for_track.inv_idxs_img = new int[camera_width*camera_height];
	distance_transform.init(camera_width, camera_height);
}





//跟踪函数
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

	for (int i = 0; i < num_joints; ++i)
	{
		this->Target_joints(i, 0) = input_data_for_track.joint_read(i, 0);
		this->Target_joints(i, 1) = input_data_for_track.joint_read(i, 1);
		this->Target_joints(i, 2) = input_data_for_track.joint_read(i, 2);
	}

	///--- Optimization phases	
	LinearSystem system(num_thetas);
	//E_fitting.track(system, input_data_for_track);
	E_fitting.track_Joints(system, this->Target_joints, rigid_only, eval_error, tracking_error.error_3D, tracking_error.error_2D, iter);
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






//这些是以前测试的时候使用的函数，现在应该不用了
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

void Worker::save_DatasetParams()
{
	//string dataset_path = "P:\\数据集\\cvpr15_MSRAHandGestureDB\\cvpr15_MSRAHandGestureDB\\P0\\5";
	string dataset_param_filename = "\\param.txt";

	string filename = dataset_path + dataset_param_filename;

	std::ofstream f;
	f.open(filename, std::ios::out+std::ios::app);
	for (int i = 0; i < num_thetas; i++) {
		f << model->Params[i] << "  ";
	}
	f << endl;
	f.close();
	printf("Save Dataset vertices succeed!!!\n");
}



//读取输入数据
void Worker::fetch_Input(int index)
{
	switch (camera->mode())
	{
	case Kinect:
		fetch_KinectInput(index);
		break;
	case Dataset_MSRA_14:
		fetch_DatasetMSRA_14Inpute(index);
		break;
	case Dataset_MSRA_15:
		fetch_DatasetMSRA_15Inpute(index);
		break;
	default:
		cerr << "Invalid CAMERAMODE , Cann't Fetch Input\n";
		exit(-1);
	}
}

void Worker::fetch_KinectInput(int index)
{
	input_data_for_track.index = index;

	string depth_img = ".\\test\\depth_" + to_string(index) + ".png";
	string handsegment = ".\\test\\Hand_" + to_string(index) + ".jpg";
	string params = ".\\test\\Params_" + to_string(index) + ".txt";


	input_data_for_track.depth = cv::imread(depth_img, CV_LOAD_IMAGE_UNCHANGED);  //这里采用CV_LOAD_IMAGE_UNCHANGED或者CV_LOAD_IMAGE_ANYDEPTH这个模式才可以真确的读入，不然读入都是不正确的，可能和存储的深度值是16位有关系。
	input_data_for_track.Handsegment = cv::imread(handsegment, CV_LOAD_IMAGE_GRAYSCALE);

	cv::flip(input_data_for_track.Handsegment, input_data_for_track.Handsegment, 0);
	cv::bitwise_not(input_data_for_track.Handsegment, input_data_for_track.Handsegment_inv);

	ifstream f;
	f.open(params, ios::in);
	for (int i = 3; i < num_thetas; ++i) f >> input_data_for_track.params[i];
	for (int i = 4; i < num_thetas; ++i) input_data_for_track.params[i] = -input_data_for_track.params[i];
	f.close();

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

	mypointcloud.DepthMatToPointCloud(sensor_indicator, num_sensor_points, input_data_for_track);
	delete[] sensor_indicator;


	//距离变换
	distance_transform.exec(input_data_for_track.Handsegment.data, 125);
	std::copy(distance_transform.idxs_image_ptr(), distance_transform.idxs_image_ptr() + camera->width()*camera->height(), input_data_for_track.idxs_img);

	distance_transform.exec(input_data_for_track.Handsegment_inv.data, 125);
	std::copy(distance_transform.idxs_image_ptr(), distance_transform.idxs_image_ptr() + camera->width()*camera->height(), input_data_for_track.inv_idxs_img);
}
void Worker::fetch_DatasetMSRA_14Inpute(int index)
{
	//string dataset_path = "P:\\数据集\\cvpr14_MSRAHandTrackingDB\\cvpr14_MSRAHandTrackingDB\\Subject1";
	string dataset_joint_filename = "\\joint.txt";
	string dataset_depth_path = dataset_path + generate_filename(index);
	string dataset_joint_path = dataset_path + dataset_joint_filename;

	input_data_for_track.depth = cv::Mat::zeros(camera->height(), camera->width(), CV_16UC1);
	input_data_for_track.Handsegment = cv::Mat::zeros(camera->height(), camera->width(), CV_8UC1);


	//读取深度图
	FILE *pDepthFile = fopen(dataset_depth_path.c_str(), "rb");
	int camera_width = camera->width();
	int camera_height = camera->height();
	float* pDepth = new float[camera_width *camera_height];
	std::ifstream fin(dataset_depth_path, std::ios::binary);
	fin.read((char*)pDepth, sizeof(float)*camera_height *camera_width);

	for (int col = 0; col < camera_width; ++col)
	{
		for (int row = 0; row < camera_height; ++row)
		{
			if (pDepth[row*camera_width + col] != 0)
			{
				input_data_for_track.depth.at<ushort>(row, col) = pDepth[row*camera_width + col];
				input_data_for_track.Handsegment.at<uchar>(row, col) = 255;
			}
		}
	}
	fin.close();

	//将数据集中的左手（至少看起来是左手，变成右手）
	cv::flip(input_data_for_track.depth, input_data_for_track.depth, 1);
	cv::flip(input_data_for_track.Handsegment, input_data_for_track.Handsegment, 1);

	//下面对右手做Kinect Input一样的处理
	cv::flip(input_data_for_track.Handsegment, input_data_for_track.Handsegment, 0);
	cv::bitwise_not(input_data_for_track.Handsegment, input_data_for_track.Handsegment_inv);


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

	mypointcloud.DepthMatToPointCloud(sensor_indicator, num_sensor_points, input_data_for_track);
	delete[] sensor_indicator;


	//距离变换
	distance_transform.exec(input_data_for_track.Handsegment.data, 125);
	std::copy(distance_transform.idxs_image_ptr(), distance_transform.idxs_image_ptr() + camera->width()*camera->height(), input_data_for_track.idxs_img);

	distance_transform.exec(input_data_for_track.Handsegment_inv.data, 125);
	std::copy(distance_transform.idxs_image_ptr(), distance_transform.idxs_image_ptr() + camera->width()*camera->height(), input_data_for_track.inv_idxs_img);


	//读取关节点
	ifstream f_joint;
	f_joint.open(dataset_joint_path, ios::in);


	f_joint >> fileAmount;

	input_data_for_track.joint_read.setZero();

	for (int i = 0; i <= index; ++i)
	{
		{
			f_joint >> input_data_for_track.joint_read(0, 0)
				>> input_data_for_track.joint_read(0, 1)
				>> input_data_for_track.joint_read(0, 2);

			input_data_for_track.joint_read(0, 0) = -input_data_for_track.joint_read(0, 0);
			input_data_for_track.joint_read(0, 2) = -input_data_for_track.joint_read(0, 2);
		}

		for (int j = 5; j < 21; ++j)
		{
			f_joint >> input_data_for_track.joint_read(j, 0)
				>> input_data_for_track.joint_read(j, 1)
				>> input_data_for_track.joint_read(j, 2);

			input_data_for_track.joint_read(j, 0) = -input_data_for_track.joint_read(j, 0);
			input_data_for_track.joint_read(j, 2) = -input_data_for_track.joint_read(j, 2);
		}

		{
			f_joint >> input_data_for_track.joint_read(1, 0)
				>> input_data_for_track.joint_read(1, 1)
				>> input_data_for_track.joint_read(1, 2);

			input_data_for_track.joint_read(1, 0) = -input_data_for_track.joint_read(1, 0);
			input_data_for_track.joint_read(1, 2) = -input_data_for_track.joint_read(1, 2);
		}

		{
			f_joint >> input_data_for_track.joint_read(2, 0)
				>> input_data_for_track.joint_read(2, 1)
				>> input_data_for_track.joint_read(2, 2);

			input_data_for_track.joint_read(2, 0) = -input_data_for_track.joint_read(2, 0);
			input_data_for_track.joint_read(2, 2) = -input_data_for_track.joint_read(2, 2);
		}

		{
			f_joint >> input_data_for_track.joint_read(3, 0)
				>> input_data_for_track.joint_read(3, 1)
				>> input_data_for_track.joint_read(3, 2);

			input_data_for_track.joint_read(3, 0) = -input_data_for_track.joint_read(3, 0);
			input_data_for_track.joint_read(3, 2) = -input_data_for_track.joint_read(3, 2);
		}

		{
			f_joint >> input_data_for_track.joint_read(4, 0)
				>> input_data_for_track.joint_read(4, 1)
				>> input_data_for_track.joint_read(4, 2);

			input_data_for_track.joint_read(4, 0) = -input_data_for_track.joint_read(4, 0);
			input_data_for_track.joint_read(4, 2) = -input_data_for_track.joint_read(4, 2);
		}
	}

	f_joint.close();

	//读取关节角度（这个关节角度需要自己转化，还没做）


}
void Worker::fetch_DatasetMSRA_15Inpute(int index)
{
	input_data_for_track.index = index;
	//string dataset_path = "P:\\数据集\\cvpr15_MSRAHandGestureDB\\cvpr15_MSRAHandGestureDB\\P0\\5";
	string dataset_joint_filename = "\\joint.txt";
	string dataset_depth_path = dataset_path + generate_filename(index);
	string dataset_joint_path = dataset_path + dataset_joint_filename;

	input_data_for_track.depth = cv::Mat::zeros(camera->height(), camera->width(), CV_16UC1);
	input_data_for_track.Handsegment = cv::Mat::zeros(camera->height(), camera->width(), CV_8UC1);

	//读取深度图
	FILE *pDepthFile = fopen(dataset_depth_path.c_str(), "rb");

	int img_width, img_height;
	int left, right, top, bottom;

	fread(&img_width, sizeof(int), 1, pDepthFile);
	fread(&img_height, sizeof(int), 1, pDepthFile);

	fread(&left, sizeof(int), 1, pDepthFile);
	fread(&top, sizeof(int), 1, pDepthFile);
	fread(&right, sizeof(int), 1, pDepthFile);
	fread(&bottom, sizeof(int), 1, pDepthFile);

	int bounding_box_width = right - left;
	int bounding_box_height = bottom - top;

	int cur_pixel_num = bounding_box_width * bounding_box_height;
	float* pDepth = new float[cur_pixel_num];
	fread(pDepth, sizeof(float), cur_pixel_num, pDepthFile);
	fclose(pDepthFile);

	for (int row = top; row < bottom; ++row)
	{
		for (int col = left; col < right; ++col)
		{
			if (pDepth[(row - top)*bounding_box_width + (col - left)] != 0)
			{
				input_data_for_track.depth.at<ushort>(row, col) = pDepth[(row - top)*bounding_box_width + (col - left)];
				input_data_for_track.Handsegment.at<uchar>(row, col) = 255;
			}
			
		}
	}

	//将数据集中的左手（至少看起来是左手，变成右手）
	cv::flip(input_data_for_track.depth, input_data_for_track.depth, 1);
	cv::flip(input_data_for_track.Handsegment, input_data_for_track.Handsegment, 1);

	//下面对右手做Kinect Input一样的处理
	cv::flip(input_data_for_track.Handsegment, input_data_for_track.Handsegment, 0);
	cv::bitwise_not(input_data_for_track.Handsegment, input_data_for_track.Handsegment_inv);


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

	mypointcloud.DepthMatToPointCloud(sensor_indicator, num_sensor_points, input_data_for_track);
	delete[] sensor_indicator;


	//距离变换
	distance_transform.exec(input_data_for_track.Handsegment.data, 125);
	std::copy(distance_transform.idxs_image_ptr(), distance_transform.idxs_image_ptr() + camera->width()*camera->height(), input_data_for_track.idxs_img);

	distance_transform.exec(input_data_for_track.Handsegment_inv.data, 125);
	std::copy(distance_transform.idxs_image_ptr(), distance_transform.idxs_image_ptr() + camera->width()*camera->height(), input_data_for_track.inv_idxs_img);


	//读取关节点
	ifstream f_joint;
	f_joint.open(dataset_joint_path, ios::in);


	f_joint >> fileAmount;

	input_data_for_track.joint_read.setZero();

	for (int i = 0; i <= index; ++i)
	{
		{
			f_joint >> input_data_for_track.joint_read(0, 0)
				>> input_data_for_track.joint_read(0, 1)
				>> input_data_for_track.joint_read(0, 2);

			input_data_for_track.joint_read(0, 0) = -input_data_for_track.joint_read(0, 0);
			input_data_for_track.joint_read(0, 2) = -input_data_for_track.joint_read(0, 2);
		}

		for (int j = 5; j < 21; ++j)
		{
			f_joint >> input_data_for_track.joint_read(j, 0)
				>> input_data_for_track.joint_read(j, 1)
				>> input_data_for_track.joint_read(j, 2);

			input_data_for_track.joint_read(j, 0) = -input_data_for_track.joint_read(j, 0);
			input_data_for_track.joint_read(j, 2) = -input_data_for_track.joint_read(j, 2);
		}

		{
			f_joint >> input_data_for_track.joint_read(1, 0)
				>> input_data_for_track.joint_read(1, 1)
				>> input_data_for_track.joint_read(1, 2);

			input_data_for_track.joint_read(1, 0) = -input_data_for_track.joint_read(1, 0);
			input_data_for_track.joint_read(1, 2) = -input_data_for_track.joint_read(1, 2);
		}

		{
			f_joint >> input_data_for_track.joint_read(2, 0)
				>> input_data_for_track.joint_read(2, 1)
				>> input_data_for_track.joint_read(2, 2);

			input_data_for_track.joint_read(2, 0) = -input_data_for_track.joint_read(2, 0);
			input_data_for_track.joint_read(2, 2) = -input_data_for_track.joint_read(2, 2);
		}

		{
			f_joint >> input_data_for_track.joint_read(3, 0)
				>> input_data_for_track.joint_read(3, 1)
				>> input_data_for_track.joint_read(3, 2);

			input_data_for_track.joint_read(3, 0) = -input_data_for_track.joint_read(3, 0);
			input_data_for_track.joint_read(3, 2) = -input_data_for_track.joint_read(3, 2);
		}

		{
			f_joint >> input_data_for_track.joint_read(4, 0)
				>> input_data_for_track.joint_read(4, 1)
				>> input_data_for_track.joint_read(4, 2);

			input_data_for_track.joint_read(4, 0) = -input_data_for_track.joint_read(4, 0);
			input_data_for_track.joint_read(4, 2) = -input_data_for_track.joint_read(4, 2);
		}
	}

	f_joint.close();

	//读取关节角度（这个关节角度需要自己转化，还没做）

}


//void Worker::fetch_Input(int index)
//{
//	DatasetInput = false;
//	string depth_img = ".\\test\\depth_" + to_string(index) + ".png";
//	string handsegment = ".\\test\\Hand_" + to_string(index) + ".jpg";
//	string params = ".\\test\\Params_" + to_string(index) + ".txt";
//	//string joint_true = ".\\test\\new_joints_" + to_string(index) + ".txt";
//
//	input_data_for_track.depth_Kinect = cv::imread(depth_img, CV_LOAD_IMAGE_UNCHANGED);  //这里采用CV_LOAD_IMAGE_UNCHANGED或者CV_LOAD_IMAGE_ANYDEPTH这个模式才可以真确的读入，不然读入都是不正确的，可能和存储的深度值是16位有关系。
//	input_data_for_track.Handsegment = cv::imread(handsegment, CV_LOAD_IMAGE_GRAYSCALE);
//
//	cv::flip(input_data_for_track.Handsegment, input_data_for_track.Handsegment, 0);
//	cv::bitwise_not(input_data_for_track.Handsegment, input_data_for_track.Handsegment_inv);
//
//	ifstream f;
//	f.open(params, ios::in);
//	for (int i = 3; i < num_thetas; ++i) f >> input_data_for_track.params[i];
//	for (int i = 4; i < num_thetas; ++i) input_data_for_track.params[i] = -input_data_for_track.params[i];
//	f.close();
//
//	int count = 0;
//	int *sensor_indicator = new int[424 * 512];
//	int num_sensor_points = 0;
//	for (int row = 0; row < input_data_for_track.Handsegment.rows; ++row) {
//		for (int col = 0; col < input_data_for_track.Handsegment.cols; ++col) {
//			if (input_data_for_track.Handsegment.at<uchar>(row, col) != 255) continue;
//			if (count % 2 == 0) {
//				sensor_indicator[num_sensor_points++] = row * input_data_for_track.Handsegment.cols + col;
//			}
//			count++;
//		}
//	}
//
//	distance_transform.exec(input_data_for_track.Handsegment.data, 125);
//	std::copy(distance_transform.idxs_image_ptr(), distance_transform.idxs_image_ptr() + 424 * 512, input_data_for_track.idxs_img);
//
//	distance_transform.exec(input_data_for_track.Handsegment_inv.data, 125);
//	std::copy(distance_transform.idxs_image_ptr(), distance_transform.idxs_image_ptr() + 424 * 512, input_data_for_track.inv_idxs_img);
//
//	mypointcloud.DepthMatToPointCloud(sensor_indicator, num_sensor_points, input_data_for_track);
//	delete[] sensor_indicator;
//
//
//}
//void Worker::fetch_DatasetInput(int index)
//{
//	DatasetInput = true;
//	Dataset_index = index;
//	string depth_img = ".\\test\\new_depth_" + to_string(index) + ".png";
//	string handsegment = ".\\test\\new_Hand_" + to_string(index) + ".png";
//	string joint_true = ".\\test\\new_joints_" + to_string(index) + ".txt";
//	string params = ".\\test\\new_Params_" + to_string(index) + ".txt";
//
//	input_data_for_track.depth_Kinect = cv::imread(depth_img, CV_LOAD_IMAGE_UNCHANGED);  //这里采用CV_LOAD_IMAGE_UNCHANGED或者CV_LOAD_IMAGE_ANYDEPTH这个模式才可以真确的读入，不然读入都是不正确的，可能和存储的深度值是16位有关系。
//	input_data_for_track.Handsegment = cv::imread(handsegment, CV_LOAD_IMAGE_GRAYSCALE);
//
//	cv::flip(input_data_for_track.Handsegment, input_data_for_track.Handsegment, 0);
//	cv::bitwise_not(input_data_for_track.Handsegment, input_data_for_track.Handsegment_inv);
//
//	ifstream f;
//	f.open(params, ios::in);
//	if (f.is_open())
//	{
//		for (int i = 0; i < num_thetas; ++i) f >> input_data_for_track.params[i];
//		f.close();
//	}
//
//
//	input_data_for_track.joint_read.setZero();
//	ifstream f_joint;
//	f_joint.open(joint_true, ios::in);
//	for (int i = 0; i < 21; ++i)
//	{
//		f_joint >> input_data_for_track.joint_read(i, 0) >> input_data_for_track.joint_read(i, 1) >> input_data_for_track.joint_read(i, 2);
//	}
//	f_joint.close();
//
//
//	int count = 0;
//	int *sensor_indicator = new int[424 * 512];
//	int num_sensor_points = 0;
//	for (int row = 0; row < input_data_for_track.Handsegment.rows; ++row) {
//		for (int col = 0; col < input_data_for_track.Handsegment.cols; ++col) {
//			if (input_data_for_track.Handsegment.at<uchar>(row, col) != 255) continue;
//			if (count % 2 == 0) {
//				sensor_indicator[num_sensor_points++] = row * input_data_for_track.Handsegment.cols + col;
//			}
//			count++;
//		}
//	}
//
//
//	//vector<vector<cv::Point> > contours;
//	//cv::findContours(input_data_for_track.Handsegment, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); //找轮廓
//	////outputImage.setTo(0);
//	//cv::drawContours(input_data_for_track.Handsegment, contours, -1, cv::Scalar(255), CV_FILLED); //在遮罩图层上，用白色像素填充轮廓，得到MASK
//
//
//	distance_transform.exec(input_data_for_track.Handsegment.data, 125);
//	std::copy(distance_transform.idxs_image_ptr(), distance_transform.idxs_image_ptr() + 424 * 512, input_data_for_track.idxs_img);
//
//	distance_transform.exec(input_data_for_track.Handsegment_inv.data, 125);
//	std::copy(distance_transform.idxs_image_ptr(), distance_transform.idxs_image_ptr() + 424 * 512, input_data_for_track.inv_idxs_img);
//
//	mypointcloud.DepthMatToPointCloud(sensor_indicator, num_sensor_points, input_data_for_track);
//	delete[] sensor_indicator;
//
//
//	this->Target_joints.row(0) = this->input_data_for_track.joint_read.row(0);
//	for (int i = 5; i < model->NumofJoints; ++i)
//	{
//		this->Target_joints.row(i) = this->input_data_for_track.joint_read.row(i - 4);
//	}
//	this->Target_joints.row(1) = this->input_data_for_track.joint_read.row(17);
//	this->Target_joints.row(2) = this->input_data_for_track.joint_read.row(18);
//	this->Target_joints.row(3) = this->input_data_for_track.joint_read.row(19);
//	this->Target_joints.row(4) = this->input_data_for_track.joint_read.row(20);
//
//}