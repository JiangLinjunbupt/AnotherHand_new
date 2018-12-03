#include"MyPointCloud.h"


void MyPointCloud::DepthMatToPointCloud(int *indicator, int Num_indicator, InputDataForTrack& inputdatafortrack)
{
	int camera_width = camera->width();
	int camera_height = camera->height();

	inputdatafortrack.pointcloud_from_depth.clear();
	inputdatafortrack.params[0] = 0; inputdatafortrack.params[1] = 0; inputdatafortrack.params[2] = 0;
	cv::Mat depth_flip;
	cv::flip(inputdatafortrack.depth, depth_flip, 0);

	for (int i = 0; i < Num_indicator; i++)
	{
		pcl::PointXYZ p;

		int col = indicator[i] % camera_width;        //x
		int row = indicator[i] / camera_width;        //y

		int z = depth_flip.at<unsigned short>(row, col);
		Eigen::Vector3f p_pixel = camera->depth_to_world(row, col, z);

		p.x = p_pixel.x();
		p.y = p_pixel.y();
		p.z = p_pixel.z();

		inputdatafortrack.params[0] += p_pixel.x();
		inputdatafortrack.params[1] += p_pixel.y();
		inputdatafortrack.params[2] += p_pixel.z();

		inputdatafortrack.pointcloud_from_depth.push_back(p);
	}

	if (inputdatafortrack.pointcloud_from_depth.points.size() != 0)
	{
		inputdatafortrack.params[0] = inputdatafortrack.params[0] / (float)(inputdatafortrack.pointcloud_from_depth.points.size());
		inputdatafortrack.params[1] = inputdatafortrack.params[1] / (float)(inputdatafortrack.pointcloud_from_depth.points.size());
		inputdatafortrack.params[2] = inputdatafortrack.params[2] / (float)(inputdatafortrack.pointcloud_from_depth.points.size());
	}

	RestrictGlobalPosition(inputdatafortrack, 10000);
	Filter_visible_cloud(inputdatafortrack);
	downSample(inputdatafortrack);
}


void MyPointCloud::Filter_visible_cloud(InputDataForTrack& inputdatafortrack)
{
	if (inputdatafortrack.pointcloud_from_depth.points.size() > 0)
	{
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(inputdatafortrack.pointcloud_from_depth.makeShared());
		sor.setMeanK(100);
		sor.setStddevMulThresh(1.0);
		sor.filter(inputdatafortrack.pointcloud_filtered);
	}
}

void MyPointCloud::downSample(InputDataForTrack& inputdatafortrack)
{
	if (inputdatafortrack.pointcloud_filtered.points.size())
	{
		pcl::VoxelGrid<pcl::PointXYZ> sor;  //体素栅格下采样对象
		sor.setInputCloud(inputdatafortrack.pointcloud_filtered.makeShared());             //原始点云
		sor.setLeafSize(8.0f, 8.0f, 8.0f);    // 设置采样体素大小
		sor.filter(inputdatafortrack.pointcloud_downsample);        //保存

	//std::cout << "after filter ,the point cloud size is : " << after_cloud.points.size() << std::endl;
	}
}

void MyPointCloud::RestrictGlobalPosition(InputDataForTrack& inputdatafortrack,int max)
{
	if (inputdatafortrack.params[0] > max) inputdatafortrack.params[0] = max;
	if (inputdatafortrack.params[0] < -max) inputdatafortrack.params[0] = -max;

	if (inputdatafortrack.params[1] > max) inputdatafortrack.params[1] = max;
	if (inputdatafortrack.params[1] < -max) inputdatafortrack.params[1] = -max;

	if (inputdatafortrack.params[2] > max) inputdatafortrack.params[2] = max;
	if (inputdatafortrack.params[2] < -max) inputdatafortrack.params[2] = -max;
}