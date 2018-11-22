#pragma once
#include <pcl/kdtree/kdtree_flann.h>      //pcl库的flann 和 opencv的flann的库有冲突，解决方法：https://blog.csdn.net/u011418173/article/details/52614617
#include"Energy.h"
#include"HandModel.h"

namespace energy {
	class Energy_Fitting : public Energy {
	public:
		struct Settings {
			///--- E_2D
			bool  fit2D_enable = true;
			float fit2D_weight = 0.4f;

			///--- E_3D
			bool  fit3D_enable = true;
			float fit3D_weight = 1.0f;
		}_settings;
		Settings*const settings = &_settings;

		HandModel *model;
		pcl::PointCloud<pcl::PointXYZ> Handmodel_visible_cloud;
		std::vector<int> cloud_correspond;

		void init(HandModel *handmodel);
		void track_Joints(LinearSystem &sys, 
			Matrix_Nx3 &Target_joints,
			bool rigid_only, 
			bool eval_error, 
			float &error_3D, 
			float &error_2D, 
			int iter);

		void load_handmodel_visible_cloud();
		void find_correspondences(pcl::PointCloud<pcl::PointXYZ>& Downsample_cloud);

		void track_3D(LinearSystem &sys, pcl::PointCloud<pcl::PointXYZ>& Downsample_cloud);
		void track_2D(LinearSystem &sys, int *idx_img);
		void track_2D_using_Silhouette(LinearSystem &sys, int *idx_img);
		void track(LinearSystem &sys, InputDataForTrack& inputdata);


		void track_Shape_Joints(LinearSystem &sys,
			Matrix_Nx3 &Target_joints,
			bool rigid_only,
			bool eval_error,
			float &error_3D,
			float &error_2D,
			int iter);
		void track_Shape(LinearSystem &sys, Matrix_Nx3 &Target_vertices, int iter);

		void track_Shape_2_3D(LinearSystem &sys, pcl::PointCloud<pcl::PointXYZ>& Downsample_cloud);
		void track_Shape_2_2D(LinearSystem &sys, int *idx_img);
		void track_Shape_2_2D_uing_Silhouette(LinearSystem &sys, int *idx_img);
		void track_Shape_2(LinearSystem &sys, InputDataForTrack& inputdata);
	};
} /// energy::
