#include"Energy_Fitting.h"


namespace energy
{
	void Energy_Fitting::init(HandModel* handmodel)
	{
		this->model = handmodel;
		Handmodel_visible_cloud.points.clear();
		cloud_correspond.clear();
	}

	void Energy_Fitting::track_Joints(LinearSystem &sys,
		Matrix_Nx3 &Target_joints,
		bool rigid_only,
		bool eval_error,
		float &error_3D,
		float &error_2D,
		int iter)
	{
		int NumofJoints = model->NumofJoints;
		int NumberofParams = model->NumberofParams;

		Eigen::VectorXf e = Eigen::VectorXf::Zero(3 * NumofJoints, 1);
		for (int i = 0; i < NumofJoints; i++)
		{
			e(i * 3 + 0) = Target_joints(i, 0) - model->Joint_matrix(i, 0);
			e(i * 3 + 1) = Target_joints(i, 1) - model->Joint_matrix(i, 1);
			e(i * 3 + 2) = Target_joints(i, 2) - model->Joint_matrix(i, 2);
		}

		Eigen::MatrixXf Joints_jacobian = Eigen::MatrixXf::Zero(3 * NumofJoints, NumberofParams);

		for (int i = 0; i < NumofJoints; ++i)
		{
			Joints_jacobian.block(i * 3, 0, 3, NumberofParams) = model->Compute_one_Joint_Jacobian(i);
		}

		sys.Jt_J.noalias() += Joints_jacobian.transpose() * Joints_jacobian;
		sys.Jt_e.noalias() += Joints_jacobian.transpose() * e;

		///--- Check
		if (Energy::safety_check)
			Energy::has_nan(sys);
	}

	void Energy_Fitting::load_handmodel_visible_cloud()
	{
		int Num_visible_vertices = static_cast<int>(this->model->Visible_vertices.size());
		Handmodel_visible_cloud.points.resize(Num_visible_vertices);
		if (Num_visible_vertices> 0)
		{
			for (int i = 0; i < Num_visible_vertices; i++)
			{
				pcl::PointXYZ p;
				p.x = this->model->Visible_vertices[i](0);
				p.y = this->model->Visible_vertices[i](1);
				p.z = this->model->Visible_vertices[i](2);
				Handmodel_visible_cloud.points[i] = p;
			}
		}
		else
		{
			cerr << "hand model have no Visible_vertices!!!!  please cheak if some thing wrong ? " << endl;
		}
	}

	void Energy_Fitting::find_correspondences(pcl::PointCloud<pcl::PointXYZ>& Downsample_cloud)
	{
		int Num_visible_cloud = static_cast<int>(Handmodel_visible_cloud.points.size());
		if (Num_visible_cloud > 0)
		{
			cloud_correspond.resize(Downsample_cloud.points.size());

			pcl::KdTreeFLANN<pcl::PointXYZ> search_kdtree;
			search_kdtree.setInputCloud(Handmodel_visible_cloud.makeShared());

			const int k = 1;
			std::vector<int> k_indices(k);
			std::vector<float> k_squared_distances(k);
			for (int i = 0; i < Downsample_cloud.points.size(); ++i)
			{
				search_kdtree.nearestKSearch(Downsample_cloud, i, k, k_indices, k_squared_distances);
				cloud_correspond[i] = k_indices[0];
			}
		}
		else
		{
			cerr << "hand model have no Visible_vertices!!!!  please cheak if some thing wrong ? " << endl;
		}
	}


	void Energy_Fitting::track_3D(LinearSystem &sys, pcl::PointCloud<pcl::PointXYZ>& Downsample_cloud)
	{
		load_handmodel_visible_cloud();
		find_correspondences(Downsample_cloud);

		int NumofCorrespond = static_cast<int>(Downsample_cloud.points.size());
		Eigen::VectorXf e = Eigen::VectorXf::Zero(3 * NumofCorrespond, 1);
		Eigen::MatrixXf jacobian_correspond = Eigen::MatrixXf::Zero(NumofCorrespond * 3, model->NumberofParams);

		for (int i = 0; i < NumofCorrespond; i++)
		{
			float length = sqrt((Downsample_cloud.points[i].x - model->Visible_vertices[cloud_correspond[i]].x())*(Downsample_cloud.points[i].x - model->Visible_vertices[cloud_correspond[i]].x())
				+ (Downsample_cloud.points[i].y - model->Visible_vertices[cloud_correspond[i]].y())*(Downsample_cloud.points[i].y - model->Visible_vertices[cloud_correspond[i]].y())
				+ (Downsample_cloud.points[i].z - model->Visible_vertices[cloud_correspond[i]].z())*(Downsample_cloud.points[i].z - model->Visible_vertices[cloud_correspond[i]].z()));

			float reweight = 1.0f / sqrt(length + 1e-3);

			e(i * 3 + 0) = reweight*(Downsample_cloud.points[i].x - model->Visible_vertices[cloud_correspond[i]].x());
			e(i * 3 + 1) = reweight*(Downsample_cloud.points[i].y - model->Visible_vertices[cloud_correspond[i]].y());
			e(i * 3 + 2) = reweight*(Downsample_cloud.points[i].z - model->Visible_vertices[cloud_correspond[i]].z());


			int vert_idx = model->Visible_vertices_index[cloud_correspond[i]];
			jacobian_correspond.block(i * 3, 0, 3, model->NumberofParams) = reweight*model->Compute_one_Vertice_Jacobian(vert_idx);
		}


		//for (int cor_id = 0; cor_id < NumofCorrespond; ++cor_id)
		//{
		//	int vert_idx = model->Visible_vertices_index[cloud_correspond[cor_id]];
		//	jacobian_correspond.block(cor_id * 3, 0, 3, model->NumberofParams) = model->Compute_one_Vertice_Jacobian(vert_idx);
		//}

		float omiga = settings->fit3D_weight;
		sys.Jt_J.noalias() += omiga*jacobian_correspond.transpose() * jacobian_correspond;
		sys.Jt_e.noalias() += omiga*jacobian_correspond.transpose() * e;

		///--- Check
		if (Energy::safety_check)
			Energy::has_nan(sys);
	}

	void Energy_Fitting::track_2D(LinearSystem &sys, int *idx_img)
	{
		vector<pair<Eigen::Matrix<float, 2, 26>, Eigen::Vector2f>> outside_silhouette;

		for (int i = 0; i < model->Visible_vertices_2D.size(); i++)
		{
			Eigen::Vector3f pixel_3D_position(model->Visible_vertices[i]);
			int vert_idx = model->Visible_vertices_index[i];
			Eigen::Vector2i pixel_2D_position(model->Visible_vertices_2D[i]);

			if ((pixel_2D_position(0) >= 0) &&
				(pixel_2D_position(0) <= 512 - 1) &&
				(pixel_2D_position(1) >= 0) &&
				(pixel_2D_position(1) <= 424 - 1))
			{
				Eigen::Vector2i pixel_2D_closest;
				pixel_2D_closest << idx_img[pixel_2D_position(1) * 512 + pixel_2D_position(0)] % 512, idx_img[pixel_2D_position(1) * 512 + pixel_2D_position(0)] / 512;

				float closest_distance = (pixel_2D_closest(0) - pixel_2D_position(0))* (pixel_2D_closest(0) - pixel_2D_position(0)) + (pixel_2D_closest(1) - pixel_2D_position(1))*(pixel_2D_closest(1) - pixel_2D_position(1));

				if (closest_distance > 1)
				{
					//����J��e
					pair<Eigen::Matrix2Xf, Eigen::Vector2f> J_and_e;

					//����e
					Eigen::Vector2f e;
					e(0) = (float)pixel_2D_closest(0) - (float)pixel_2D_position(0);
					e(1) = (float)pixel_2D_closest(1) - (float)pixel_2D_position(1);

					J_and_e.second = e;

					//����J
					Matrix_2x3 J_perspective = model->camera->projection_jacobian(pixel_3D_position);
					Eigen::Matrix<float, 2, 26> J_2D = Eigen::MatrixXf::Zero(2, 26);    //J_2D = J_perspective * J_3D

					J_2D = J_perspective*model->Compute_one_Vertice_Jacobian(vert_idx);

					J_and_e.first = J_2D;

					outside_silhouette.push_back(J_and_e);
				}
			}
		}

		int size = static_cast<int>(outside_silhouette.size());

		if (size > 0)
		{
			Eigen::MatrixXf J_sil = Eigen::MatrixXf::Zero(2 * size, 26);

			Eigen::VectorXf e_sil = Eigen::VectorXf::Zero(2 * size, 1);

			for (int i = 0; i < size; i++)
			{
				J_sil.row(i * 2 + 0) = outside_silhouette[i].first.row(0);
				J_sil.row(i * 2 + 1) = outside_silhouette[i].first.row(1);

				e_sil.row(i * 2 + 0) = outside_silhouette[i].second.row(0);
				e_sil.row(i * 2 + 1) = outside_silhouette[i].second.row(1);
			}

			float omiga = settings->fit2D_weight;
			sys.Jt_J.noalias() += omiga*J_sil.transpose() * J_sil;
			sys.Jt_e.noalias() += omiga*J_sil.transpose() * e_sil;

			///--- Check
			if (Energy::safety_check)
				Energy::has_nan(sys);
		}
	}

	void Energy_Fitting::track(LinearSystem &sys, pcl::PointCloud<pcl::PointXYZ>& Downsample_cloud, int *idx_img)
	{
		if(settings->fit3D_enable) this->track_3D(sys, Downsample_cloud);
		if(settings->fit2D_enable) this->track_2D(sys, idx_img);
	}

	void Energy_Fitting::track_Shape_Joints(LinearSystem &sys,
		Matrix_Nx3 &Target_joints,
		bool rigid_only,
		bool eval_error,
		float &error_3D,
		float &error_2D,
		int iter)
	{
		int NumofJoints = model->NumofJoints;
		int NumberofShapeParams = model->NumofShape_Params;

		Eigen::VectorXf e = Eigen::VectorXf::Zero(3 * NumofJoints, 1);
		for (int i = 0; i < NumofJoints; i++)
		{
			e(i * 3 + 0) = Target_joints(i, 0) - model->Joint_matrix(i, 0);
			e(i * 3 + 1) = Target_joints(i, 1) - model->Joint_matrix(i, 1);
			e(i * 3 + 2) = Target_joints(i, 2) - model->Joint_matrix(i, 2);
		}

		cout << "Index MCP difference is ��  " << e(5 * 3 + 0) << "   " << e(5 * 3 + 1) << "    " << e(5 * 3 + 2) << endl;

		Eigen::MatrixXf Joints_jacobian = Eigen::MatrixXf::Zero(3 * NumofJoints, NumberofShapeParams);

		for (int i = 0; i < NumofJoints; ++i)
		{
			Joints_jacobian.block(i * 3, 0, 3, NumberofShapeParams) = model->Compute_one_Joint_Shape_Jacobian(i);
		}

		sys.Jt_J.noalias() += Joints_jacobian.transpose() * Joints_jacobian;
		sys.Jt_e.noalias() += Joints_jacobian.transpose() * e;

		///--- Check
		if (Energy::safety_check)
			Energy::has_nan(sys);
	}


	void Energy_Fitting::track_Shape(LinearSystem &sys, Matrix_Nx3 &Target_vertices, int iter)
	{
		int NumofVertices = model->NumofVertices;
		int NumberofShapeParams = model->NumofShape_Params;

		Eigen::VectorXf e = Eigen::VectorXf::Zero(3 * NumofVertices, 1);
		for (int i = 0; i < NumofVertices; i++)
		{
			e(i * 3 + 0) = Target_vertices(i, 0) - model->Vertices_update_(i, 0);
			e(i * 3 + 1) = Target_vertices(i, 1) - model->Vertices_update_(i, 1);
			e(i * 3 + 2) = Target_vertices(i, 2) - model->Vertices_update_(i, 2);
		}

		Eigen::MatrixXf Joints_jacobian = Eigen::MatrixXf::Zero(3 * NumofVertices, NumberofShapeParams);

		for (int i = 0; i < NumofVertices; ++i)
		{
			Joints_jacobian.block(i * 3, 0, 3, NumberofShapeParams) = model->Compute_one_Vertice_Shape_Jacobian(i);
		}

		sys.Jt_J.noalias() += Joints_jacobian.transpose() * Joints_jacobian;
		sys.Jt_e.noalias() += Joints_jacobian.transpose() * e;

		///--- Check
		if (Energy::safety_check)
			Energy::has_nan(sys);
	}

	void Energy_Fitting::track_Shape_2(LinearSystem &sys, pcl::PointCloud<pcl::PointXYZ>& Downsample_cloud)
	{
		load_handmodel_visible_cloud();
		find_correspondences(Downsample_cloud);

		int NumofCorrespond = static_cast<int>(Downsample_cloud.points.size());
		Eigen::VectorXf e = Eigen::VectorXf::Zero(3 * NumofCorrespond, 1);
		Eigen::MatrixXf jacobian_correspond = Eigen::MatrixXf::Zero(NumofCorrespond * 3, model->NumberofParams);

		for (int i = 0; i < NumofCorrespond; i++)
		{
			float length = sqrt((Downsample_cloud.points[i].x - model->Visible_vertices[cloud_correspond[i]].x())*(Downsample_cloud.points[i].x - model->Visible_vertices[cloud_correspond[i]].x())
				+ (Downsample_cloud.points[i].y - model->Visible_vertices[cloud_correspond[i]].y())*(Downsample_cloud.points[i].y - model->Visible_vertices[cloud_correspond[i]].y())
				+ (Downsample_cloud.points[i].z - model->Visible_vertices[cloud_correspond[i]].z())*(Downsample_cloud.points[i].z - model->Visible_vertices[cloud_correspond[i]].z()));

			float reweight = 1.0f / sqrt(length + 1e-3);

			e(i * 3 + 0) = reweight*(Downsample_cloud.points[i].x - model->Visible_vertices[cloud_correspond[i]].x());
			e(i * 3 + 1) = reweight*(Downsample_cloud.points[i].y - model->Visible_vertices[cloud_correspond[i]].y());
			e(i * 3 + 2) = reweight*(Downsample_cloud.points[i].z - model->Visible_vertices[cloud_correspond[i]].z());


			int vert_idx = model->Visible_vertices_index[cloud_correspond[i]];
			jacobian_correspond.block(i * 3, 0, 3, model->NumberofParams) = reweight*model->Compute_one_Vertice_Shape_Jacobian(vert_idx);
		}


		//for (int cor_id = 0; cor_id < NumofCorrespond; ++cor_id)
		//{
		//	int vert_idx = model->Visible_vertices_index[cloud_correspond[cor_id]];
		//	jacobian_correspond.block(cor_id * 3, 0, 3, model->NumberofParams) = model->Compute_one_Vertice_Shape_Jacobian(vert_idx);
		//}

		float omiga = settings->fit3D_weight;
		sys.Jt_J.noalias() += omiga*jacobian_correspond.transpose() * jacobian_correspond;
		sys.Jt_e.noalias() += omiga*jacobian_correspond.transpose() * e;

		///--- Check
		if (Energy::safety_check)
			Energy::has_nan(sys);
	}
}