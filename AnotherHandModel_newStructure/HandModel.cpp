#include"HandModel.h"

//加载模型相关函数
void HandModel::load_faces(char* filename)
{
	std::ifstream f;
	f.open(filename, std::ios::in);
	f >> NumofFaces;
	FaceIndex = Eigen::MatrixXi::Zero(NumofFaces, 3);
	for (int i = 0; i < NumofFaces; ++i) {
		f >> FaceIndex(i, 0) >> FaceIndex(i, 1) >> FaceIndex(i, 2);
	}
	f.close();
	printf("Load Faces succeed!!!\n");
	std::cout << "num of Face is: " << NumofFaces << std::endl;
}
void HandModel::load_vertices(char* filename)
{
	std::ifstream f;
	f.open(filename, std::ios::in);
	f >> NumofVertices;
	Vectices_init = Eigen::MatrixXf::Zero(NumofVertices, 3);
	Vertices_normal = Eigen::MatrixXf::Zero(NumofVertices, 3);
	for (int i = 0; i < NumofVertices; ++i) {
		f >> Vectices_init(i, 0) >> Vectices_init(i, 1) >> Vectices_init(i, 2);
	}
	f.close();
	printf("Load vertices succeed!!!\n");
	std::cout << "num of vertices is: " << NumofVertices << std::endl;
}
void HandModel::load_weight(char* filename)
{
	std::ifstream f;
	f.open(filename, std::ios::in);
	Weights = Eigen::MatrixXf::Zero(NumofVertices, NumofJoints);
	for (int i = 0; i < NumofVertices; ++i) {
		for (int j = 0; j < NumofJoints; ++j)
		{
			f >> Weights(i, j);
		}
	}
	f.close();
	printf("Load weights succeed!!!\n");
}
void HandModel::Load_HandModel()
{
	this->load_vertices(".\\model\\Adjust_Vertices.txt");
	this->load_faces(".\\model\\Faces.txt");
	this->load_weight(".\\model\\Weight.txt");
}


//初始化手模相关函数
void HandModel::set_local_coordinate() {

	Eigen::Vector3f axis_x, axis_y, axis_z;

	for (int i = 0; i < NumofJoints; ++i)
	{
		if (Joints[i].HasChild)
		{
			axis_x(0) = Joints[i].ChildGlobalInitPosition(0) - Joints[i].GlobalInitPosition(0);
			axis_x(1) = Joints[i].ChildGlobalInitPosition(1) - Joints[i].GlobalInitPosition(1);
			axis_x(2) = Joints[i].ChildGlobalInitPosition(2) - Joints[i].GlobalInitPosition(2);

			axis_z << 0.0f, 0.0f, 1.0f;

			float position[3] = { Joints[i].GlobalInitPosition(0) ,Joints[i].GlobalInitPosition(1) ,Joints[i].GlobalInitPosition(2) };

			axis_x.normalize();
			axis_y = axis_x.cross(axis_z);
			axis_y.normalize();
			axis_z = axis_x.cross(axis_y);
			axis_z.normalize();

			Joints[i].local = Eigen::MatrixXf::Zero(4, 4);

			Joints[i].local(0, 0) = axis_x(0); Joints[i].local(1, 0) = axis_x(1); Joints[i].local(2, 0) = axis_x(2);
			Joints[i].local(0, 1) = axis_y(0); Joints[i].local(1, 1) = axis_y(1); Joints[i].local(2, 1) = axis_y(2);
			Joints[i].local(0, 2) = axis_z(0); Joints[i].local(1, 2) = axis_z(1); Joints[i].local(2, 2) = axis_z(2);
			Joints[i].local(0, 3) = position[0]; Joints[i].local(1, 3) = position[1]; Joints[i].local(2, 3) = position[2];
			Joints[i].local(3, 3) = 1.0;
		}
		else
		{
			float position[3] = { Joints[i].GlobalInitPosition(0) ,Joints[i].GlobalInitPosition(1) ,Joints[i].GlobalInitPosition(2) };
			int parent_index = Joints[i].parent_joint_index;
			Joints[i].local = Joints[parent_index].local;
			Joints[i].local(0, 3) = position[0]; Joints[i].local(1, 3) = position[1]; Joints[i].local(2, 3) = position[2];
		}
	}
}
void HandModel::set_parent_child_transform()
{
	for (int i = 0; i < NumofJoints; ++i)
	{
		int parent_joint_index = Joints[i].parent_joint_index;
		if (parent_joint_index != -1)
		{
			Joints[i].trans = Joints[parent_joint_index].local.inverse()*Joints[i].local;
		}
		else
		{
			Joints[i].trans = Joints[i].local;
		}
	}
}
void HandModel::set_params_bound()  
{
	//pose params bounds
	{
		//wrist
		ParamsUpperBound[0] = 1.0e+10f;	ParamsLowerBound[0] = -1.0e+10f;
		ParamsUpperBound[1] = 1.0e+10f;	ParamsLowerBound[1] = -1.0e+10f;
		ParamsUpperBound[2] = 1.0e+10f;	ParamsLowerBound[2] = -1.0e+10f;
		ParamsUpperBound[3] = 1.0e+10f;	ParamsLowerBound[3] = -1.0e+10f;
		ParamsUpperBound[4] = 1.0e+10f;	ParamsLowerBound[4] = -1.0e+10f;
		ParamsUpperBound[5] = 1.0e+10f;	ParamsLowerBound[5] = -1.0e+10f;

		//thumb
		ParamsUpperBound[6] = 10.0f;   ParamsLowerBound[6] = -60.0f;
		ParamsUpperBound[7] = 90.0f;   ParamsLowerBound[7] = -10.0f;
		ParamsUpperBound[8] = 50.0f;   ParamsLowerBound[8] = -10.0f;
		ParamsUpperBound[9] = 90.0f;   ParamsLowerBound[9] = -10.0f;

		//index
		ParamsUpperBound[10] = 10.0f;  ParamsLowerBound[10] = -90.0f;
		ParamsUpperBound[11] = 20.0f;  ParamsLowerBound[11] = -30.0f;  //--正的向中指靠，负的向拇指靠
		ParamsUpperBound[12] = 10.0f;  ParamsLowerBound[12] = -90.0f;
		ParamsUpperBound[13] = 10.0f;  ParamsLowerBound[13] = -90.0f;

		//middle
		ParamsUpperBound[14] = 10.0f;  ParamsLowerBound[14] = -90.0f;
		ParamsUpperBound[15] = 20.0;   ParamsLowerBound[15] = -10.0f; //--正的向无名指靠，负的向食指靠
		ParamsUpperBound[16] = 10.0f;  ParamsLowerBound[16] = -90.0f;
		ParamsUpperBound[17] = 10.0f;  ParamsLowerBound[17] = -90.0f;

		//ring
		ParamsUpperBound[18] = 10.0;  ParamsLowerBound[18] = -90.0f;
		ParamsUpperBound[19] = 20.0f;  ParamsLowerBound[19] = -15.0f;
		ParamsUpperBound[20] = 10.0f;  ParamsLowerBound[20] = -90.0f;
		ParamsUpperBound[21] = 10.0f;  ParamsLowerBound[21] = -90.0f;

		//pinkey
		ParamsUpperBound[22] = 10.0f;  ParamsLowerBound[22] = -90.0f;
		ParamsUpperBound[23] = 20.0f;  ParamsLowerBound[23] = -15.0f;
		ParamsUpperBound[24] = 10.0f;  ParamsLowerBound[24] = -90.0f;
		ParamsUpperBound[25] = 10.0f;  ParamsLowerBound[25] = -90.0f;
	}

	//shape params bounds
	{
		// 0  --------- 整体的scale   拉长或者缩短
		this->Shape_ParamsUpperBound[0] = 1.5f; this->Shape_ParamsLowerBound[0] = 0.5f;
		// 1  --------- thumb_PIP_trans_x             
		// 2  --------- thumb_DPI_trans_x           
		// 3  --------- thumb_DPI_scale_x
		this->Shape_ParamsUpperBound[1] = 3.94f; this->Shape_ParamsLowerBound[1] = -3.94f;
		this->Shape_ParamsUpperBound[2] = 3.31f; this->Shape_ParamsLowerBound[2] = -3.31f;
		this->Shape_ParamsUpperBound[3] = 1.059f; this->Shape_ParamsLowerBound[3] = 0.941f;
		// 4  --------- Index_MCP_trans_x
		// 5 --------- Index_PIP_trans_x
		// 6 --------- Index_DIP_trans_x
		// 7 --------- Index_DIP_scale_x
		this->Shape_ParamsUpperBound[4] = 6.27f; this->Shape_ParamsLowerBound[4] = -6.27f;
		this->Shape_ParamsUpperBound[5] = 4.94f; this->Shape_ParamsLowerBound[5] = -4.94f;
		this->Shape_ParamsUpperBound[6] = 2.51f; this->Shape_ParamsLowerBound[6] = 2.51f;
		this->Shape_ParamsUpperBound[7] = 1.115f; this->Shape_ParamsLowerBound[7] = 0.885f;
		// 8  --------- Middle_MCP_trans_x
		// 9  --------- Middle_PIP_trans_x
		// 10  --------- Middle_DIP_trans_x
		// 11  --------- Middle_DIP_scale_x 
		this->Shape_ParamsUpperBound[8] = 5.38f; this->Shape_ParamsLowerBound[8] = -5.38f;
		this->Shape_ParamsUpperBound[9] = 3.81f; this->Shape_ParamsLowerBound[9] = -3.81f;
		this->Shape_ParamsUpperBound[10] = 3.00f; this->Shape_ParamsLowerBound[10] = -3.00f;
		this->Shape_ParamsUpperBound[11] = 1.087f; this->Shape_ParamsLowerBound[11] = 0.913f;
		// 12  --------- Ring_MCP_trans_x
		// 13  --------- Ring_PIP_trans_x
		// 14  --------- Ring_DIP_trans_x
		// 15  --------- Ring_DIP_scale_x
		this->Shape_ParamsUpperBound[12] = 5.06f; this->Shape_ParamsLowerBound[12] = -5.06f;
		this->Shape_ParamsUpperBound[13] = 3.87f; this->Shape_ParamsLowerBound[13] = -3.87f;
		this->Shape_ParamsUpperBound[14] = 3.29f; this->Shape_ParamsLowerBound[14] = -3.29f;
		this->Shape_ParamsUpperBound[15] = 1.104f; this->Shape_ParamsLowerBound[15] = 0.896f;
		// 16  --------- Pinkey_MCP_trans_x
		// 17  --------- Pinkey_PIP_trans_x
		// 18  --------- Pinkey_DIP_trans_x
		// 19  --------- Pinkey_DIP_scale_x
		this->Shape_ParamsUpperBound[16] = 4.36f; this->Shape_ParamsLowerBound[16] = -4.36f;
		this->Shape_ParamsUpperBound[17] = 2.77f; this->Shape_ParamsLowerBound[17] = -2.77f;
		this->Shape_ParamsUpperBound[18] = 2.54f; this->Shape_ParamsLowerBound[18] = -2.54f;
		this->Shape_ParamsUpperBound[19] = 1.124f; this->Shape_ParamsLowerBound[19] = 0.876f;
	}

}
void HandModel::Init_HandModel()
{
	this->Joints = new Joint[NumofJoints];
	//init_joints
	{
		//wrist
		{
			Joints[0].joint_name = "Wrist";
			Joints[0].joint_index = 0;
			Joints[0].GlobalInitPosition << 0.0f, 0.0f, 0.0f, 1.0f;
			Joints[0].ChildGlobalInitPosition << -0.399423f, 84.5637f, -2.46058f, 1.0f;
			Joints[0].parent_joint_index = -1;

			Joints[0].pose_params_length = 6;
			Joints[0].pose_params_index = new int[6]; Joints[0].pose_params_type = new int[6];
			Joints[0].pose_params_index[0] = 0; Joints[0].pose_params_type[0] = dof_type(x_axis_trans);
			Joints[0].pose_params_index[1] = 1; Joints[0].pose_params_type[1] = dof_type(y_axis_trans);
			Joints[0].pose_params_index[2] = 2; Joints[0].pose_params_type[2] = dof_type(z_axis_trans);
			Joints[0].pose_params_index[3] = 3; Joints[0].pose_params_type[3] = dof_type(x_axis_rotate);
			Joints[0].pose_params_index[4] = 4; Joints[0].pose_params_type[4] = dof_type(y_axis_rotate);
			Joints[0].pose_params_index[5] = 5; Joints[0].pose_params_type[5] = dof_type(z_axis_rotate);

			Joints[0].shape_params_length = 3;
			Joints[0].shape_params_index = new int[3]; Joints[0].shape_params_type = new int[3];
			Joints[0].shape_params_index[0] = 0; Joints[0].shape_params_type[0] = shape_type(x_axis_scale);
			Joints[0].shape_params_index[1] = 0; Joints[0].shape_params_type[1] = shape_type(y_axis_scale);
			Joints[0].shape_params_index[2] = 0; Joints[0].shape_params_type[2] = shape_type(z_axis_scale);
		}

		//thumb
		{
			{
				Joints[1].joint_name = "ThumbLower";
				Joints[1].joint_index = 1;
				Joints[1].GlobalInitPosition << -28.468f, 27.4715f, -18.4322f, 1.0f;
				Joints[1].ChildGlobalInitPosition << -63.5129f, 52.0925f, -35.8438f, 1.0f;
				Joints[1].parent_joint_index = 0;

				Joints[1].pose_params_length = 2;
				Joints[1].pose_params_index = new int[2]; Joints[1].pose_params_type = new int[2];
				Joints[1].pose_params_index[0] = 6; Joints[1].pose_params_type[0] = dof_type(y_axis_rotate);
				Joints[1].pose_params_index[1] = 7; Joints[1].pose_params_type[1] = dof_type(z_axis_rotate);

				Joints[1].shape_params_length = 0;
				Joints[1].shape_params_index = NULL; Joints[1].shape_params_type = NULL;
			}

			{
				Joints[2].joint_name = "ThumbMiddle";
				Joints[2].joint_index = 2;
				Joints[2].GlobalInitPosition << -63.5129f, 52.0925f, -35.8438f, 1.0f;
				Joints[2].ChildGlobalInitPosition << -80.7877f, 74.8383f, -49.2702f, 1.0f;
				Joints[2].parent_joint_index = 1;

				Joints[2].pose_params_length = 1;
				Joints[2].pose_params_index = new int[1]; Joints[2].pose_params_type = new int[1];
				Joints[2].pose_params_index[0] = 8; Joints[2].pose_params_type[0] = dof_type(z_axis_rotate);

				Joints[2].shape_params_length = 1;
				Joints[2].shape_params_index = new int[1]; Joints[2].shape_params_type = new int[1];
				Joints[2].shape_params_index[0] = 1; Joints[2].shape_params_type[0] = shape_type(x_axis_trans_);
			}

			{
				Joints[3].joint_name = "ThumbTop";
				Joints[3].joint_index = 3;
				Joints[3].GlobalInitPosition << -80.7877f, 74.8383f, -49.2702f, 1.0f;
				Joints[3].ChildGlobalInitPosition << -91.3441f, 96.1361f, -62.7773f, 1.0f;
				Joints[3].parent_joint_index = 2;

				Joints[3].pose_params_length = 1;
				Joints[3].pose_params_index = new int[1]; Joints[3].pose_params_type = new int[1];
				Joints[3].pose_params_index[0] = 9; Joints[3].pose_params_type[0] = dof_type(z_axis_rotate);

				Joints[3].shape_params_length = 2;
				Joints[3].shape_params_index = new int[2]; Joints[3].shape_params_type = new int[2];
				Joints[3].shape_params_index[0] = 2; Joints[3].shape_params_type[0] = shape_type(x_axis_trans_);
				Joints[3].shape_params_index[1] = 3; Joints[3].shape_params_type[1] = shape_type(x_axis_scale);
			}

			{
				Joints[4].joint_name = "ThumbSite";
				Joints[4].joint_index = 4;
				Joints[4].GlobalInitPosition << -91.3441f, 96.1361f, -62.7773f, 1.0f;
				Joints[4].HasChild = false;
				Joints[4].parent_joint_index = 3;

				Joints[4].pose_params_length = 0;
				Joints[4].pose_params_index = NULL; Joints[4].pose_params_type = NULL;

				Joints[4].shape_params_length = 0;
				Joints[4].shape_params_index = NULL; Joints[4].shape_params_type = NULL;
			}
		}

		//index
		{
			{
				Joints[5].joint_name = "IndexLower";
				Joints[5].joint_index = 5;
				Joints[5].GlobalInitPosition << -25.5361f, 84.2559f, -2.98504f, 1.0f;
				Joints[5].ChildGlobalInitPosition << -30.5187f, 123.568f, -5.80625f, 1.0f;
				Joints[5].parent_joint_index = 0;

				Joints[5].pose_params_length = 2;
				Joints[5].pose_params_index = new int[2]; Joints[5].pose_params_type = new int[2];
				Joints[5].pose_params_index[0] = 10; Joints[5].pose_params_type[0] = dof_type(y_axis_rotate);
				Joints[5].pose_params_index[1] = 11; Joints[5].pose_params_type[1] = dof_type(z_axis_rotate);

				Joints[5].shape_params_length = 1;
				Joints[5].shape_params_index = new int[1]; Joints[5].shape_params_type = new int[1];
				Joints[5].shape_params_index[0] = 4; Joints[5].shape_params_type[0] = shape_type(x_axis_trans_);
			}

			{
				Joints[6].joint_name = "IndexMiddle";
				Joints[6].joint_index = 6;
				Joints[6].GlobalInitPosition << -30.5187f, 123.568f, -5.80625f, 1.0f;
				Joints[6].ChildGlobalInitPosition << -32.45f, 144.277f, -14.0787f, 1.0f;
				Joints[6].parent_joint_index = 5;

				Joints[6].pose_params_length = 1;
				Joints[6].pose_params_index = new int[1]; Joints[6].pose_params_type = new int[1];
				Joints[6].pose_params_index[0] = 12; Joints[6].pose_params_type[0] = dof_type(y_axis_rotate);

				Joints[6].shape_params_length = 1;
				Joints[6].shape_params_index = new int[1]; Joints[6].shape_params_type = new int[1];
				Joints[6].shape_params_index[0] = 5; Joints[6].shape_params_type[0] = shape_type(x_axis_trans_);
			}

			{
				Joints[7].joint_name = "IndexTop";
				Joints[7].joint_index = 7;
				Joints[7].GlobalInitPosition << -32.45f, 144.277f, -14.0787f, 1.0f;
				Joints[7].ChildGlobalInitPosition << -33.5417f, 162.155f, -22.184f, 1.0f;
				Joints[7].parent_joint_index = 6;

				Joints[7].pose_params_length = 1;
				Joints[7].pose_params_index = new int[1]; Joints[7].pose_params_type = new int[1];
				Joints[7].pose_params_index[0] = 13; Joints[7].pose_params_type[0] = dof_type(y_axis_rotate);

				Joints[7].shape_params_length = 2;
				Joints[7].shape_params_index = new int[2]; Joints[7].shape_params_type = new int[2];
				Joints[7].shape_params_index[0] = 6; Joints[7].shape_params_type[0] = shape_type(x_axis_trans_);
				Joints[7].shape_params_index[1] = 7; Joints[7].shape_params_type[1] = shape_type(x_axis_scale);
			}

			{
				Joints[8].joint_name = "IndexSite";
				Joints[8].joint_index = 8;
				Joints[8].GlobalInitPosition << -33.5417f, 162.155f, -22.184f, 1.0f;
				Joints[8].HasChild = false;
				Joints[8].parent_joint_index = 7;

				Joints[8].pose_params_length = 0;
				Joints[8].pose_params_index = NULL; Joints[8].pose_params_type = NULL;

				Joints[8].shape_params_length = 0;
				Joints[8].shape_params_index = NULL; Joints[8].shape_params_type = NULL;
			}
		}

		//middle
		{
			{
				Joints[9].joint_name = "MiddleLower";
				Joints[9].joint_index = 9;
				Joints[9].GlobalInitPosition << -0.399423f, 84.5637f, -2.46058f, 1.0f;
				Joints[9].ChildGlobalInitPosition << 0.735502f, 129.175f, -5.76824f, 1.0f;
				Joints[9].parent_joint_index = 0;

				Joints[9].pose_params_length = 2;
				Joints[9].pose_params_index = new int[2]; Joints[9].pose_params_type = new int[2];
				Joints[9].pose_params_index[0] = 14; Joints[9].pose_params_type[0] = dof_type(y_axis_rotate);
				Joints[9].pose_params_index[1] = 15; Joints[9].pose_params_type[1] = dof_type(z_axis_rotate);

				Joints[9].shape_params_length = 1;
				Joints[9].shape_params_index = new int[1]; Joints[9].shape_params_type = new int[1];
				Joints[9].shape_params_index[0] = 8; Joints[9].shape_params_type[0] = shape_type(x_axis_trans_);
			}

			{
				Joints[10].joint_name = "MiddleMiddle";
				Joints[10].joint_index = 10;
				Joints[10].GlobalInitPosition << 0.735502f, 129.175f, -5.76824f, 1.0f;
				Joints[10].ChildGlobalInitPosition << -1.19208f, 152.724f, -17.399f, 1.0f;
				Joints[10].parent_joint_index = 9;

				Joints[10].pose_params_length = 1;
				Joints[10].pose_params_index = new int[1]; Joints[10].pose_params_type = new int[1];
				Joints[10].pose_params_index[0] = 16; Joints[10].pose_params_type[0] = dof_type(y_axis_rotate);

				Joints[10].shape_params_length = 1;
				Joints[10].shape_params_index = new int[1]; Joints[10].shape_params_type = new int[1];
				Joints[10].shape_params_index[0] = 9; Joints[10].shape_params_type[0] = shape_type(x_axis_trans_);
			}

			{
				Joints[11].joint_name = "MiddleTop";
				Joints[11].joint_index = 11;
				Joints[11].GlobalInitPosition << -1.19208f, 152.724f, -17.399f, 1.0f;
				Joints[11].ChildGlobalInitPosition << -3.1667f, 171.001f, -28.2518f, 1.0f;
				Joints[11].parent_joint_index = 10;

				Joints[11].pose_params_length = 1;
				Joints[11].pose_params_index = new int[1]; Joints[11].pose_params_type = new int[1];
				Joints[11].pose_params_index[0] = 17; Joints[11].pose_params_type[0] = dof_type(y_axis_rotate);

				Joints[11].shape_params_length = 2;
				Joints[11].shape_params_index = new int[2]; Joints[11].shape_params_type = new int[2];
				Joints[11].shape_params_index[0] = 10; Joints[11].shape_params_type[0] = shape_type(x_axis_trans_);
				Joints[11].shape_params_index[1] = 11; Joints[11].shape_params_type[1] = shape_type(x_axis_scale);
			}

			{
				Joints[12].joint_name = "MiddleSite";
				Joints[12].joint_index = 12;
				Joints[12].GlobalInitPosition << -3.1667f, 171.001f, -28.2518f, 1.0f;
				Joints[12].HasChild = false;
				Joints[12].parent_joint_index = 11;

				Joints[12].pose_params_length = 0;
				Joints[12].pose_params_index = NULL; Joints[12].pose_params_type = NULL;
			
				Joints[12].shape_params_length = 0;
				Joints[12].shape_params_index = NULL; Joints[12].shape_params_type = NULL;
			}
		}

		//Ring
		{
			{
				Joints[13].joint_name = "RingLower";
				Joints[13].joint_index = 13;
				Joints[13].GlobalInitPosition << 17.9766f, 75.7575f, -5.24194f, 1.0f;
				Joints[13].ChildGlobalInitPosition << 25.3781f, 116.629f, -9.22043f, 1.0f;
				Joints[13].parent_joint_index = 0;

				Joints[13].pose_params_length = 2;
				Joints[13].pose_params_index = new int[2]; Joints[13].pose_params_type = new int[2];
				Joints[13].pose_params_index[0] = 18; Joints[13].pose_params_type[0] = dof_type(y_axis_rotate);
				Joints[13].pose_params_index[1] = 19; Joints[13].pose_params_type[1] = dof_type(z_axis_rotate);

				Joints[13].shape_params_length = 1;
				Joints[13].shape_params_index = new int[1]; Joints[13].shape_params_type = new int[1];
				Joints[13].shape_params_index[0] = 12; Joints[13].shape_params_type[0] = shape_type(x_axis_trans_);
			}

			{
				Joints[14].joint_name = "RingMiddle";
				Joints[14].joint_index = 14;
				Joints[14].GlobalInitPosition << 25.3781f, 116.629f, -9.22043f, 1.0f;
				Joints[14].ChildGlobalInitPosition << 25.3024f, 137.195f, -24.547f, 1.0f;
				Joints[14].parent_joint_index = 13;

				Joints[14].pose_params_length = 1;
				Joints[14].pose_params_index = new int[1]; Joints[14].pose_params_type = new int[1];
				Joints[14].pose_params_index[0] = 20; Joints[14].pose_params_type[0] = dof_type(y_axis_rotate);
			
				Joints[14].shape_params_length = 1;
				Joints[14].shape_params_index = new int[1]; Joints[14].shape_params_type = new int[1];
				Joints[14].shape_params_index[0] = 13; Joints[14].shape_params_type[0] = shape_type(x_axis_trans_);
			}

			{
				Joints[15].joint_name = "RingTop";
				Joints[15].joint_index = 15;
				Joints[15].GlobalInitPosition << 25.3024f, 137.195f, -24.547f, 1.0f;
				Joints[15].ChildGlobalInitPosition << 24.7268f, 153.522f, -38.1339f, 1.0f;
				Joints[15].parent_joint_index = 14;

				Joints[15].pose_params_length = 1;
				Joints[15].pose_params_index = new int[1]; Joints[15].pose_params_type = new int[1];
				Joints[15].pose_params_index[0] = 21; Joints[15].pose_params_type[0] = dof_type(y_axis_rotate);

				Joints[15].shape_params_length = 2;
				Joints[15].shape_params_index = new int[2]; Joints[15].shape_params_type = new int[2];
				Joints[15].shape_params_index[0] = 14; Joints[15].shape_params_type[0] = shape_type(x_axis_trans_);
				Joints[15].shape_params_index[1] = 15; Joints[15].shape_params_type[1] = shape_type(x_axis_scale);
			}

			{
				Joints[16].joint_name = "RingSite";
				Joints[16].joint_index = 16;
				Joints[16].GlobalInitPosition << 24.7268f, 153.522f, -38.1339f, 1.0f;
				Joints[16].HasChild = false;
				Joints[16].parent_joint_index = 15;

				Joints[16].pose_params_length = 0;
				Joints[16].pose_params_index = NULL; Joints[16].pose_params_type = NULL;

				Joints[16].shape_params_length = 0;
				Joints[16].shape_params_index = NULL; Joints[16].shape_params_type = NULL;
			}
		}

		//Pinky
		{

			{
				Joints[17].joint_name = "PinkeyLower";
				Joints[17].joint_index = 17;
				Joints[17].GlobalInitPosition << 33.4795f, 65.2179f, -8.45142f, 1.0f;
				Joints[17].ChildGlobalInitPosition << 44.1109f, 94.4087f, -18.2167f, 1.0f;
				Joints[17].parent_joint_index = 0;

				Joints[17].pose_params_length = 2;
				Joints[17].pose_params_index = new int[2]; Joints[17].pose_params_type = new int[2];
				Joints[17].pose_params_index[0] = 22; Joints[17].pose_params_type[0] = dof_type(y_axis_rotate);
				Joints[17].pose_params_index[1] = 23; Joints[17].pose_params_type[1] = dof_type(z_axis_rotate);

				Joints[17].shape_params_length = 1;
				Joints[17].shape_params_index = new int[1]; Joints[17].shape_params_type = new int[1];
				Joints[17].shape_params_index[0] = 16; Joints[17].shape_params_type[0] = shape_type(x_axis_trans_);
			}

			{
				Joints[18].joint_name = "PinkeyMiddle";
				Joints[18].joint_index = 18;
				Joints[18].GlobalInitPosition << 44.1109f, 94.4087f, -18.2167f, 1.0f;
				Joints[18].ChildGlobalInitPosition << 47.1591f, 109.145f, -28.3087f, 1.0f;
				Joints[18].parent_joint_index = 17;

				Joints[18].pose_params_length = 1;
				Joints[18].pose_params_index = new int[1]; Joints[18].pose_params_type = new int[1];
				Joints[18].pose_params_index[0] = 24; Joints[18].pose_params_type[0] = dof_type(y_axis_rotate);

				Joints[18].shape_params_length = 1;
				Joints[18].shape_params_index = new int[1]; Joints[18].shape_params_type = new int[1];
				Joints[18].shape_params_index[0] = 17; Joints[18].shape_params_type[0] = shape_type(x_axis_trans_);
			}

			{
				Joints[19].joint_name = "PinkeyTop";
				Joints[19].joint_index = 19;
				Joints[19].GlobalInitPosition << 47.1591f, 109.145f, -28.3087f, 1.0f;
				Joints[19].ChildGlobalInitPosition << 48.8287f, 124.917f, -39.9742f, 1.0f;
				Joints[19].parent_joint_index = 18;

				Joints[19].pose_params_length = 1;
				Joints[19].pose_params_index = new int[1]; Joints[19].pose_params_type = new int[1];
				Joints[19].pose_params_index[0] = 25; Joints[19].pose_params_type[0] = dof_type(y_axis_rotate);
			
				Joints[19].shape_params_length = 2;
				Joints[19].shape_params_index = new int[2]; Joints[19].shape_params_type = new int[2];
				Joints[19].shape_params_index[0] = 18; Joints[19].shape_params_type[0] = shape_type(x_axis_trans_);
				Joints[19].shape_params_index[1] = 19; Joints[19].shape_params_type[1] = shape_type(x_axis_scale);
			}

			{
				Joints[20].joint_name = "PinkeySite";
				Joints[20].joint_index = 20;
				Joints[20].GlobalInitPosition << 48.8287f, 124.917f, -39.9742f, 1.0f;
				Joints[20].HasChild = false;
				Joints[20].parent_joint_index = 19;

				Joints[20].pose_params_length = 0;
				Joints[20].pose_params_index = NULL; Joints[20].pose_params_type = NULL;
			
				Joints[20].shape_params_length = 0;
				Joints[20].shape_params_index = NULL; Joints[20].shape_params_type = NULL;
			}
		}
	}


	this->HandPalmCenter << Joints[9].GlobalInitPosition(0)*0.66f, Joints[9].GlobalInitPosition(1)*0.66f, Joints[9].GlobalInitPosition(2)*0.66f, 1.0f;

	this->GlobalPosition << 0.0f, 0.0f, 0.0f, 0.0f;
	this->Load_HandModel();

	//shape_params的对应关系
	// 0  --------- 整体的scale   拉长或者缩短

	// 1  --------- thumb_PIP_trans_x             
	// 2  --------- thumb_DPI_trans_x           
	// 3  --------- thumb_DPI_scale_x

	// 4  --------- Index_MCP_trans_x
	// 5 --------- Index_PIP_trans_x
	// 6 --------- Index_DIP_trans_x
	// 7 --------- Index_DIP_scale_x

	// 8  --------- Middle_MCP_trans_x
	// 9  --------- Middle_PIP_trans_x
	// 10  --------- Middle_DIP_trans_x
	// 11  --------- Middle_DIP_scale_x             

	// 12  --------- Ring_MCP_trans_x
	// 13  --------- Ring_PIP_trans_x
	// 14  --------- Ring_DIP_trans_x
	// 15  --------- Ring_DIP_scale_x

	// 16  --------- Pinky_MCP_trans_x
	// 17  --------- Pinky_PIP_trans_x
	// 18  --------- Pinky_DIP_trans_x
	// 19  --------- Pinky_DIP_scale_x

	this->Params = new float[NumberofParams]();
	this->init_Params = new float[NumberofParams]();
	this->ParamsUpperBound = new float[NumberofParams]();
	this->ParamsLowerBound = new float[NumberofParams]();

	this->Shape_Params = new float[NumofShape_Params]();
	this->Shape_ParamsUpperBound = new float[NumofShape_Params]();
	this->Shape_ParamsLowerBound = new float[NumofShape_Params]();

	this->Shape_Params[0] = 0.8f; 
	this->Shape_Params[3] = 1.0f;
	this->Shape_Params[7] =1.0f;
	this->Shape_Params[11] = 1.0f;
	this->Shape_Params[15] = 1.0f;
	this->Shape_Params[19] = 1.0f;

	outputImage = cv::Mat::zeros(424, 512, CV_8UC1);

	this->set_local_coordinate();
	this->set_parent_child_transform();
	this->set_params_bound();
	this->Updata(this->Params,this->Shape_Params);
}


HandModel::HandModel(Camera * camera_)
{
	this->camera = camera_;
	this->Init_HandModel();
}


//更新手模相关函数
void HandModel::compute_global_matrix()
{
	Joints[0].global = Joints[0].local*Joints[0].TransShape*Joints[0].Scale*Joints[0].rotation;

	for (int i = 0; i < NumofJoints; ++i)
	{
		int parent_joint_index = Joints[i].parent_joint_index;
		if (parent_joint_index != -1)
		{
			Joints[i].global = Joints[parent_joint_index].global*Joints[i].trans*Joints[i].TransShape*Joints[i].Scale*Joints[i].rotation;
		}
	}
}
void HandModel::set_one_rotation(const Eigen::Vector3f& pose, int index)
{
	Eigen::MatrixXf x = Eigen::MatrixXf::Identity(4, 4);
	Eigen::MatrixXf y = Eigen::MatrixXf::Identity(4, 4);
	Eigen::MatrixXf z = Eigen::MatrixXf::Identity(4, 4);

	float cx = cos(pose.x() / 180 * PI);
	float sx = sin(pose.x() / 180 * PI);

	float cy = cos(pose.y() / 180 * PI);
	float sy = sin(pose.y() / 180 * PI);

	float cz = cos(pose.z() / 180 * PI);
	float sz = sin(pose.z() / 180 * PI);

	x(1, 1) = cx; x(2, 2) = cx;
	x(1, 2) = -sx; x(2, 1) = sx;

	y(0, 0) = cy; y(0, 2) = sy;
	y(2, 0) = -sy; y(2, 2) = cy;

	z(0, 0) = cz; z(1, 1) = cz;
	z(0, 1) = -sz; z(1, 0) = sz;

	if (index == 0)
	{
		Joints[index].rotation = y*x*z;
	}
	else
	{
		if (index == 1)
		{
			Joints[index].rotation = z*x*y;
		}
		else
		{
			Joints[index].rotation = x*y*z;
		}
	}

}
void HandModel::set_one_TransShape(const Eigen::Vector3f& p, int index)
{
	Eigen::Matrix<float, 4, 4> Transing = Eigen::MatrixXf::Identity(4, 4);
	Transing(0, 3) = p(0);
	Transing(1, 3) = p(1);
	Transing(2, 3) = p(2);

	Joints[index].TransShape = Transing;
}
void HandModel::set_one_Scale(const Eigen::Vector3f& p, int index)
{
	Eigen::Matrix<float, 4, 4> Scaling = Eigen::MatrixXf::Zero(4, 4);
	Scaling(0, 0) = p(0);
	Scaling(1, 1) = p(1);
	Scaling(2, 2) = p(2);
	Scaling(3, 3) = 1;

	Joints[index].Scale = Scaling;
}
void HandModel::Updata_rotation_matrix(float* poseparams)
{

	//Params对应关系
	//       0       ------>    wrist_T_x    //全局平移
	//       1       ------>    wrist_T_y    //全局平移
	//       2       ------>    wrist_T_z    //全局平移
	//       3       ------>    wrist_R_x
	//       4       ------>    wrist_R_y
	//       5       ------>    wrist_R_z
	//       6       ------>    Thumb_Low_R_y
	//       7       ------>    Thumb_Low_R_z
	//       8       ------>    Thumb_mid_R_z    //这里注意了，是z不是y了
	//       9       ------>    Thumb_top_R_z    //这里注意了，是z不是y了
	//       10      ------>    Index_Low_R_y
	//       11      ------>    Index_Low_R_z
	//       12      ------>    Index_mid_R_y
	//       13      ------>    Index_top_R_y
	//       14      ------>    Middle_Low_R_y
	//       15      ------>    Middle_Low_R_z
	//       16      ------>    Middle_mid_R_y
	//       17      ------>    Middle_top_R_y
	//       18      ------>    Ring_Low_R_y
	//       19      ------>    Ring_Low_R_z
	//       20      ------>    Ring_mid_R_y
	//       21      ------>    Ring_top_R_y
	//       22      ------>    Pinkey_Low_R_y
	//       23      ------>    Pinkey_Low_R_z
	//       24      ------>    Pinkey_mid_R_y
	//       25      ------>    Pinkey_top_R_y

	GlobalPosition(0) = poseparams[0];
	GlobalPosition(1) = poseparams[1];
	GlobalPosition(2) = poseparams[2];
	GlobalPosition(3) = 1;

	Eigen::Vector3f p_wrist(poseparams[3], poseparams[4], poseparams[5]);
	set_one_rotation(p_wrist, 0);

	//thumb
	Eigen::Vector3f p_thumb_lower(0, poseparams[6], poseparams[7]);
	Eigen::Vector3f p_thumb_middle(0, 0, poseparams[8]);        //这里注意了，是z不是y了
	Eigen::Vector3f p_thumb_top(0, 0, poseparams[9]);           //这里注意了，是z不是y了
	set_one_rotation(p_thumb_lower, 1);
	set_one_rotation(p_thumb_middle, 2);
	set_one_rotation(p_thumb_top, 3);

	//index
	Eigen::Vector3f p_pinkey_lower(0, poseparams[10], poseparams[11]);
	Eigen::Vector3f p_pinkey_middle(0, poseparams[12], 0);
	Eigen::Vector3f p_pinkey_top(0, poseparams[13], 0);
	set_one_rotation(p_pinkey_lower, 5);
	set_one_rotation(p_pinkey_middle, 6);
	set_one_rotation(p_pinkey_top, 7);

	//middle
	Eigen::Vector3f p_ring_lower(0, poseparams[14], poseparams[15]);
	Eigen::Vector3f p_ring_middle(0, poseparams[16], 0);
	Eigen::Vector3f p_ring_top(0, poseparams[17], 0);
	set_one_rotation(p_ring_lower, 9);
	set_one_rotation(p_ring_middle, 10);
	set_one_rotation(p_ring_top, 11);

	//ring
	Eigen::Vector3f p_middle_lower(0, poseparams[18], poseparams[19]);
	Eigen::Vector3f p_middle_middle(0, poseparams[20], 0);
	Eigen::Vector3f p_middle_top(0, poseparams[21], 0);
	set_one_rotation(p_middle_lower, 13);
	set_one_rotation(p_middle_middle, 14);
	set_one_rotation(p_middle_top, 15);

	//pinkey
	Eigen::Vector3f p_index_lower(0, poseparams[22], poseparams[23]);
	Eigen::Vector3f p_index_middle(0, poseparams[24], 0);
	Eigen::Vector3f p_index_top(0, poseparams[25], 0);
	set_one_rotation(p_index_lower, 17);
	set_one_rotation(p_index_middle, 18);
	set_one_rotation(p_index_top, 19);
}
void HandModel::Updata_TransShape_matrix(float* shapeParams)
{
	//shape_params的对应关系
	// 1  --------- thumb_PIP_trans_x
	// 2  --------- thumb_DIP_trans_x
	Eigen::Vector3f thumb_PIP_trans(shapeParams[1], 0, 0);
	Eigen::Vector3f thumb_DIP_trans(shapeParams[2], 0, 0);
	set_one_TransShape(thumb_PIP_trans, 2);
	set_one_TransShape(thumb_DIP_trans, 3);

	// 4  --------- Index_MCP_trans_x
	// 5 --------- Index_MCP_trans_y
	// 6 --------- Index_PIP_trans_x
	Eigen::Vector3f Index_MCP_trans(shapeParams[4], 0, 0);
	Eigen::Vector3f Index_PIP_trans(shapeParams[5], 0, 0);
	Eigen::Vector3f Index_DIP_trans(shapeParams[6], 0, 0);
	set_one_TransShape(Index_MCP_trans, 5);
	set_one_TransShape(Index_PIP_trans, 6);
	set_one_TransShape(Index_DIP_trans, 7);

	// 8  --------- Middle_MCP_trans_x
	// 9  --------- Middle_PIP_trans_x
	// 10  --------- Middle_DIP_trans_x
	Eigen::Vector3f Middle_MCP_trans(shapeParams[8], 0 , 0);
	Eigen::Vector3f Middle_PIP_trans(shapeParams[9], 0, 0);
	Eigen::Vector3f Middle_DIP_trans(shapeParams[10], 0, 0);
	set_one_TransShape(Middle_MCP_trans, 9);
	set_one_TransShape(Middle_PIP_trans, 10);
	set_one_TransShape(Middle_DIP_trans, 11);

	// 12  --------- Ring_MCP_trans_x
	// 13  --------- Ring_PIP_trans_x
	// 14  --------- Ring_DIP_trans_x
	Eigen::Vector3f Ring_MCP_trans(shapeParams[12], 0, 0);
	Eigen::Vector3f Ring_PIP_trans(shapeParams[13], 0, 0);
	Eigen::Vector3f Ring_DIP_trans(shapeParams[14], 0, 0);
	set_one_TransShape(Ring_MCP_trans, 13);
	set_one_TransShape(Ring_PIP_trans, 14);
	set_one_TransShape(Ring_DIP_trans, 15);

	// 16  --------- Pinkey_MCP_trans_x
	// 17  --------- Pinkey_PIP_trans_x
	// 18  --------- Pinkey_DIP_trans_x
	Eigen::Vector3f Pinkey_MCP_trans(shapeParams[16], 0, 0);
	Eigen::Vector3f Pinkey_PIP_trans(shapeParams[17], 0, 0);
	Eigen::Vector3f Pinkey_DIP_trans(shapeParams[18], 0, 0);
	set_one_TransShape(Pinkey_MCP_trans, 17);
	set_one_TransShape(Pinkey_PIP_trans, 18);
	set_one_TransShape(Pinkey_DIP_trans, 19);
}
void HandModel::Updata_Scale_matrix(float* shapeParams)
{
	//shape_params的对应关系
	// 0  --------- wrist_scale_x
	Eigen::Vector3f wrist_scale(shapeParams[0], shapeParams[0], shapeParams[0]);
	set_one_Scale(wrist_scale, 0);

	// 3  --------- thumb_DIP_scale_x
	Eigen::Vector3f thumb_DIP_scale(shapeParams[3], 1, 1);
	set_one_Scale(thumb_DIP_scale, 3);

	// 7 --------- Index_DIP_scale_x
	Eigen::Vector3f Index_DIP_scale(shapeParams[7], 1, 1);
	set_one_Scale(Index_DIP_scale, 7);

	// 11  --------- Middle_DIP_scale_x
	Eigen::Vector3f Middle_DIP_scale(shapeParams[11], 1, 1);
	set_one_Scale(Middle_DIP_scale, 11);

	// 15  --------- Ring_DIP_scale_x
	Eigen::Vector3f Ring_DIP_scale(shapeParams[15], 1, 1);
	set_one_Scale(Ring_DIP_scale, 15);

	// 19  --------- Pinkey_DIP_scale_x
	Eigen::Vector3f Pinkey_DIP_scale(shapeParams[19], 1, 1);
	set_one_Scale(Pinkey_DIP_scale, 19);
}

void HandModel::Updata_Joints()
{
	Vector4 Updata_palmCenter(Joints[0].global*Joints[0].local.inverse()*this->HandPalmCenter);
	//updata joints
	for (int i = 0; i < NumofJoints; ++i)
	{
		Joints[i].CorrespondingPosition << Joints[i].global*Joints[i].local.inverse()*Joints[i].GlobalInitPosition + GlobalPosition - Updata_palmCenter;
	}

	Joint_matrix = Eigen::MatrixXf::Zero(NumofJoints, 3);
	for (int i = 0; i < NumofJoints; ++i)
	{
		Joint_matrix(i, 0) = Joints[i].CorrespondingPosition(0);
		Joint_matrix(i, 1) = Joints[i].CorrespondingPosition(1);
		Joint_matrix(i, 2) = Joints[i].CorrespondingPosition(2);
	}
}
void HandModel::Updata_axis()
{
	Vector4 Updata_palmCenter(Joints[0].global*Joints[0].local.inverse()*this->HandPalmCenter);
	//updata axis  这里的axis指的是绕的哪个轴旋转的axis的更新，对于不饶旋转的轴，可以不更新，比如: joint[1、5、9、13、17]这五根手指的low 
	//for pose
	for (int i = 0; i < NumofJoints; ++i)
	{
		if (i != 0 && i != 1 && i != 5 && i != 9 && i != 13 && i != 17)
		{
			Joints[i].CorrespondingAxis_for_Pose[0] << Joints[i].global*Joints[i].dof_axis[0] + GlobalPosition - Updata_palmCenter;
			Joints[i].CorrespondingAxis_for_Pose[1] << Joints[i].global*Joints[i].dof_axis[1] + GlobalPosition - Updata_palmCenter;
			Joints[i].CorrespondingAxis_for_Pose[2] << Joints[i].global*Joints[i].dof_axis[2] + GlobalPosition - Updata_palmCenter;
		}
		else
		{
			if (i == 0)
			{
				//y*x*z;         ==> y轴不变，x轴左乘y轴旋转，z轴完全旋转
				Eigen::Vector3f pose(Params[3], Params[4], Params[5]);
				Eigen::MatrixXf x = Eigen::MatrixXf::Identity(4, 4);
				Eigen::MatrixXf y = Eigen::MatrixXf::Identity(4, 4);
				Eigen::MatrixXf z = Eigen::MatrixXf::Identity(4, 4);

				float cx = cos(pose.x() / 180 * PI);
				float sx = sin(pose.x() / 180 * PI);

				float cy = cos(pose.y() / 180 * PI);
				float sy = sin(pose.y() / 180 * PI);

				float cz = cos(pose.z() / 180 * PI);
				float sz = sin(pose.z() / 180 * PI);

				x(1, 1) = cx; x(2, 2) = cx;
				x(1, 2) = -sx; x(2, 1) = sx;

				y(0, 0) = cy; y(0, 2) = sy;
				y(2, 0) = -sy; y(2, 2) = cy;

				z(0, 0) = cz; z(1, 1) = cz;
				z(0, 1) = -sz; z(1, 0) = sz;


				Joints[i].CorrespondingAxis_for_Pose[0] << Joints[i].local*Joints[i].TransShape*Joints[i].Scale*y*Joints[i].dof_axis[0] + GlobalPosition - Updata_palmCenter;
				Joints[i].CorrespondingAxis_for_Pose[1] << Joints[i].local*Joints[i].TransShape*Joints[i].Scale*Joints[i].dof_axis[1] + GlobalPosition - Updata_palmCenter;
				Joints[i].CorrespondingAxis_for_Pose[2] << Joints[i].local*Joints[i].TransShape*Joints[i].Scale*y*x*Joints[i].dof_axis[2] + GlobalPosition - Updata_palmCenter;
			}
			else
			{
				if (i == 1)
				{
					//z*x*y;   ==>z轴不变，x轴要左乘z的旋转，但是x轴用不到，因此不更新，y轴则遵循完全旋转
					int parent_joint_index = Joints[i].parent_joint_index;

					Joints[i].CorrespondingAxis_for_Pose[1] << Joints[i].global*Joints[i].dof_axis[1] + GlobalPosition - Updata_palmCenter;
					Joints[i].CorrespondingAxis_for_Pose[2] << Joints[parent_joint_index].global*Joints[i].trans*Joints[i].TransShape*Joints[i].Scale*Joints[i].dof_axis[2] + GlobalPosition - Updata_palmCenter;
				}
				else
				{
					//x*y*z;    ==>x轴不变，但是x轴用不到，因此不更新，y轴左乘x轴旋转，但是x轴不旋转，因此绕x旋转是单位矩阵，因此y轴也不变，z轴完全旋转
					int parent_joint_index = Joints[i].parent_joint_index;

					Joints[i].CorrespondingAxis_for_Pose[1] << Joints[parent_joint_index].global*Joints[i].trans*Joints[i].TransShape*Joints[i].Scale*Joints[i].dof_axis[1] + GlobalPosition - Updata_palmCenter;
					Joints[i].CorrespondingAxis_for_Pose[2] << Joints[i].global*Joints[i].dof_axis[2] + GlobalPosition - Updata_palmCenter;
				}
			}
		}
	}


	//for shape
	for (int i = 0; i < NumofJoints; ++i)
	{
		int parent_index = Joints[i].parent_joint_index;
		if (i == 0)
		{
			Joints[i].CorrespondingAxis_for_shape[0] << Joints[i].global*Joints[i].dof_axis[0] + GlobalPosition - Updata_palmCenter;
			Joints[i].CorrespondingAxis_for_shape[1] << Joints[i].global*Joints[i].dof_axis[1] + GlobalPosition - Updata_palmCenter;
			Joints[i].CorrespondingAxis_for_shape[2] << Joints[i].global*Joints[i].dof_axis[2] + GlobalPosition - Updata_palmCenter;
		}
		else
		{
			Joints[i].CorrespondingAxis_for_shape[0] << Joints[parent_index].global*Joints[i].trans*Joints[i].TransShape*Joints[i].Scale*Joints[i].dof_axis[0] + GlobalPosition - Updata_palmCenter;
			Joints[i].CorrespondingAxis_for_shape[1] << Joints[parent_index].global*Joints[i].trans*Joints[i].TransShape*Joints[i].Scale*Joints[i].dof_axis[1] + GlobalPosition - Updata_palmCenter;
			Joints[i].CorrespondingAxis_for_shape[2] << Joints[parent_index].global*Joints[i].trans*Joints[i].TransShape*Joints[i].Scale*Joints[i].dof_axis[2] + GlobalPosition - Updata_palmCenter;
		}
	}
}
void HandModel::Updata_Vertices()
{
	Vector4 Updata_palmCenter(Joints[0].global*Joints[0].local.inverse()*this->HandPalmCenter);
	Eigen::MatrixXf t = Eigen::MatrixXf::Zero(4, NumofVertices);
	Eigen::MatrixXf x = Eigen::MatrixXf::Ones(4, NumofVertices);
	x.block(0, 0, 3, NumofVertices) = Vectices_init.block(0, 0, NumofVertices, 3).transpose();

	{
		for (int i = 0; i < NumofJoints; ++i) {
			Eigen::MatrixXf y = Weights.block(0, i, NumofVertices, 1);// 在所有顶点 对于 该关节点的weight
			Eigen::MatrixXf y0 = y.replicate(1, 4);    //分别是行重复1遍，列重复4遍，结果为（num_vertices_，4）这么大小的矩阵
			Eigen::MatrixXf z = Joints[i].global * Joints[i].local.inverse() * x;
			{
				t = t + z.cwiseProduct(y0.transpose());
			}
		}
	}

	Vertices_update_ = t.transpose();

	for (int i = 0; i < Vertices_update_.rows(); ++i) {
		Vertices_update_(i, 0) += GlobalPosition(0) - Updata_palmCenter(0);
		Vertices_update_(i, 1) += GlobalPosition(1) - Updata_palmCenter(1);
		Vertices_update_(i, 2) += GlobalPosition(2) - Updata_palmCenter(2);
	}
}
void HandModel::Updata_normal_And_visibel_vertices()
{
	Visible_vertices.clear();
	Visible_vertices_index.clear();
	Visible_vertices_2D.clear();
	Face_normal.clear();
	Face_normal.resize(NumofFaces);

	Vertices_normal.setZero();

	{
		for (int i = 0; i < NumofFaces; ++i)
		{
			Eigen::Vector3f A, B, C, BA, BC;
			//这里我假设，如果假设错了，那么叉乘时候，就BC*BA变成BA*BC
			//            A
			//          /  \
									//         B — C
			A << Vertices_update_(FaceIndex(i, 0), 0), Vertices_update_(FaceIndex(i, 0), 1), Vertices_update_(FaceIndex(i, 0), 2);
			B << Vertices_update_(FaceIndex(i, 1), 0), Vertices_update_(FaceIndex(i, 1), 1), Vertices_update_(FaceIndex(i, 1), 2);
			C << Vertices_update_(FaceIndex(i, 2), 0), Vertices_update_(FaceIndex(i, 2), 1), Vertices_update_(FaceIndex(i, 2), 2);

			BC << C - B;
			BA << A - B;

			Eigen::Vector3f nom(BC.cross(BA));

			nom.normalize();

			Face_normal[i] = nom;

			Vertices_normal(FaceIndex(i, 0), 0) += nom(0);
			Vertices_normal(FaceIndex(i, 1), 0) += nom(0);
			Vertices_normal(FaceIndex(i, 2), 0) += nom(0);

			Vertices_normal(FaceIndex(i, 0), 1) += nom(1);
			Vertices_normal(FaceIndex(i, 1), 1) += nom(1);
			Vertices_normal(FaceIndex(i, 2), 1) += nom(1);

			Vertices_normal(FaceIndex(i, 0), 2) += nom(2);
			Vertices_normal(FaceIndex(i, 1), 2) += nom(2);
			Vertices_normal(FaceIndex(i, 2), 2) += nom(2);

		}
	}


	for (int i = 0; i < Vertices_normal.rows(); ++i)
	{
		Vertices_normal.row(i).normalize();

		Eigen::Vector3f visible_v(Vertices_update_(i, 0), Vertices_update_(i, 1), Vertices_update_(i, 2));
		Eigen::Vector2i visible_2D;
		visible_2D << (int)(camera->world_to_depth_image(visible_v)(0)), (int)(camera->world_to_depth_image(visible_v)(1));

		Visible_vertices_2D.push_back(visible_2D);


		if ((Vertices_normal(i, 2)) < 0)
		{
			Eigen::Vector3f visible_v(Vertices_update_(i, 0), Vertices_update_(i, 1), Vertices_update_(i, 2));
			Visible_vertices.push_back(visible_v);
			Visible_vertices_index.push_back(i);

			/*Eigen::Vector2i visible_2D;
			visible_2D << (int)(camera->world_to_depth_image(visible_v)(0)), (int)(camera->world_to_depth_image(visible_v)(1));

			Visible_vertices_2D.push_back(visible_2D);*/
		}
	}

	//cout << "visible vertices find done!" << endl;
}
void HandModel::Updata(float *poseparams, float *shapeParams)
{
	Updata_rotation_matrix(poseparams);
	Updata_Scale_matrix(shapeParams);
	Updata_TransShape_matrix(shapeParams);

	compute_global_matrix();
	Updata_Joints();
	Updata_axis();
	Updata_Vertices();   
	Updata_normal_And_visibel_vertices();   
}



//计算某一个点的雅各比，包括关节点、带权重的Vector顶点、不带权重的碰撞体顶点
Eigen::MatrixXf HandModel::Compute_one_Joint_Jacobian(int index)
{
	float omiga = 3.141592f / 180.0f;
	//float omiga = 1;
	Eigen::MatrixXf Jacobain_ = Eigen::MatrixXf::Zero(3, NumberofParams);

	int current_indx = index;
	Eigen::Vector3f current_joint_position(Joints[index].CorrespondingPosition(0), Joints[index].CorrespondingPosition(1), Joints[index].CorrespondingPosition(2));

	//计算时候会用到的变量
	Eigen::Vector3f axis_base_position;
	Eigen::Vector3f x_axis_position;
	Eigen::Vector3f y_axis_position;
	Eigen::Vector3f z_axis_position;

	Eigen::Vector3f w_x, w_y, w_z;
	Eigen::Vector3f S;
	Eigen::Vector3f result;

	while (current_indx >= 0)
	{
		axis_base_position << Joints[current_indx].CorrespondingPosition(0), Joints[current_indx].CorrespondingPosition(1), Joints[current_indx].CorrespondingPosition(2);

		x_axis_position << Joints[current_indx].CorrespondingAxis_for_Pose[0](0), Joints[current_indx].CorrespondingAxis_for_Pose[0](1), Joints[current_indx].CorrespondingAxis_for_Pose[0](2);
		y_axis_position << Joints[current_indx].CorrespondingAxis_for_Pose[1](0), Joints[current_indx].CorrespondingAxis_for_Pose[1](1), Joints[current_indx].CorrespondingAxis_for_Pose[1](2);
		z_axis_position << Joints[current_indx].CorrespondingAxis_for_Pose[2](0), Joints[current_indx].CorrespondingAxis_for_Pose[2](1), Joints[current_indx].CorrespondingAxis_for_Pose[2](2);

		w_x << (x_axis_position - axis_base_position);
		w_y << (y_axis_position - axis_base_position);
		w_z << (z_axis_position - axis_base_position);

		w_x.normalize();
		w_y.normalize();
		w_z.normalize();

		S << (current_joint_position - axis_base_position);

		int params_len = Joints[current_indx].pose_params_length;
		for (int idx = 0; idx < params_len; idx++)
		{
			int params_idx = Joints[current_indx].pose_params_index[idx];

			switch (Joints[current_indx].pose_params_type[idx])
			{
			case dof_type(x_axis_rotate): {
				result << omiga*w_x.cross(S);
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(y_axis_rotate): {
				result << omiga*w_y.cross(S);
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(z_axis_rotate): {
				result << omiga*w_z.cross(S);
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(x_axis_trans): {
				result << 1, 0, 0;
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(y_axis_trans): {
				result << 0, 1, 0;
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(z_axis_trans): {
				result << 0, 0, 1;
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			}

		}

		current_indx = Joints[current_indx].parent_joint_index;
	}

	return Jacobain_;
}
Eigen::MatrixXf HandModel::Compute_one_Vertice_Jacobian(int index)
{
	float omiga = 3.141592f / 180.0f;
	//float omiga = 1;
	Eigen::MatrixXf Jacobain_ = Eigen::MatrixXf::Zero(3, NumberofParams);

	int vert_idx = index;

	Eigen::Vector3f vertice_position(Vertices_update_(vert_idx, 0), Vertices_update_(vert_idx, 1), Vertices_update_(vert_idx, 2));

	//计算时候会用到的变量
	Eigen::Vector3f axis_base_position;
	Eigen::Vector3f x_axis_position;
	Eigen::Vector3f y_axis_position;
	Eigen::Vector3f z_axis_position;

	Eigen::Vector3f w_x, w_y, w_z;
	Eigen::Vector3f S;
	Eigen::Vector3f result;

	for (int joint_idx = 0; joint_idx < NumofJoints; joint_idx++)
	{
		float weight = Weights(vert_idx, joint_idx);
		if (weight > 0.0f)
		{
			//do some thing follow the kanimatic chains
			int current_joint_idx = joint_idx;

			while (current_joint_idx >= 0)
			{
				axis_base_position << Joints[current_joint_idx].CorrespondingPosition(0), Joints[current_joint_idx].CorrespondingPosition(1), Joints[current_joint_idx].CorrespondingPosition(2);

				x_axis_position << Joints[current_joint_idx].CorrespondingAxis_for_Pose[0](0), Joints[current_joint_idx].CorrespondingAxis_for_Pose[0](1), Joints[current_joint_idx].CorrespondingAxis_for_Pose[0](2);
				y_axis_position << Joints[current_joint_idx].CorrespondingAxis_for_Pose[1](0), Joints[current_joint_idx].CorrespondingAxis_for_Pose[1](1), Joints[current_joint_idx].CorrespondingAxis_for_Pose[1](2);
				z_axis_position << Joints[current_joint_idx].CorrespondingAxis_for_Pose[2](0), Joints[current_joint_idx].CorrespondingAxis_for_Pose[2](1), Joints[current_joint_idx].CorrespondingAxis_for_Pose[2](2);


				w_x << (x_axis_position - axis_base_position);
				w_y << (y_axis_position - axis_base_position);
				w_z << (z_axis_position - axis_base_position);

				w_x.normalize();
				w_y.normalize();
				w_z.normalize();

				S << (vertice_position - axis_base_position);

				int params_len = Joints[current_joint_idx].pose_params_length;
				for (int idx = 0; idx < params_len; idx++)
				{
					int params_idx = Joints[current_joint_idx].pose_params_index[idx];

					switch (Joints[current_joint_idx].pose_params_type[idx])
					{
					case dof_type(x_axis_rotate): {
						result << omiga*w_x.cross(S);
						Jacobain_(0, params_idx) += weight*result(0);
						Jacobain_(1, params_idx) += weight*result(1);
						Jacobain_(2, params_idx) += weight*result(2);
						break;
					}
					case dof_type(y_axis_rotate): {
						result << omiga*w_y.cross(S);
						Jacobain_(0, params_idx) += weight*result(0);
						Jacobain_(1, params_idx) += weight*result(1);
						Jacobain_(2, params_idx) += weight*result(2);
						break;
					}
					case dof_type(z_axis_rotate): {
						result << omiga*w_z.cross(S);
						Jacobain_(0, params_idx) += weight*result(0);
						Jacobain_(1, params_idx) += weight*result(1);
						Jacobain_(2, params_idx) += weight*result(2);
						break;
					}
					case dof_type(x_axis_trans): {
						result << 1, 0, 0;
						Jacobain_(0, params_idx) += weight*result(0);
						Jacobain_(1, params_idx) += weight*result(1);
						Jacobain_(2, params_idx) += weight*result(2);
						break;
					}
					case dof_type(y_axis_trans): {
						result << 0, 1, 0;
						Jacobain_(0, params_idx) += weight*result(0);
						Jacobain_(1, params_idx) += weight*result(1);
						Jacobain_(2, params_idx) += weight*result(2);
						break;
					}
					case dof_type(z_axis_trans): {
						result << 0, 0, 1;
						Jacobain_(0, params_idx) += weight*result(0);
						Jacobain_(1, params_idx) += weight*result(1);
						Jacobain_(2, params_idx) += weight*result(2);
						break;
					}
					}

				}
				current_joint_idx = Joints[current_joint_idx].parent_joint_index;
			}
		}
	}

	return Jacobain_;
}
Eigen::MatrixXf HandModel::Compute_one_CollisionPoint_Jacobian(Collision& a, Eigen::Vector3f& point)
{
	float omiga = 3.141592f / 180.0f;
	//float omiga = 1;
	Eigen::MatrixXf Jacobain_ = Eigen::MatrixXf::Zero(3, NumberofParams);

	int current_indx = a.joint_index;
	Eigen::Vector3f current_CollisionPoint_position(point(0), point(1), point(2));

	//计算时候会用到的变量
	Eigen::Vector3f axis_base_position;
	Eigen::Vector3f x_axis_position;
	Eigen::Vector3f y_axis_position;
	Eigen::Vector3f z_axis_position;

	Eigen::Vector3f w_x, w_y, w_z;
	Eigen::Vector3f S;
	Eigen::Vector3f result;

	while (current_indx >= 0)
	{
		axis_base_position << Joints[current_indx].CorrespondingPosition(0), Joints[current_indx].CorrespondingPosition(1), Joints[current_indx].CorrespondingPosition(2);

		x_axis_position << Joints[current_indx].CorrespondingAxis_for_Pose[0](0), Joints[current_indx].CorrespondingAxis_for_Pose[0](1), Joints[current_indx].CorrespondingAxis_for_Pose[0](2);
		y_axis_position << Joints[current_indx].CorrespondingAxis_for_Pose[1](0), Joints[current_indx].CorrespondingAxis_for_Pose[1](1), Joints[current_indx].CorrespondingAxis_for_Pose[1](2);
		z_axis_position << Joints[current_indx].CorrespondingAxis_for_Pose[2](0), Joints[current_indx].CorrespondingAxis_for_Pose[2](1), Joints[current_indx].CorrespondingAxis_for_Pose[2](2);


		w_x << (x_axis_position - axis_base_position);
		w_y << (y_axis_position - axis_base_position);
		w_z << (z_axis_position - axis_base_position);

		w_x.normalize();
		w_y.normalize();
		w_z.normalize();

		S << (current_CollisionPoint_position - axis_base_position);

		int params_len = Joints[current_indx].pose_params_length;
		for (int idx = 0; idx < params_len; idx++)
		{
			int params_idx = Joints[current_indx].pose_params_index[idx];

			switch (Joints[current_indx].pose_params_type[idx])
			{
			case dof_type(x_axis_rotate): {
				result << omiga*w_x.cross(S);
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(y_axis_rotate): {
				result << omiga*w_y.cross(S);
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(z_axis_rotate): {
				result << omiga*w_z.cross(S);
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(x_axis_trans): {
				result << 1, 0, 0;
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(y_axis_trans): {
				result << 0, 1, 0;
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			case dof_type(z_axis_trans): {
				result << 0, 0, 1;
				Jacobain_(0, params_idx) += result(0);
				Jacobain_(1, params_idx) += result(1);
				Jacobain_(2, params_idx) += result(2);
				break;
			}
			}

		}

		current_indx = Joints[current_indx].parent_joint_index;
	}

	return Jacobain_;
}

Eigen::MatrixXf HandModel::Compute_one_Joint_Shape_Jacobian(int index)
{
	Eigen::MatrixXf Jacobain_ = Eigen::MatrixXf::Zero(3, NumofShape_Params);

	int current_indx = index;

	//计算时候会用到的变量
	Eigen::Vector3f axis_base_position;
	Eigen::Vector3f x_axis_position;
	Eigen::Vector3f y_axis_position;
	Eigen::Vector3f z_axis_position;

	Eigen::Vector3f w_x, w_y, w_z;

	Vector4 weight_;

	while (current_indx >= 0)
	{
		axis_base_position << Joints[current_indx].CorrespondingPosition(0), Joints[current_indx].CorrespondingPosition(1), Joints[current_indx].CorrespondingPosition(2);

		x_axis_position << Joints[current_indx].CorrespondingAxis_for_shape[0](0), Joints[current_indx].CorrespondingAxis_for_shape[0](1), Joints[current_indx].CorrespondingAxis_for_shape[0](2);
		y_axis_position << Joints[current_indx].CorrespondingAxis_for_shape[1](0), Joints[current_indx].CorrespondingAxis_for_shape[1](1), Joints[current_indx].CorrespondingAxis_for_shape[1](2);
		z_axis_position << Joints[current_indx].CorrespondingAxis_for_shape[2](0), Joints[current_indx].CorrespondingAxis_for_shape[2](1), Joints[current_indx].CorrespondingAxis_for_shape[2](2);

		w_x << (x_axis_position - axis_base_position);
		w_y << (y_axis_position - axis_base_position);
		w_z << (z_axis_position - axis_base_position);

		w_x.normalize();
		w_y.normalize();
		w_z.normalize();

		weight_ << Joints[current_indx].local.inverse()*Joints[index].GlobalInitPosition;

		int params_len = Joints[current_indx].shape_params_length;
		for (int idx = 0; idx < params_len; idx++)
		{
			int params_idx = Joints[current_indx].shape_params_index[idx];

			switch (Joints[current_indx].shape_params_type[idx])
			{
			case shape_type(x_axis_scale): {
				Jacobain_(0, params_idx) += weight_(0)*w_x(0);
				Jacobain_(1, params_idx) += weight_(0)*w_x(1);
				Jacobain_(2, params_idx) += weight_(0)*w_x(2);
				break;
			}
			case shape_type(y_axis_scale): {
				Jacobain_(0, params_idx) += weight_(1)*w_y(0);
				Jacobain_(1, params_idx) += weight_(1)*w_y(1);
				Jacobain_(2, params_idx) += weight_(1)*w_y(2);
				break;
			}
			case shape_type(z_axis_scale): {
				Jacobain_(0, params_idx) += weight_(2)*w_z(0);
				Jacobain_(1, params_idx) += weight_(2)*w_z(1);
				Jacobain_(2, params_idx) += weight_(2)*w_z(2);
				break;
			}
			case shape_type(x_axis_trans_): {
				Jacobain_(0, params_idx) += w_x(0);
				Jacobain_(1, params_idx) += w_x(1);
				Jacobain_(2, params_idx) += w_x(2);
				break;
			}
			case shape_type(y_axis_trans_): {
				Jacobain_(0, params_idx) += w_y(0);
				Jacobain_(1, params_idx) += w_y(1);
				Jacobain_(2, params_idx) += w_y(2);
				break;
			}
			case shape_type(z_axis_trans_): {
				Jacobain_(0, params_idx) += w_z(0);
				Jacobain_(1, params_idx) += w_z(1);
				Jacobain_(2, params_idx) += w_z(2);
				break;
			}
			}

		}

		current_indx = Joints[current_indx].parent_joint_index;
	}

	return Jacobain_;
}
Eigen::MatrixXf HandModel::Compute_one_Vertice_Shape_Jacobian(int index)
{

	Eigen::MatrixXf Jacobain_ = Eigen::MatrixXf::Zero(3, NumofShape_Params);

	int vert_idx = index;

	Eigen::Vector4f vertice_position(Vectices_init(vert_idx, 0), Vectices_init(vert_idx, 1), Vectices_init(vert_idx, 2),1);

	//计算时候会用到的变量
	Eigen::Vector3f axis_base_position;
	Eigen::Vector3f x_axis_position;
	Eigen::Vector3f y_axis_position;
	Eigen::Vector3f z_axis_position;

	Eigen::Vector3f w_x, w_y, w_z;
	Vector4 weight_;

	for (int joint_idx = 0; joint_idx < NumofJoints; joint_idx++)
	{
		float weight = Weights(vert_idx, joint_idx);
		if (weight > 0.0f)
		{
			//do some thing follow the kanimatic chains
			int current_joint_idx = joint_idx;

			while (current_joint_idx >= 0)
			{
				axis_base_position << Joints[current_joint_idx].CorrespondingPosition(0), Joints[current_joint_idx].CorrespondingPosition(1), Joints[current_joint_idx].CorrespondingPosition(2);

				x_axis_position << Joints[current_joint_idx].CorrespondingAxis_for_shape[0](0), Joints[current_joint_idx].CorrespondingAxis_for_shape[0](1), Joints[current_joint_idx].CorrespondingAxis_for_shape[0](2);
				y_axis_position << Joints[current_joint_idx].CorrespondingAxis_for_shape[1](0), Joints[current_joint_idx].CorrespondingAxis_for_shape[1](1), Joints[current_joint_idx].CorrespondingAxis_for_shape[1](2);
				z_axis_position << Joints[current_joint_idx].CorrespondingAxis_for_shape[2](0), Joints[current_joint_idx].CorrespondingAxis_for_shape[2](1), Joints[current_joint_idx].CorrespondingAxis_for_shape[2](2);

				w_x << (x_axis_position - axis_base_position);
				w_y << (y_axis_position - axis_base_position);
				w_z << (z_axis_position - axis_base_position);

				w_x.normalize();
				w_y.normalize();
				w_z.normalize();

				weight_ << Joints[current_joint_idx].local.inverse()*vertice_position;

				int shape_params_len = Joints[current_joint_idx].shape_params_length;
				for (int idx = 0; idx < shape_params_len; idx++)
				{
					int params_idx = Joints[current_joint_idx].shape_params_index[idx];

					switch (Joints[current_joint_idx].shape_params_type[idx])
					{
					case shape_type(x_axis_scale): {
						Jacobain_(0, params_idx) += weight*weight_(0)*w_x(0);
						Jacobain_(1, params_idx) += weight*weight_(0)*w_x(1);
						Jacobain_(2, params_idx) += weight*weight_(0)*w_x(2);
						break;
					}
					case shape_type(y_axis_scale): {
						Jacobain_(0, params_idx) += weight*weight_(1)*w_y(0);
						Jacobain_(1, params_idx) += weight*weight_(1)*w_y(1);
						Jacobain_(2, params_idx) += weight*weight_(1)*w_y(2);
						break;
					}
					case shape_type(z_axis_scale): {
						Jacobain_(0, params_idx) += weight*weight_(2)*w_z(0);
						Jacobain_(1, params_idx) += weight*weight_(2)*w_z(1);
						Jacobain_(2, params_idx) += weight*weight_(2)*w_z(2);
						break;
					}
					case shape_type(x_axis_trans_): {
						Jacobain_(0, params_idx) += weight*w_x(0);
						Jacobain_(1, params_idx) += weight*w_x(1);
						Jacobain_(2, params_idx) += weight*w_x(2);
						break;
					}
					case shape_type(y_axis_trans_): {
						Jacobain_(0, params_idx) += weight*w_y(0);
						Jacobain_(1, params_idx) += weight*w_y(1);
						Jacobain_(2, params_idx) += weight*w_y(2);
						break;
					}
					case shape_type(z_axis_trans_): {
						Jacobain_(0, params_idx) += weight*w_z(0);
						Jacobain_(1, params_idx) += weight*w_z(1);
						Jacobain_(2, params_idx) += weight*w_z(2);
						break;
					}
					}

				}
				current_joint_idx = Joints[current_joint_idx].parent_joint_index;
			}
		}
	}

	return Jacobain_;
}


//显示参数
void HandModel::Print_fingerLength()
{
	cout << "============>>>Hand Length : " << endl;

	for (int i = 0; i < 5; ++i)
	{
		double Length_pd, Length_pm, Length_pp, m;
		Length_pp = (Joint_matrix.row(i * 4 + 2) - Joint_matrix.row(i * 4 + 1)).norm();
		Length_pm = (Joint_matrix.row(i * 4 + 3) - Joint_matrix.row(i * 4 + 2)).norm();
		Length_pd = (Joint_matrix.row(i * 4 + 4) - Joint_matrix.row(i * 4 + 3)).norm();

		if (i != 0) m = (Joint_matrix.row(i * 4 + 1) - Joint_matrix.row(0)).norm();

		switch (i)
		{
		case 0:
			cout << "Thumb  : " << Length_pd-5.67 << "    " << Length_pm << "    " << Length_pp << endl;
			break;
		case 1:
			cout << "Index  : " << Length_pd-3.84 << "    " << Length_pm << "    " << Length_pp << "    " << m<<endl;
			break;
		case 2:
			cout << "Middle : " << Length_pd-3.95 << "    " << Length_pm << "    " << Length_pp << "    " << m << endl;
			break;
		case 3:
			cout << "Ring   : " << Length_pd-3.95 << "    " << Length_pm << "    " << Length_pp << "    " << m << endl;
			break;
		case 4:
			cout << "Pineky : " << Length_pd-3.73 << "    " << Length_pm << "    " << Length_pp << "    " << m << endl;
			break;
		}

	}

	cout << endl;
}

void HandModel::Save()
{
	std::ofstream f;
	f.open(".\\test\\Adjust_vertices.txt", std::ios::out);
	f << this->NumofVertices << endl;
	for (int i = 0; i < this->NumofVertices; i++) {
		f << this->Vertices_update_(i, 0) << "  "
			<< this->Vertices_update_(i, 1) << "  "
			<< this->Vertices_update_(i, 2) << endl;
	}
	f.close();
	printf("Save Adjust vertices succeed!!!\n");

	std::ofstream f_2;
	f_2.open(".\\test\\Adjust_joints.txt", std::ios::out);
	f_2 << this->NumofJoints << endl;
	for (int i = 0; i < this->NumofJoints; i++) {
		f_2 << this->Joints[i].CorrespondingPosition(0) << ","
			<< this->Joints[i].CorrespondingPosition(1) << ","
			<< this->Joints[i].CorrespondingPosition(2) << ","
			<<1<<endl;
	}
	f_2.close();
	printf("Save Target Joints succeed!!!\n");
}

void HandModel::Generate_img()
{
	outputImage.setTo(0);
	uchar * pointer = outputImage.data;
	//cout << "size" << Visible_vertices_2D.size() << endl;
	for (int i = 0; i < Visible_vertices_2D.size(); i++)
	{
		if((Visible_vertices_2D[i](0)>=0) && 
			(Visible_vertices_2D[i](0) <=512-1) && 
			(Visible_vertices_2D[i](1) >= 0)&& 
			(Visible_vertices_2D[i](1) <=424-1) )

			pointer[Visible_vertices_2D[i](1)*512 + Visible_vertices_2D[i](0)] = 255;
	}

	vector<vector<cv::Point> > contours;
	cv::findContours(outputImage, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); //找轮廓
	//outputImage.setTo(0);
	cv::drawContours(outputImage, contours, -1, cv::Scalar(255), CV_FILLED); //在遮罩图层上，用白色像素填充轮廓，得到MASK
	
	cout<<cv::countNonZero(outputImage)<<endl;
}