#pragma once
#include"Types.h"
class Joint
{
public:
	string joint_name;
	int joint_index;

	bool HasChild;

	Vector4 GlobalInitPosition;       //position before change
	Vector4 ChildGlobalInitPosition;


	Vector4 dof_axis[3];//��������

	int pose_params_length;
	int* pose_params_index;
	int* pose_params_type;

	int shape_params_length;
	int* shape_params_index;
	int* shape_params_type;

	int parent_joint_index;

	// local coordiate // 4*4
	Matrix_MxN local;     //ģ�;��������������ģ�;������ģ�����꣩								
	// local rotation // 4*4
	Matrix_MxN rotation;
	// trans
	Matrix_MxN TransShape;
	// scale
	Matrix_MxN Scale;

	// transformation to parent: 4*4
	Matrix_MxN trans;
	// global coordinate // 4*4
	Matrix_MxN global;


	Vector4 CorrespondingPosition;   // position after change
	Vector4 CorrespondingAxis_for_Pose[3];    // Axis after change �����������ᶼ������̬����״����������ת���״̬
	Vector4 CorrespondingAxis_for_shape[3];    //Axis after change �����������ᶼ������̬����״����������ת���״̬

	Joint() {
		HasChild = true;
		dof_axis[0] << 1, 0, 0, 1;
		dof_axis[1] << 0, 1, 0, 1;
		dof_axis[2] << 0, 0, 1, 1;
		local = Matrix_MxN::Zero(4, 4);
		rotation = Matrix_MxN::Identity(4, 4);
		TransShape = Matrix_MxN::Identity(4, 4);
		Scale = Matrix_MxN::Identity(4, 4);
		trans = Matrix_MxN::Zero(4, 4);
		global = Matrix_MxN::Zero(4, 4);
	}
	~Joint()
	{
		delete pose_params_index;
		delete pose_params_type;
		delete shape_params_index;
		delete shape_params_type;
	}
};