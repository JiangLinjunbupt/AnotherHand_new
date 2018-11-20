#pragma once
#include"Joint.h"
#include<fstream>
#include<iostream>
#include<vector>
#include<string>
#include<ctime>
#include"Camera.h"

class HandModel
{
public:
	const int NumofJoints = 21;
	const int NumberofParams = 26;
	const int NumofShape_Params = 20;
public:
	Camera * camera;
	Joint* Joints;

	Eigen::MatrixXi         FaceIndex;             //��ģ�������������
	vector<Eigen::Vector3f> Face_normal;
	Matrix_Nx3              Vectices_init;         //��ģ��ʼ��������
	Matrix_MxN              Vertices_update_;      //���º���ģ�Ķ���
	Matrix_Nx3              Vertices_normal;       //����ķ�����
	Matrix_MxN              Weights;

	vector<Eigen::Vector3f> Visible_vertices;                   //�ɼ�����
	vector<Eigen::Vector2i> Visible_vertices_2D;
	vector<int>             Visible_vertices_index;            //�ɼ����������          
	Matrix_Nx3              Joint_matrix;                      //�����ǽ�joint*�����еĹؽڵ�λ�����ϵ�һ��������

	int NumofVertices;
	int NumofFaces;

	float *Params;
	float *init_Params;         //�������׸��ĳ�ʼ�ؽڽǶ�
	float* ParamsUpperBound;
	float* ParamsLowerBound;

	float *Shape_Params;
	float* Shape_ParamsUpperBound;
	float* Shape_ParamsLowerBound;

	Vector4 GlobalPosition;
	Vector4 HandPalmCenter;


	HandModel(Camera * camera_);
	~HandModel() { delete ParamsLowerBound; delete ParamsUpperBound; delete Params; delete Joints; }


public:
    //������ģ
	void load_faces(char* file);
	void load_vertices(char* file);
	void load_weight(char* file);
	void Load_HandModel();

	//��ʼ����ģ
	void set_local_coordinate();
	void set_parent_child_transform();
	void set_params_bound();
	void Init_HandModel();

	//������̬������ģ
	void compute_global_matrix();
	void set_one_rotation(const Eigen::Vector3f& p, int index);
	void set_one_TransShape(const Eigen::Vector3f& p, int index);
	void set_one_Scale(const Eigen::Vector3f& p, int index);
	void Updata_rotation_matrix(float* params);
	void Updata_TransShape_matrix(float* shapeParams);
	void Updata_Scale_matrix(float* shapeParams);
	void Updata_Joints();
	void Updata_axis();
	void Updata_Vertices();
	void Updata_normal_And_visibel_vertices();
	void Updata(float *Pose_Params,float *Shape_Params);


	//����ĳһ������Ÿ��ȣ������ؽڵ㡢��Ȩ�ص�Vector���㡢����Ȩ�ص���ײ�嶥��
	Eigen::MatrixXf Compute_one_Joint_Jacobian(int index);
	Eigen::MatrixXf Compute_one_Vertice_Jacobian(int index);
	Eigen::MatrixXf Compute_one_CollisionPoint_Jacobian(Collision& a, Eigen::Vector3f& point);

	Eigen::MatrixXf Compute_one_Joint_Shape_Jacobian(int index);
	Eigen::MatrixXf Compute_one_Vertice_Shape_Jacobian(int index);

	//��ʾ����
	void Print_fingerLength();
	void Save();
};