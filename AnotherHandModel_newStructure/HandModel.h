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

	Eigen::MatrixXi         FaceIndex;             //手模三角网格的引索
	vector<Eigen::Vector3f> Face_normal;
	Matrix_Nx3              Vectices_init;         //手模初始顶点坐标
	Matrix_MxN              Vertices_update_;      //更新后手模的顶点
	Matrix_Nx3              Vertices_normal;       //顶点的法向量
	Matrix_MxN              Weights;

	vector<Eigen::Vector3f> Visible_vertices;                   //可见顶点
	vector<Eigen::Vector2i> Visible_vertices_2D;
	vector<int>             Visible_vertices_index;            //可见顶点的引索          
	Matrix_Nx3              Joint_matrix;                      //这里是将joint*数组中的关节点位置整合到一个矩阵中

	int NumofVertices;
	int NumofFaces;

	float *Params;
	float *init_Params;         //代表手套给的初始关节角度
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
    //加载手模
	void load_faces(char* file);
	void load_vertices(char* file);
	void load_weight(char* file);
	void Load_HandModel();

	//初始化手模
	void set_local_coordinate();
	void set_parent_child_transform();
	void set_params_bound();
	void Init_HandModel();

	//根据姿态更新手模
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


	//计算某一个点的雅各比，包括关节点、带权重的Vector顶点、不带权重的碰撞体顶点
	Eigen::MatrixXf Compute_one_Joint_Jacobian(int index);
	Eigen::MatrixXf Compute_one_Vertice_Jacobian(int index);
	Eigen::MatrixXf Compute_one_CollisionPoint_Jacobian(Collision& a, Eigen::Vector3f& point);

	Eigen::MatrixXf Compute_one_Joint_Shape_Jacobian(int index);
	Eigen::MatrixXf Compute_one_Vertice_Shape_Jacobian(int index);

	//显示参数
	void Print_fingerLength();
	void Save();
};