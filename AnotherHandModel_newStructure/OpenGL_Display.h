#pragma once
#include <GL\freeglut.h>
#include"Worker.h"
#include<time.h>


namespace DS {

	struct Control {
		int x;
		int y;
		bool mouse_click;
		GLfloat rotx;
		GLfloat roty;
		double gx;
		double gy;
		double gz;

		Control() :x(0), y(0), rotx(0.0), roty(0.0), mouse_click(false),
			gx(0), gy(0), gz(0) {

		}
	};

	enum DisplayMode
	{
		MESH,
		Skeleton,
		WIRE_FRAME,
	};

	int itr = 0;
	Worker *worker;
	HandModel *handmodel;
	bool with_Kinect = false;
	bool with_glove = false;
	bool watch_result = false;
	bool shapeTrack = false;
	//绘制的模型类型
	DisplayMode displayMode = DisplayMode(MESH);
	//测量时间
	clock_t  Begin, End;
	double duration;
	//共享内存的指针
	float *GetSharedMemeryPtr;
	//鼠标控制
	Control control;


	//定义光照
	void light() {
		glEnable(GL_LIGHTING);
		glEnable(GL_NORMALIZE);
		// 定义太阳光源，它是一种白色的光源  
		GLfloat sun_light_position[] = { 0.0f, 0.0f, 0.0f, 1.0f };
		GLfloat sun_light_ambient[] = { 0.25f, 0.25f, 0.15f, 1.0f };
		GLfloat sun_light_diffuse[] = { 0.7f, 0.7f, 0.55f, 1.0f };
		GLfloat sun_light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };

		glLightfv(GL_LIGHT0, GL_POSITION, sun_light_position); //指定第0号光源的位置   
		glLightfv(GL_LIGHT0, GL_AMBIENT, sun_light_ambient); //GL_AMBIENT表示各种光线照射到该材质上，  
															 //经过很多次反射后最终遗留在环境中的光线强度（颜色）  
		glLightfv(GL_LIGHT0, GL_DIFFUSE, sun_light_diffuse); //漫反射后~~  
		glLightfv(GL_LIGHT0, GL_SPECULAR, sun_light_specular);//镜面反射后~~~  

		glEnable(GL_LIGHT0); //使用第0号光照   
	}

	//定义窗口大小重新调整
	void reshape(int width, int height) {

		GLfloat fieldOfView = 90.0f;
		glViewport(0, 0, (GLsizei)width, (GLsizei)height);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(fieldOfView, (GLfloat)width / (GLfloat)height, 0.1, 500.0);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
	}

	//键盘鼠标响应函数
	void keyboardDown(unsigned char key, int x, int y) {

		if (key == 'q') exit(0);
		if (key == 'm')
		{
			switch (displayMode)
			{
			case MESH:
				displayMode = WIRE_FRAME;
				break;
			case WIRE_FRAME:
				displayMode = Skeleton;
				break;
			case Skeleton:
				displayMode = MESH;
				break;
			default:
				break;
			}
		}
		if (key == 'b') with_Kinect = !with_Kinect;
		if (key == 'w')
		{
			watch_result = !watch_result;
			itr = 0;
		}
		if (key == 's')
		{
			//worker->save_target_vertices();
			shapeTrack = !shapeTrack;
		}
	}
	void mouseClick(int button, int state, int x, int y) {
		control.mouse_click = 1;
		control.x = x;
		control.y = y;
	}
	void mouseMotion(int x, int y) {
		control.rotx = (x - control.x)*0.05f;
		control.roty = (y - control.y)*0.05f;

		//cout<< control.rotx <<" " << control.roty << endl;
		glutPostRedisplay();
	}


	//一系列绘制函数
#pragma region A_Set_of_Draw_Funtion
	void draw_mesh()
	{
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		{
			glColor3f(0.4f, 0.8f, 0.3f);
			glBegin(GL_TRIANGLES);
			for (int i = 0; i < handmodel->NumofFaces; ++i)
			{

				glNormal3f(handmodel->Face_normal[i](0), handmodel->Face_normal[i](1), handmodel->Face_normal[i](2));

				glVertex3f(handmodel->Vertices_update_(handmodel->FaceIndex(i, 0), 0), handmodel->Vertices_update_(handmodel->FaceIndex(i, 0), 1), handmodel->Vertices_update_(handmodel->FaceIndex(i, 0), 2));
				glVertex3f(handmodel->Vertices_update_(handmodel->FaceIndex(i, 1), 0), handmodel->Vertices_update_(handmodel->FaceIndex(i, 1), 1), handmodel->Vertices_update_(handmodel->FaceIndex(i, 1), 2));
				glVertex3f(handmodel->Vertices_update_(handmodel->FaceIndex(i, 2), 0), handmodel->Vertices_update_(handmodel->FaceIndex(i, 2), 1), handmodel->Vertices_update_(handmodel->FaceIndex(i, 2), 2));
			}
			glEnd();
		}
	}
	void draw_WireHand()
	{
		glDisable(GL_LIGHT0);
		glDisable(GL_LIGHTING);
		{
			for (int i = 0; i < handmodel->NumofFaces; i++)
			{
				glLineWidth(1);
				glColor3f(0.4, 0.8, 0.3);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Vertices_update_(handmodel->FaceIndex(i, 0), 0), handmodel->Vertices_update_(handmodel->FaceIndex(i, 0), 1), handmodel->Vertices_update_(handmodel->FaceIndex(i, 0), 2));
				glVertex3f(handmodel->Vertices_update_(handmodel->FaceIndex(i, 1), 0), handmodel->Vertices_update_(handmodel->FaceIndex(i, 1), 1), handmodel->Vertices_update_(handmodel->FaceIndex(i, 1), 2));
				glEnd();

				glLineWidth(1);
				glColor3f(0.4, 0.8, 0.3);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Vertices_update_(handmodel->FaceIndex(i, 0), 0), handmodel->Vertices_update_(handmodel->FaceIndex(i, 0), 1), handmodel->Vertices_update_(handmodel->FaceIndex(i, 0), 2));
				glVertex3f(handmodel->Vertices_update_(handmodel->FaceIndex(i, 2), 0), handmodel->Vertices_update_(handmodel->FaceIndex(i, 2), 1), handmodel->Vertices_update_(handmodel->FaceIndex(i, 2), 2));
				glEnd();

				glLineWidth(1);
				glColor3f(0.4, 0.8, 0.3);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Vertices_update_(handmodel->FaceIndex(i, 1), 0), handmodel->Vertices_update_(handmodel->FaceIndex(i, 1), 1), handmodel->Vertices_update_(handmodel->FaceIndex(i, 1), 2));
				glVertex3f(handmodel->Vertices_update_(handmodel->FaceIndex(i, 2), 0), handmodel->Vertices_update_(handmodel->FaceIndex(i, 2), 1), handmodel->Vertices_update_(handmodel->FaceIndex(i, 2), 2));
				glEnd();
			}
		}
	}
	void draw_skeleton()
	{
		glDisable(GL_LIGHT0);
		glDisable(GL_LIGHTING);
		for (int i = 0; i < handmodel->NumofJoints; ++i) {
			//画点开始 
			glColor3f(1.0, 0.0, 0.0);
			glPushMatrix();
			glTranslatef(handmodel->Joints[i].CorrespondingPosition(0), handmodel->Joints[i].CorrespondingPosition(1), handmodel->Joints[i].CorrespondingPosition(2));
			glutSolidSphere(3, 31, 10);
			glPopMatrix();
			//画点结束，使用push和popmatrix是因为保证每个关节点的偏移都是相对于全局坐标中心点做的变换。
		}

		//画每一个手指
		{
			//画拇指
			{
				glLineWidth(5);
				glColor3f(1.0, 0.0, 0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[0].CorrespondingPosition(0), handmodel->Joints[0].CorrespondingPosition(1), handmodel->Joints[0].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[1].CorrespondingPosition(0), handmodel->Joints[1].CorrespondingPosition(1), handmodel->Joints[1].CorrespondingPosition(2));
				glEnd();


				glLineWidth(5);
				glColor3f(1.0, 0.0, 0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[1].CorrespondingPosition(0), handmodel->Joints[1].CorrespondingPosition(1), handmodel->Joints[1].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[2].CorrespondingPosition(0), handmodel->Joints[2].CorrespondingPosition(1), handmodel->Joints[2].CorrespondingPosition(2));
				glEnd();

				glLineWidth(5);
				glColor3f(1.0, 0.0, 0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[2].CorrespondingPosition(0), handmodel->Joints[2].CorrespondingPosition(1), handmodel->Joints[2].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[3].CorrespondingPosition(0), handmodel->Joints[3].CorrespondingPosition(1), handmodel->Joints[3].CorrespondingPosition(2));
				glEnd();

				glLineWidth(5);
				glColor3f(1.0, 0.0, 0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[3].CorrespondingPosition(0), handmodel->Joints[3].CorrespondingPosition(1), handmodel->Joints[3].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[4].CorrespondingPosition(0), handmodel->Joints[4].CorrespondingPosition(1), handmodel->Joints[4].CorrespondingPosition(2));
				glEnd();
			}

			//画食指
			{
				glLineWidth(5);
				glColor3f(0.0, 1.0, 0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[0].CorrespondingPosition(0), handmodel->Joints[0].CorrespondingPosition(1), handmodel->Joints[0].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[5].CorrespondingPosition(0), handmodel->Joints[5].CorrespondingPosition(1), handmodel->Joints[5].CorrespondingPosition(2));
				glEnd();

				glLineWidth(5);
				glColor3f(0.0, 1.0, 0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[5].CorrespondingPosition(0), handmodel->Joints[5].CorrespondingPosition(1), handmodel->Joints[5].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[6].CorrespondingPosition(0), handmodel->Joints[6].CorrespondingPosition(1), handmodel->Joints[6].CorrespondingPosition(2));
				glEnd();

				glLineWidth(5);
				glColor3f(0.0, 1.0, 0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[6].CorrespondingPosition(0), handmodel->Joints[6].CorrespondingPosition(1), handmodel->Joints[6].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[7].CorrespondingPosition(0), handmodel->Joints[7].CorrespondingPosition(1), handmodel->Joints[7].CorrespondingPosition(2));
				glEnd();

				glLineWidth(5);
				glColor3f(0.0, 1.0, 0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[7].CorrespondingPosition(0), handmodel->Joints[7].CorrespondingPosition(1), handmodel->Joints[7].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[8].CorrespondingPosition(0), handmodel->Joints[8].CorrespondingPosition(1), handmodel->Joints[8].CorrespondingPosition(2));
				glEnd();
			}

			//画中指
			{
				glLineWidth(5);
				glColor3f(0.0, 0.0, 1.0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[0].CorrespondingPosition(0), handmodel->Joints[0].CorrespondingPosition(1), handmodel->Joints[0].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[9].CorrespondingPosition(0), handmodel->Joints[9].CorrespondingPosition(1), handmodel->Joints[9].CorrespondingPosition(2));
				glEnd();


				glLineWidth(5);
				glColor3f(0.0, 0.0, 1.0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[9].CorrespondingPosition(0), handmodel->Joints[9].CorrespondingPosition(1), handmodel->Joints[9].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[10].CorrespondingPosition(0), handmodel->Joints[10].CorrespondingPosition(1), handmodel->Joints[10].CorrespondingPosition(2));
				glEnd();

				glLineWidth(5);
				glColor3f(0.0, 0.0, 1.0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[10].CorrespondingPosition(0), handmodel->Joints[10].CorrespondingPosition(1), handmodel->Joints[10].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[11].CorrespondingPosition(0), handmodel->Joints[11].CorrespondingPosition(1), handmodel->Joints[11].CorrespondingPosition(2));
				glEnd();

				glLineWidth(5);
				glColor3f(0.0, 0.0, 1.0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[11].CorrespondingPosition(0), handmodel->Joints[11].CorrespondingPosition(1), handmodel->Joints[11].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[12].CorrespondingPosition(0), handmodel->Joints[12].CorrespondingPosition(1), handmodel->Joints[12].CorrespondingPosition(2));
				glEnd();
			}

			//画无名指
			{
				glLineWidth(5);
				glColor3f(1.0, 1.0, 0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[0].CorrespondingPosition(0), handmodel->Joints[0].CorrespondingPosition(1), handmodel->Joints[0].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[13].CorrespondingPosition(0), handmodel->Joints[13].CorrespondingPosition(1), handmodel->Joints[13].CorrespondingPosition(2));
				glEnd();


				glLineWidth(5);
				glColor3f(1.0, 1.0, 0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[13].CorrespondingPosition(0), handmodel->Joints[13].CorrespondingPosition(1), handmodel->Joints[13].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[14].CorrespondingPosition(0), handmodel->Joints[14].CorrespondingPosition(1), handmodel->Joints[14].CorrespondingPosition(2));
				glEnd();

				glLineWidth(5);
				glColor3f(1.0, 1.0, 0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[14].CorrespondingPosition(0), handmodel->Joints[14].CorrespondingPosition(1), handmodel->Joints[14].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[15].CorrespondingPosition(0), handmodel->Joints[15].CorrespondingPosition(1), handmodel->Joints[15].CorrespondingPosition(2));
				glEnd();

				glLineWidth(5);
				glColor3f(1.0, 1.0, 0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[15].CorrespondingPosition(0), handmodel->Joints[15].CorrespondingPosition(1), handmodel->Joints[15].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[16].CorrespondingPosition(0), handmodel->Joints[16].CorrespondingPosition(1), handmodel->Joints[16].CorrespondingPosition(2));
				glEnd();
			}

			//画小指
			{
				glLineWidth(5);
				glColor3f(1.0, 0.0, 1.0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[0].CorrespondingPosition(0), handmodel->Joints[0].CorrespondingPosition(1), handmodel->Joints[0].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[17].CorrespondingPosition(0), handmodel->Joints[17].CorrespondingPosition(1), handmodel->Joints[17].CorrespondingPosition(2));
				glEnd();


				glLineWidth(5);
				glColor3f(1.0, 0.0, 1.0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[17].CorrespondingPosition(0), handmodel->Joints[17].CorrespondingPosition(1), handmodel->Joints[17].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[18].CorrespondingPosition(0), handmodel->Joints[18].CorrespondingPosition(1), handmodel->Joints[18].CorrespondingPosition(2));
				glEnd();

				glLineWidth(5);
				glColor3f(1.0, 0.0, 1.0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[18].CorrespondingPosition(0), handmodel->Joints[18].CorrespondingPosition(1), handmodel->Joints[18].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[19].CorrespondingPosition(0), handmodel->Joints[19].CorrespondingPosition(1), handmodel->Joints[19].CorrespondingPosition(2));
				glEnd();

				glLineWidth(5);
				glColor3f(1.0, 0.0, 1.0);
				glBegin(GL_LINES);
				glVertex3f(handmodel->Joints[19].CorrespondingPosition(0), handmodel->Joints[19].CorrespondingPosition(1), handmodel->Joints[19].CorrespondingPosition(2));
				glVertex3f(handmodel->Joints[20].CorrespondingPosition(0), handmodel->Joints[20].CorrespondingPosition(1), handmodel->Joints[20].CorrespondingPosition(2));
				glEnd();
			}
		}
	}

	void draw_vertex()
	{
		glDisable(GL_LIGHT0);
		glDisable(GL_LIGHTING);
		glPointSize(2);
		glBegin(GL_POINTS);
		glColor3d(1.0, 1.0, 0.0);
		for (int i = 0; i < handmodel->NumofVertices; ++i) {
			glVertex3d(handmodel->Vertices_update_(i, 0), handmodel->Vertices_update_(i, 1), handmodel->Vertices_update_(i, 2));
		}
		glEnd();
	}
	void draw_Hand_visible_vertex()
	{
		glDisable(GL_LIGHT0);
		glDisable(GL_LIGHTING);
		glPointSize(2);
		glBegin(GL_POINTS);
		glColor3d(1.0, 1.0, 0.0);
		for (int i = 0; i < handmodel->Visible_vertices.size(); i++) {
			glVertex3d(handmodel->Visible_vertices[i](0), handmodel->Visible_vertices[i](1), handmodel->Visible_vertices[i](2));
		}
		glEnd();
	}
	
	void draw_joint_local_coordinate(int index)
	{
		glDisable(GL_LIGHT0);
		glDisable(GL_LIGHTING);
		int scale = 30;

		Vector4 x = handmodel->Joints[index].CorrespondingAxis_for_shape[0] - handmodel->Joints[index].CorrespondingPosition;
		glLineWidth(2);
		glColor3f(1.0, 0.0, 0);
		glBegin(GL_LINES);
		glVertex3f(handmodel->Joints[index].CorrespondingPosition(0), handmodel->Joints[index].CorrespondingPosition(1), handmodel->Joints[index].CorrespondingPosition(2));
		glVertex3f(handmodel->Joints[index].CorrespondingPosition(0) + scale*x(0), handmodel->Joints[index].CorrespondingPosition(1) + scale * x(1), handmodel->Joints[index].CorrespondingPosition(2) + scale * x(2));
		glEnd();


		Vector4 y = handmodel->Joints[index].CorrespondingAxis_for_shape[1] - handmodel->Joints[index].CorrespondingPosition;
		glLineWidth(2);
		glColor3f(0.0, 1.0, 0);
		glBegin(GL_LINES);
		glVertex3f(handmodel->Joints[index].CorrespondingPosition(0), handmodel->Joints[index].CorrespondingPosition(1), handmodel->Joints[index].CorrespondingPosition(2));
		glVertex3f(handmodel->Joints[index].CorrespondingPosition(0) + scale * y(0), handmodel->Joints[index].CorrespondingPosition(1) + scale * y(1), handmodel->Joints[index].CorrespondingPosition(2) + scale * y(2));
		glEnd();

		Vector4 z = handmodel->Joints[index].CorrespondingAxis_for_shape[2] - handmodel->Joints[index].CorrespondingPosition;
		glLineWidth(2);
		glColor3f(0.0, 0.0, 1.0);
		glBegin(GL_LINES);
		glVertex3f(handmodel->Joints[index].CorrespondingPosition(0), handmodel->Joints[index].CorrespondingPosition(1), handmodel->Joints[index].CorrespondingPosition(2));
		glVertex3f(handmodel->Joints[index].CorrespondingPosition(0) + scale * z(0), handmodel->Joints[index].CorrespondingPosition(1) + scale * z(1), handmodel->Joints[index].CorrespondingPosition(2) + scale * z(2));
		glEnd();
	}
	void draw_ALL_joint_coordinate()
	{
		for (int i = 0; i<21; ++i)  	draw_joint_local_coordinate(i);
	}

	void draw_Jointtarget_difference()
	{
		glDisable(GL_LIGHT0);
		glDisable(GL_LIGHTING);
		for (int i = 0; i < handmodel->NumofJoints; ++i) {
			//画点开始 
			glColor3f(0.0, 1.0, 0.0);
			glPushMatrix();
			glTranslatef(worker->Target_joints(i, 0), worker->Target_joints(i, 1), worker->Target_joints(i, 2));
			glutSolidSphere(3, 31, 10);
			glPopMatrix();
			//画点结束，使用push和popmatrix是因为保证每个关节点的偏移都是相对于全局坐标中心点做的变换。

			//画线开始  //不画wrist到arm的那条线

			glLineWidth(5);
			glColor3f(1.0, 1.0, 1.0);
			glBegin(GL_LINES);
			glVertex3f(handmodel->Joints[i].CorrespondingPosition(0), handmodel->Joints[i].CorrespondingPosition(1), handmodel->Joints[i].CorrespondingPosition(2));
			glVertex3f(worker->Target_joints(i, 0), worker->Target_joints(i, 1), worker->Target_joints(i, 2));
			glEnd();

		}
	}
	void draw_Verticestarget_difference()
	{
		glDisable(GL_LIGHT0);
		glDisable(GL_LIGHTING);
		for (int i = 0; i < handmodel->NumofVertices; ++i) {
			glLineWidth(2);
			glColor3f(1.0, 1.0, 1);
			glBegin(GL_LINES);
			glVertex3d(worker->Target_vertices(i,0), worker->Target_vertices(i, 1), worker->Target_vertices(i, 2));
			glVertex3d(handmodel->Vertices_update_(i,0),
				handmodel->Vertices_update_(i, 1),
				handmodel->Vertices_update_(i, 2));
			glEnd();

		}
	}

	void draw_CloudPoint()
	{
		if (worker->Downsample_pointcloud.points.size() > 0)
		{
			glDisable(GL_LIGHT0);
			glDisable(GL_LIGHTING);
			glPointSize(2);
			glBegin(GL_POINTS);
			glColor3f(1.0f, 0.0f, 0.0f);
			for (int i = 0; i < worker->Downsample_pointcloud.points.size(); i++) {
				glVertex3d(worker->Downsample_pointcloud.points[i].x, worker->Downsample_pointcloud.points[i].y, worker->Downsample_pointcloud.points[i].z);
			}
			glEnd();
		}
	}
	void draw_Cooresponding_connection()
	{
		glDisable(GL_LIGHT0);
		glDisable(GL_LIGHTING);
		if (worker->Downsample_pointcloud.points.size() > 0 && worker->E_fitting.cloud_correspond.size() > 0)
		{
			for (int i = 0; i < worker->Downsample_pointcloud.points.size(); i++)
			{
				glLineWidth(2);
				glColor3f(1.0, 1.0, 1);
				glBegin(GL_LINES);
				glVertex3d(worker->Downsample_pointcloud.points[i].x, worker->Downsample_pointcloud.points[i].y, worker->Downsample_pointcloud.points[i].z);
				glVertex3d(worker->E_fitting.Handmodel_visible_cloud.points[worker->E_fitting.cloud_correspond[i]].x,
					worker->E_fitting.Handmodel_visible_cloud.points[worker->E_fitting.cloud_correspond[i]].y,
					worker->E_fitting.Handmodel_visible_cloud.points[worker->E_fitting.cloud_correspond[i]].z);
				glEnd();
			}
		}
	}

	void draw_Coordinate()
	{
		glDisable(GL_LIGHT0);
		glDisable(GL_LIGHTING);
		//x
		glLineWidth(5);
		glColor3f(1.0, 0.0, 0.0);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(100, 0, 0);
		glEnd();

		//y
		glLineWidth(5);
		glColor3f(0.0, 1.0, 0.0);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 100, 0);
		glEnd();

		//z
		glLineWidth(5);
		glColor3f(0.0, 0.0, 1.0);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 0, 100);
		glEnd();
	}

#pragma endregion A_Set_of_Draw_Funtion

	void draw() {

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glMatrixMode(GL_MODELVIEW);
		gluPerspective(180, 1.5, -1000, 1000);
		glLoadIdentity();
		control.gx = handmodel->Params[0];
		control.gy = handmodel->Params[1];
		control.gz = handmodel->Params[2];
		double r = 210;
		double x = r*cos(control.roty)*sin(control.rotx);
		double y = r*sin(control.roty);
		double z = r*cos(control.roty)*cos(control.rotx);
		//cout<< x <<" "<< y <<" " << z<<endl;
		gluLookAt(x + control.gx, y + control.gy, z + control.gz, control.gx, control.gy, control.gz, 0.0, 1.0, 0.0);//个人理解最开始是看向-z的，之后的角度是在global中心上叠加的，所以要加

		switch (displayMode)
		{
		case MESH:
			draw_mesh();
			break;
		case WIRE_FRAME:
			draw_WireHand();
			break;
		case Skeleton:
			draw_skeleton();
			break;
		default:
			break;
		}

	    //draw_Verticestarget_difference();
		draw_Coordinate();
		//draw_ALL_joint_coordinate();
		//draw_Hand_visible_vertex();
		draw_CloudPoint();
		draw_Cooresponding_connection();
		glFlush();
		glutSwapBuffers();
	}

	void idle() {

		//Begin = clock();//开始计时
		if (!watch_result)
		{
			if (itr == 0)
			{
				for (int i = 0; i < 26; ++i) handmodel->Params[i] = GetSharedMemeryPtr[i];

				handmodel->Params[0] = worker->Downsample_pointcloud_center_x;
				handmodel->Params[1] = worker->Downsample_pointcloud_center_y;
				handmodel->Params[2] = worker->Downsample_pointcloud_center_z;

				for (int i = 0; i < num_shape_thetas; i++)
				{
					handmodel->Shape_Params[i] = 0;
				}


				handmodel->Shape_Params[0] = 0.7f;
				handmodel->Shape_Params[3] = 1.0f;
				handmodel->Shape_Params[7] = 1.0f;
				handmodel->Shape_Params[11] = 1.0f;
				handmodel->Shape_Params[15] = 1.0f;
				handmodel->Shape_Params[19] = 1.0f;

				//handmodel->Shape_Params[4] = -10.0f;
				//handmodel->Shape_Params[8] = -11.0f;
				//handmodel->Shape_Params[12] = -11.0f;
				//handmodel->Shape_Params[16] = -11.0f;

				handmodel->Updata(handmodel->Params,handmodel->Shape_Params);
			}

			if (with_Kinect)
			{
				worker->track_till_convergence(with_glove, shapeTrack);
				itr++;
				//watch_result = true;
			}

		}
		//End = clock();//结束计时
		//duration = double(End - Begin) / CLK_TCK;//duration就是运行函数所打的
		//std::cout << "time is : " << duration * 1000 << std::endl;

		glutPostRedisplay();
	}


	//GL初始化函数
	void InitializeGlutCallbacks()
	{
		glutKeyboardFunc(keyboardDown);
		glutMouseFunc(mouseClick);
		glutMotionFunc(mouseMotion);
		glutReshapeFunc(reshape);
		glutDisplayFunc(draw);
		glutIdleFunc(idle);
		glutIgnoreKeyRepeat(true); // ignore keys held down
	}
	void initScene(int width, int height) {
		reshape(width, height);

		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		glClearDepth(1.0f);

		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LEQUAL);
		light();
	}
	void init(int argc, char* argv[]) {
		// 初始化GLUT
		glutInit(&argc, argv);

		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
		glutInitWindowSize(800, 600);
		glutInitWindowPosition(100, 100);
		glutCreateWindow("SHOW RESULT");
		InitializeGlutCallbacks();
		initScene(800, 600);
	}


	void start() {
		// 通知开始GLUT的内部循环
		glutMainLoop();
	}

}