#include "Camera.h"


//camera坐标系
//              |y
//              |
//              |________x              和Kinect的摄像头世界坐标系一样（第三人称视角）
//              /                       但是要注意个点，Kinec出来的深度图是：   O----------x
//           z /                                                                |                  |y
//                                                                              |y                 |
//                                      因此，要对深度图进行cv::filp(img,img,0)绕x轴翻转，得到     O-------x这样的坐标之后，再进行转世界坐标系
Camera::Camera() {
	//kinect 2.0 的深度空间的高*宽是 424 * 512，在官网上有说明
	_width = 512;
	_height = 424;

	_focal_length_x = 381.8452f;
	_focal_length_y = 382.1713f;

	centerx = 264.0945f;
	centery = 217.1487f;

}


Eigen::Vector3f Camera::depth_to_world(float row_, float col_, float depth) {

	Eigen::Vector3f world;

	world(0) = (col_ - centerx) * (depth) / _focal_length_x;
	world(1) = (row_ - centery) * (depth) / _focal_length_y;
	world(2) = depth;

	return world;
}

Eigen::Vector3f Camera::world_to_depth_image(const Eigen::Vector3f& world) {
	float x = world[0] / world[2];
	float y = world[1] / world[2];
	x = x*_focal_length_x + centerx;
	y = y*_focal_length_y + centery;
	return Eigen::Vector3f(x, y, world[2]);
}

Matrix_2x3 Camera::projection_jacobian(const Eigen::Vector3f &p) {
	Matrix_2x3 M;
	M << _focal_length_x / p.z(), 0, -p.x() * _focal_length_x / (p.z()*p.z()),
		0, _focal_length_y / p.z(), -p.y() * _focal_length_y / (p.z()*p.z());
	return M;
}

