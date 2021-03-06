#pragma once
#include"Types.h"


/// The running modes for cameras
enum CAMERAMODE { INVALID = -1, Kinect, Dataset_MSRA_14, Dataset_MSRA_15};

class Camera {
private:
	CAMERAMODE _mode;
	int _width;
	int _height;
	float _focal_length_x = nan();
	float _focal_length_y = nan();
	float centerx;
	float centery;
public:
	Camera(CAMERAMODE mode);
public:
	CAMERAMODE mode() const { return _mode; }
	int width() const { return _width; }
	int height() const { return _height; }
	float focal_length_x() const { return _focal_length_x; }
	float focal_length_y() const { return _focal_length_y; }

public:
	Matrix_2x3 projection_jacobian(const Eigen::Vector3f& p);
public:
	Eigen::Vector3f world_to_depth_image(const Eigen::Vector3f& wrld);
	Eigen::Vector3f depth_to_world(float row_, float col_, float depth);
};