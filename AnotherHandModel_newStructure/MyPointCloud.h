#pragma once
#include"Types.h"
#include<vector>
#include"Camera.h"

using namespace std;


class MyPointCloud
{

public:
	Camera * camera;

	MyPointCloud() {}
	~MyPointCloud() {}

	void DepthMatToPointCloud(int *indicator,int Num_indicator,InputDataForTrack& inputdatafortrack);

private:
	void Filter_visible_cloud(InputDataForTrack& inputdatafortrack);
	void downSample(InputDataForTrack& inputdatafortrack);
	void RestrictGlobalPosition(InputDataForTrack& inputdatafortrack,int max);
};