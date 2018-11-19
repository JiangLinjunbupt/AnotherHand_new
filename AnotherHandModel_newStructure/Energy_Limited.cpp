#include "Energy_Limited.h"

namespace energy
{
	void Energy_Limited::init(HandModel * model) {
		this->model = model;
	}

	void Energy_Limited::track(LinearSystem& sys, bool with_glove)
	{
		//cout << "LOW JOINTLIMITS" << endl;
		if (!jointlimits_enable) return;


		Eigen::Matrix<float, num_thetas, num_thetas> J_limit = Eigen::Matrix<float, num_thetas, num_thetas>::Zero(num_thetas, num_thetas);
		Eigen::Matrix<float, num_thetas, 1>  e_limit = Eigen::Matrix<float, num_thetas, 1>::Zero(num_thetas, 1);


		if (with_glove)
		{
			{
				//thumb
				{
					//       6       ------>    Thumb_Low_R_y
					//       7       ------>    Thumb_Low_R_z
					//       8       ------>    Thumb_mid_R_y    //这里注意了，是z不是y了
					//       9       ------>    Thumb_top_R_y    //这里注意了，是z不是y了
					{
						int i = 8;
						float q_max = (model->init_Params[i] + 10) > model->ParamsUpperBound[i] ? model->ParamsUpperBound[i] : (model->init_Params[i] + 10);
						float q_min = (model->init_Params[i] - 10) < model->ParamsLowerBound[i] ? model->ParamsLowerBound[i] : (model->init_Params[i] - 10);
						if (model->Params[i] > q_max) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 9;
						float q_max = (model->init_Params[i] + 20) > model->ParamsUpperBound[i] ? model->ParamsUpperBound[i] : (model->init_Params[i] + 20);
						float q_min = (model->init_Params[i] - 20) < model->ParamsLowerBound[i] ? model->ParamsLowerBound[i] : (model->init_Params[i] - 20);
						if (model->Params[i] > q_max) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}
				}

				//index
				{	//       10      ------>    Index_Low_R_y
					//       11      ------>    Index_Low_R_z
					//       12      ------>    Index_mid_R_y
					//       13      ------>    Index_top_R_y
					{
						int i = 10;
						float q_max = (model->init_Params[i] + 10) > model->ParamsUpperBound[i] ? model->ParamsUpperBound[i] : (model->init_Params[i] + 10);
						float q_min = (model->init_Params[i] - 10) < model->ParamsLowerBound[i] ? model->ParamsLowerBound[i] : (model->init_Params[i] - 10);
						if (model->Params[i] > q_max) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 11;
						float q_max, q_min;
						if (model->Params[i - 1] < 60)
						{
							q_max = model->ParamsUpperBound[i];
							q_min = model->ParamsLowerBound[i];
						}
						else
						{
							q_max = q_min = 0;
						}

						if (model->Params[i] > q_max) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 12;
						float q_max = (model->init_Params[i] + 20) > model->ParamsUpperBound[i] ? model->ParamsUpperBound[i] : (model->init_Params[i] + 20);
						float q_min = (model->init_Params[i] - 20) < model->ParamsLowerBound[i] ? model->ParamsLowerBound[i] : (model->init_Params[i] - 20);
						if (model->Params[i] > q_max) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 13;
						float q_max = (model->Params[i - 1] + 20) > model->ParamsUpperBound[i] ? model->ParamsUpperBound[i] : (model->Params[i - 1] + 20);
						float q_min = model->Params[i - 1] * 0.4f;
						if (model->Params[i] > q_max) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}
				}

				//middle
				{	    //       14      ------>    Middle_Low_R_y
						//       15      ------>    Middle_Low_R_z
						//       16      ------>    Middle_mid_R_y
						//       17      ------>    Middle_top_R_y
					{
						int i = 14;
						float q_max = (model->init_Params[i] + 10) > model->ParamsUpperBound[i] ? model->ParamsUpperBound[i] : (model->init_Params[i] + 10);
						float q_min = (model->init_Params[i] - 10) < model->ParamsLowerBound[i] ? model->ParamsLowerBound[i] : (model->init_Params[i] - 10);
						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 15;
						float q_max, q_min;
						if (model->Params[i - 1] < 60)
						{
							q_max = model->ParamsUpperBound[i];
							q_min = model->ParamsLowerBound[i];
						}
						else
						{
							q_max = q_min = 0;
						}

						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 16;
						float q_max = (model->init_Params[i] + 20) > model->ParamsUpperBound[i] ? model->ParamsUpperBound[i] : (model->init_Params[i] + 20);
						float q_min = (model->init_Params[i] - 20) < model->ParamsLowerBound[i] ? model->ParamsLowerBound[i] : (model->init_Params[i] - 20);
						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 17;
						float q_max = (model->Params[i - 1] + 20) > model->ParamsUpperBound[i] ? model->ParamsUpperBound[i] : (model->Params[i - 1] + 20);
						float q_min = model->Params[i - 1] * 0.4f;
						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}
				}

				//ring
				{

					//       18      ------>    Ring_Low_R_y
					//       19      ------>    Ring_Low_R_z
					//       20      ------>    Ring_mid_R_y
					//       21      ------>    Ring_top_R_y
					{
						int i = 18;
						float q_max = (model->init_Params[i] + 10) > model->ParamsUpperBound[i] ? model->ParamsUpperBound[i] : (model->init_Params[i] + 10);
						float q_min = (model->init_Params[i] - 10) < model->ParamsLowerBound[i] ? model->ParamsLowerBound[i] : (model->init_Params[i] - 10);
						if (model->Params[i] > q_max) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 19;
						float q_max, q_min;
						if (model->Params[i - 1] < 60)
						{
							q_max = model->ParamsUpperBound[i];
							q_min = model->ParamsLowerBound[i];
						}
						else
						{
							q_max = q_min = 0;
						}

						if (model->Params[i] > q_max) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 20;
						float q_max = (model->init_Params[i] + 20) > model->ParamsUpperBound[i] ? model->ParamsUpperBound[i] : (model->init_Params[i] + 20);
						float q_min = (model->init_Params[i] - 20) < model->ParamsLowerBound[i] ? model->ParamsLowerBound[i] : (model->init_Params[i] - 20);
						if (model->Params[i] > q_max) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 21;
						float q_max = (model->Params[i - 1] + 20) > model->ParamsUpperBound[i] ? model->ParamsUpperBound[i] : (model->Params[i - 1] + 20);
						float q_min = model->Params[i - 1] * 0.4f;
						if (model->Params[i] > q_max) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}
				}

				//pinkey
				{

					//       22      ------>    Pinkey_Low_R_y
					//       23      ------>    Pinkey_Low_R_z
					//       24      ------>    Pinkey_mid_R_y
					//       25      ------>    Pinkey_top_R_y
					{
						int i = 22;
						float q_max = (model->init_Params[i] + 10) > model->ParamsUpperBound[i] ? model->ParamsUpperBound[i] : (model->init_Params[i] + 10);
						float q_min = (model->init_Params[i] - 10) < model->ParamsLowerBound[i] ? model->ParamsLowerBound[i] : (model->init_Params[i] - 10);
						if (model->Params[i] > q_max) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 23;
						float q_max, q_min;
						if (model->Params[i - 1] < 60)
						{
							q_max = model->ParamsUpperBound[i];
							q_min = model->ParamsLowerBound[i];
						}
						else
						{
							q_max = q_min = 0;
						}

						if (model->Params[i] > q_max) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 24;
						float q_max = (model->init_Params[i] + 20) > model->ParamsUpperBound[i] ? model->ParamsUpperBound[i] : (model->init_Params[i] + 20);
						float q_min = (model->init_Params[i] - 20) < model->ParamsLowerBound[i] ? model->ParamsLowerBound[i] : (model->init_Params[i] - 20);
						if (model->Params[i] > q_max) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 25;
						float q_max = (model->Params[i - 1] + 20) > model->ParamsUpperBound[i] ? model->ParamsUpperBound[i] : (model->Params[i - 1] + 20);
						float q_min = model->Params[i - 1] * 0.4f;
						if (model->Params[i] > q_max) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}
				}
			}
		}
		else
		{
			{
				//thumb
				{
					//       6       ------>    Thumb_Low_R_y
					//       7       ------>    Thumb_Low_R_z
					//       8       ------>    Thumb_mid_R_y    //这里注意了，是z不是y了
					//       9       ------>    Thumb_top_R_y    //这里注意了，是z不是y了
					{
						int i = 8;
						float q_max = model->ParamsUpperBound[i];
						float q_min = model->ParamsLowerBound[i];
						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 9;
						float q_max = model->ParamsUpperBound[i] ;
						float q_min = model->ParamsLowerBound[i] ;
						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}
				}

				//index
				{	//       10      ------>    Index_Low_R_y
					//       11      ------>    Index_Low_R_z
					//       12      ------>    Index_mid_R_y
					//       13      ------>    Index_top_R_y
					{
						int i = 10;
						float q_max = model->ParamsUpperBound[i];
						float q_min = model->ParamsLowerBound[i];
						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 11;
						float q_max, q_min;
						if (model->Params[i - 1] < 60)
						{
							q_max = model->ParamsUpperBound[i];
							q_min = model->ParamsLowerBound[i];
						}
						else
						{
							q_max = q_min = 0;
						}

						if (model->Params[i] > q_max) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 12;
						float q_max = model->ParamsUpperBound[i];
						float q_min = model->ParamsLowerBound[i];
						if (model->Params[i] > q_max) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 13;
						float q_max = (model->Params[i - 1] + 20) > model->ParamsUpperBound[i] ? model->ParamsUpperBound[i] : (model->Params[i - 1] + 20);
						float q_min = model->Params[i - 1] * 0.4f;
						if (model->Params[i] > q_max) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}
				}

				//middle
				{	    //       14      ------>    Middle_Low_R_y
						//       15      ------>    Middle_Low_R_z
						//       16      ------>    Middle_mid_R_y
						//       17      ------>    Middle_top_R_y
					{
						int i = 14;
						float q_max = model->ParamsUpperBound[i];
						float q_min = model->ParamsLowerBound[i];
						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 15;
						float q_max, q_min;
						if (model->Params[i - 1] < 60)
						{
							q_max = model->ParamsUpperBound[i];
							q_min = model->ParamsLowerBound[i];
						}
						else
						{
							q_max = q_min = 0;
						}

						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 16;
						float q_max = model->ParamsUpperBound[i];
						float q_min = model->ParamsLowerBound[i];
						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 17;
						float q_max = (model->Params[i - 1] + 20) > model->ParamsUpperBound[i] ? model->ParamsUpperBound[i] : (model->Params[i - 1] + 20);
						float q_min = model->Params[i - 1] * 0.4f;
						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}
				}

				//ring
				{

					//       18      ------>    Ring_Low_R_y
					//       19      ------>    Ring_Low_R_z
					//       20      ------>    Ring_mid_R_y
					//       21      ------>    Ring_top_R_y
					{
						int i = 18;
						float q_max = model->ParamsUpperBound[i];
						float q_min = model->ParamsLowerBound[i];
						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 19;
						float q_max, q_min;
						if (model->Params[i - 1] < 60)
						{
							q_max = model->ParamsUpperBound[i];
							q_min = model->ParamsLowerBound[i];
						}
						else
						{
							q_max = q_min = 0;
						}

						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 20;
						float q_max =model->ParamsUpperBound[i];
						float q_min =model->ParamsLowerBound[i];
						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 21;
						float q_max = (model->Params[i - 1] + 20) > model->ParamsUpperBound[i] ? model->ParamsUpperBound[i] : (model->Params[i - 1] + 20);
						float q_min = model->Params[i - 1] * 0.4f;
						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}
				}

				//pinkey
				{

					//       22      ------>    Pinkey_Low_R_y
					//       23      ------>    Pinkey_Low_R_z
					//       24      ------>    Pinkey_mid_R_y
					//       25      ------>    Pinkey_top_R_y
					{
						int i = 22;
						float q_max = model->ParamsUpperBound[i];
						float q_min = model->ParamsLowerBound[i];
						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 23;
						float q_max, q_min;
						if (model->Params[i - 1] < 60)
						{
							q_max = model->ParamsUpperBound[i];
							q_min = model->ParamsLowerBound[i];
						}
						else
						{
							q_max = q_min = 0;
						}

						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 24;
						float q_max = model->ParamsUpperBound[i];
						float q_min = model->ParamsLowerBound[i];
						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}

					{
						int i = 25;
						float q_max = (model->Params[i - 1] + 20) > model->ParamsUpperBound[i] ? model->ParamsUpperBound[i] : (model->Params[i - 1] + 20);
						float q_min = model->Params[i - 1] * 0.4f;
						if (model->Params[i] > q_max) {
							e_limit(i) = (q_max - model->Params[i]) - std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else if (model->Params[i] < q_min) {
							e_limit(i) = (q_min - model->Params[i]) + std::numeric_limits<float>::epsilon();
							J_limit(i, i) = 1;
						}
						else {
							J_limit(i, i) = 0;
							e_limit(i) = 0;
						}
					}
				}
			}
		}


		///--- Add constraints to the solve
		float omega = jointlimits_weight;
		sys.Jt_J.noalias() += omega * J_limit.transpose() * J_limit;
		sys.Jt_e.noalias() += omega * J_limit.transpose() * e_limit;

		///--- Check
		if (Energy::safety_check)
			Energy::has_nan(sys);
	}

	void Energy_Limited::track_shape(LinearSystem& sys)
	{

	}
}
