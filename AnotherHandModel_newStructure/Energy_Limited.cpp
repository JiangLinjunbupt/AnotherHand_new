#include "Energy_Limited.h"

namespace energy
{
	void Energy_Limited::init(HandModel * model) {
		this->model = model;
	}


	void Energy_Limited::shape_boundLimited(LinearSystem& sys)
	{
		Eigen::Matrix<float, num_shape_thetas, num_shape_thetas> J_limit = Eigen::Matrix<float, num_shape_thetas, num_shape_thetas>::Zero(num_shape_thetas, num_shape_thetas);
		Eigen::Matrix<float, num_shape_thetas, 1> e_limit = Eigen::Matrix<float, num_shape_thetas, 1>::Zero(num_shape_thetas, 1);

		for (int i = 0; i < num_shape_thetas; ++i)
		{
			float q_max = model->Shape_ParamsUpperBound[i];
			float q_min = model->Shape_ParamsLowerBound[i];

			if (i != 0 && i != 3 && i != 7 && i != 11 && i != 15 && i != 19)
			{
				q_max = model->Shape_Params[0] * q_max;
				q_min = model->Shape_Params[0] * q_min;
			}

			if (model->Shape_Params[i] > q_max) {
				//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
				e_limit(i) = (q_max - model->Shape_Params[i]) - std::numeric_limits<float>::epsilon();
				J_limit(i, i) = 1;
			}
			else if (model->Shape_Params[i] < q_min) {
				//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
				e_limit(i) = (q_min - model->Shape_Params[i]) + std::numeric_limits<float>::epsilon();
				J_limit(i, i) = 1;
			}
			else {
				J_limit(i, i) = 0;
				e_limit(i) = 0;
			}
		}

		///--- Add constraints to the solve
		float omega = 100;
		sys.Jt_J.noalias() += omega * J_limit.transpose() * J_limit;
		sys.Jt_e.noalias() += omega * J_limit.transpose() * e_limit;

		///--- Check
		if (Energy::safety_check)
			Energy::has_nan(sys);
	}

	void Energy_Limited::shape_relativeLimited(LinearSystem& sys)
	{

		Eigen::Matrix<float, num_shape_thetas, num_shape_thetas> J_limit = Eigen::Matrix<float, num_shape_thetas, num_shape_thetas>::Zero(num_shape_thetas, num_shape_thetas);
		Eigen::Matrix<float, num_shape_thetas, 1>  e_limit = Eigen::Matrix<float, num_shape_thetas, 1>::Zero(num_shape_thetas, 1);

		//thumb
		{
			float Length_m, Length_pp, Length_pd;
			Length_m = (model->Joint_matrix.row(2) - model->Joint_matrix.row(1)).norm();
			Length_pp = (model->Joint_matrix.row(3) - model->Joint_matrix.row(2)).norm();
			Length_pd = (model->Joint_matrix.row(4) - model->Joint_matrix.row(3)).norm() - 5.67f*model->Shape_Params[0];

			if (Length_m > 1.76f*Length_pp)
			{
				e_limit(1) = (1.76f*Length_pp - Length_m) - std::numeric_limits<float>::epsilon();
				J_limit(1, 1) = 1;
			}
			else if (Length_m < 1.22f*Length_pp)
			{
				e_limit(1) = (1.22f*Length_pp - Length_m) - std::numeric_limits<float>::epsilon();
				J_limit(1, 1) = 1;
			}
			else
			{
				e_limit(1) = 0;
				J_limit(1, 1) = 0;
			}

			if (Length_pp > 1.73f*Length_pd)
			{
				e_limit(2) = (1.73f*Length_pd - Length_pp) - std::numeric_limits<float>::epsilon();
				J_limit(2, 2) = 1;
			}
			else if (Length_pp < 1.22f*Length_pd)
			{
				e_limit(2) = (1.22f*Length_pd - Length_pp) - std::numeric_limits<float>::epsilon();
				J_limit(2, 2) = 1;
			}
			else
			{
				e_limit(2) = 0;
				J_limit(2, 2) = 0;
			}

		}

		//index
		{
			float Length_m, Length_pp, Length_pm,Length_pd;
			Length_m = (model->Joint_matrix.row(5) - model->Joint_matrix.row(0)).norm() - 20.0f*model->Shape_Params[0];
			Length_pp = (model->Joint_matrix.row(6) - model->Joint_matrix.row(5)).norm();
			Length_pm = (model->Joint_matrix.row(7) - model->Joint_matrix.row(6)).norm();
			Length_pd = (model->Joint_matrix.row(8) - model->Joint_matrix.row(7)).norm() - 3.84f*model->Shape_Params[0];

			if (Length_m > 2.14f*Length_pp)
			{
				e_limit(4) = (2.14f*Length_pp - Length_m) - std::numeric_limits<float>::epsilon();
				J_limit(4, 4) = 1;
			}
			else if (Length_m < 1.38f*Length_pp)
			{
				e_limit(4) = (1.38f*Length_pp - Length_m) - std::numeric_limits<float>::epsilon();
				J_limit(4, 4) = 1;
			}
			else
			{
				e_limit(4) = 0;
				J_limit(4, 4) = 0;
			}

			if (Length_pp > 2.25f*Length_pm)
			{
				e_limit(5) = (2.25f*Length_pm - Length_pp) - std::numeric_limits<float>::epsilon();
				J_limit(5, 5) = 1;
			}
			else if (Length_pp < 1.340f*Length_pm)
			{
				e_limit(5) = (1.340f*Length_pm - Length_pp) - std::numeric_limits<float>::epsilon();
				J_limit(5, 5) = 1;
			}
			else
			{
				e_limit(5) = 0;
				J_limit(5, 5) = 0;
			}

			if (Length_pm > 1.84f*Length_pd)
			{
				e_limit(6) = (1.84f*Length_pd - Length_pm) - std::numeric_limits<float>::epsilon();
				J_limit(6, 6) = 1;
			}
			else if (Length_pm < 1.10f*Length_pd)
			{
				e_limit(6) = (1.10f*Length_pd - Length_pm) - std::numeric_limits<float>::epsilon();
				J_limit(6, 6) = 1;
			}
			else
			{
				e_limit(6) = 0;
				J_limit(6, 6) = 0;
			}
		}

		//middle
		{
			float Length_m, Length_pp, Length_pm, Length_pd;
			Length_m = (model->Joint_matrix.row(9) - model->Joint_matrix.row(0)).norm() - 20.0f*model->Shape_Params[0];
			Length_pp = (model->Joint_matrix.row(10) - model->Joint_matrix.row(9)).norm();
			Length_pm = (model->Joint_matrix.row(11) - model->Joint_matrix.row(10)).norm();
			Length_pd = (model->Joint_matrix.row(12) - model->Joint_matrix.row(11)).norm() - 3.95f*model->Shape_Params[0];

			if (Length_m > 1.71f*Length_pp)
			{
				e_limit(8) = (1.71f*Length_pp - Length_m) - std::numeric_limits<float>::epsilon();
				J_limit(8, 8) = 1;
			}
			else if (Length_m <1.22f*Length_pp)
			{
				e_limit(8) = (1.22f*Length_pp - Length_m) - std::numeric_limits<float>::epsilon();
				J_limit(8, 8) = 1;
			}
			else
			{
				e_limit(8) = 0;
				J_limit(8, 8) = 0;
			}

			if (Length_pp > 2.07f*Length_pm)
			{
				e_limit(9) = (2.07f*Length_pm - Length_pp) - std::numeric_limits<float>::epsilon();
				J_limit(9, 9) = 1;
			}
			else if (Length_pp < 1.39f*Length_pm)
			{
				e_limit(9) = (1.39f*Length_pm - Length_pp) - std::numeric_limits<float>::epsilon();
				J_limit(9, 9) = 1;
			}
			else
			{
				e_limit(9) = 0;
				J_limit(9, 9) = 0;
			}

			if (Length_pm > 1.89f*Length_pd)
			{
				e_limit(10) = (1.89f*Length_pd - Length_pm) - std::numeric_limits<float>::epsilon();
				J_limit(10, 10) = 1;
			}
			else if (Length_pm < 1.21f*Length_pd)
			{
				e_limit(10) = (1.21f*Length_pd - Length_pm) - std::numeric_limits<float>::epsilon();
				J_limit(10, 10) = 1;
			}
			else
			{
				e_limit(10) = 0;
				J_limit(10, 10) = 0;
			}
		}

		//ring
		{
			float Length_m, Length_pp, Length_pm, Length_pd;
			Length_m = (model->Joint_matrix.row(13) - model->Joint_matrix.row(0)).norm() - 20.0f*model->Shape_Params[0];
			Length_pp = (model->Joint_matrix.row(14) - model->Joint_matrix.row(13)).norm();
			Length_pm = (model->Joint_matrix.row(15) - model->Joint_matrix.row(14)).norm();
			Length_pd = (model->Joint_matrix.row(16) - model->Joint_matrix.row(15)).norm() - 3.95f*model->Shape_Params[0];

			if (Length_m > 1.68f*Length_pp)
			{
				e_limit(12) = (1.68f*Length_pp - Length_m) - std::numeric_limits<float>::epsilon();
				J_limit(12, 12) = 1;
			}
			else if (Length_m <1.17f*Length_pp)
			{
				e_limit(12) = (1.17f*Length_pp - Length_m) - std::numeric_limits<float>::epsilon();
				J_limit(12, 12) = 1;
			}
			else
			{
				e_limit(12) = 0;
				J_limit(12, 12) = 0;
			}

			if (Length_pp > 2.02f*Length_pm)
			{
				e_limit(13) = (2.02f*Length_pm - Length_pp) - std::numeric_limits<float>::epsilon();
				J_limit(13, 13) = 1;
			}
			else if (Length_pp < 1.30f*Length_pm)
			{
				e_limit(13) = (1.30f*Length_pm - Length_pp) - std::numeric_limits<float>::epsilon();
				J_limit(13, 13) = 1;
			}
			else
			{
				e_limit(13) = 0;
				J_limit(13, 13) = 0;
			}

			if (Length_pm > 1.91f*Length_pd)
			{
				e_limit(14) = (1.91f*Length_pd - Length_pm) - std::numeric_limits<float>::epsilon();
				J_limit(14, 14) = 1;
			}
			else if (Length_pm < 1.14f*Length_pd)
			{
				e_limit(14) = (1.14f*Length_pd - Length_pm) - std::numeric_limits<float>::epsilon();
				J_limit(14, 14) = 1;
			}
			else
			{
				e_limit(14) = 0;
				J_limit(14, 14) = 0;
			}
		}

		//pinky
		{
			float Length_m, Length_pp, Length_pm, Length_pd;
			Length_m = (model->Joint_matrix.row(17) - model->Joint_matrix.row(0)).norm() - 20.0f*model->Shape_Params[0];
			Length_pp = (model->Joint_matrix.row(18) - model->Joint_matrix.row(17)).norm();
			Length_pm = (model->Joint_matrix.row(19) - model->Joint_matrix.row(18)).norm();
			Length_pd = (model->Joint_matrix.row(20) - model->Joint_matrix.row(19)).norm() - 3.73f*model->Shape_Params[0];

			if (Length_m > 1.93f*Length_pp)
			{
				e_limit(16) = (1.93f*Length_pp - Length_m) - std::numeric_limits<float>::epsilon();
				J_limit(16, 16) = 1;
			}
			else if (Length_m <1.39f*Length_pp)
			{
				e_limit(16) = (1.39f*Length_pp - Length_m) - std::numeric_limits<float>::epsilon();
				J_limit(16, 16) = 1;
			}
			else
			{
				e_limit(16) = 0;
				J_limit(16, 16) = 0;
			}

			if (Length_pp > 2.28f*Length_pm)
			{
				e_limit(17) = (2.28f*Length_pm - Length_pp) - std::numeric_limits<float>::epsilon();
				J_limit(17, 17) = 1;
			}
			else if (Length_pp < 1.45f*Length_pm)
			{
				e_limit(17) = (1.45f*Length_pm - Length_pp) - std::numeric_limits<float>::epsilon();
				J_limit(17, 17) = 1;
			}
			else
			{
				e_limit(17) = 0;
				J_limit(17, 17) = 0;
			}

			if (Length_pm > 1.53f*Length_pd)
			{
				e_limit(18) = (1.53f*Length_pd - Length_pm) - std::numeric_limits<float>::epsilon();
				J_limit(18, 18) = 1;
			}
			else if (Length_pm < 1.11f*Length_pd)
			{
				e_limit(18) = (1.11f*Length_pd - Length_pm) - std::numeric_limits<float>::epsilon();
				J_limit(18, 18) = 1;
			}
			else
			{
				e_limit(18) = 0;
				J_limit(18, 18) = 0;
			}
		}


		///--- Add constraints to the solve
		float omega = 10;
		sys.Jt_J.noalias() += omega * J_limit.transpose() * J_limit;
		sys.Jt_e.noalias() += omega * J_limit.transpose() * e_limit;

		///--- Check
		if (Energy::safety_check)
			Energy::has_nan(sys);
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
		if (!shapelimits_enable) return;

		this->shape_boundLimited(sys);
		this->shape_relativeLimited(sys);
	}
}
