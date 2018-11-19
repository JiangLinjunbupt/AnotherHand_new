#include"Energy_Damping.h"


namespace energy {

	void Energy_Damping::track(LinearSystem &system) {

		Eigen::MatrixXf D = Eigen::MatrixXf::Identity(num_thetas, num_thetas);

		{
			D(0, 0) = settings->translation_damping;
			D(1, 1) = settings->translation_damping;
			D(2, 2) = settings->translation_damping;
			D(3, 3) = settings->rotation_damping;
			D(4, 4) = settings->rotation_damping;
			D(5, 5) = settings->rotation_damping;

			D(6, 6) = settings->rotation_damping;
			D(7, 7) = settings->rotation_damping;
			D(8, 8) = settings->top_phalange_damping;
			D(9, 9) = settings->top_phalange_damping;

			D(10, 10) = settings->rotation_damping;
			D(11, 11) = settings->abduction_damping;
			D(12, 12) = settings->top_phalange_damping;
			D(13, 13) = settings->top_phalange_damping;

			D(14, 14) = settings->rotation_damping;
			D(15, 15) = settings->abduction_damping;
			D(16, 16) = settings->top_phalange_damping;
			D(17, 17) = settings->top_phalange_damping;

			D(18, 18) = settings->rotation_damping;
			D(19, 19) = settings->abduction_damping;
			D(20, 20) = settings->top_phalange_damping;
			D(21, 21) = settings->top_phalange_damping;

			D(22, 22) = settings->rotation_damping;
			D(23, 23) = settings->abduction_damping;
			D(24, 24) = settings->top_phalange_damping;
			D(25, 25) = settings->top_phalange_damping;
		}


		system.Jt_J = system.Jt_J + D;
		if (Energy::safety_check) Energy::has_nan(system);
	}


	void Energy_Damping::track_shape(LinearSystem &system) {

		Eigen::MatrixXf D = Eigen::MatrixXf::Identity(num_shape_thetas, num_shape_thetas);

		//for (int i = 0; i < num_shape_thetas; i++)
		//{
		//	D(i, i) = 1000;
		//}
		system.Jt_J = system.Jt_J + D;
		if (Energy::safety_check) Energy::has_nan(system);
	}
}