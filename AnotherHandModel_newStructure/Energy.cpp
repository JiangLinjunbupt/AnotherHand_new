#include "Energy.h"

namespace energy {

	static bool print_solution = false;

	bool Energy::has_nan(LinearSystem& system) {
		for (int i = 0; i<system.Jt_J.rows(); i++) {
			for (int j = 0; j<system.Jt_J.cols(); j++) {
				if (isnan(system.Jt_J(i, j))) {
					std::cout << "--------------------------------------------------------" << std::endl;
					std::cout << "!!!WARNING: NaN DETECTED in the Jt_J!!! (skipping update)" << std::endl;
					std::cout << "--------------------------------------------------------" << std::endl;
					return true;
				}
				if (isinf(system.Jt_J(i, j))) {
					std::cout << "--------------------------------------------------------" << std::endl;
					std::cout << "!!!WARNING: inf DETECTED in the Jt_J!!! (skipping update)" << std::endl;
					std::cout << "--------------------------------------------------------" << std::endl;
					return true;
				}
			}
			if (isnan(system.Jt_e(i))) {
				std::cout << "--------------------------------------------------------" << std::endl;
				std::cout << "!!!WARNING: NaN DETECTED in the Jt_e!!! (skipping update)" << std::endl;
				std::cout << "--------------------------------------------------------" << std::endl;
				return true;
			}
			if (isinf(system.Jt_e(i))) {
				std::cout << "--------------------------------------------------------" << std::endl;
				std::cout << "!!!WARNING: inf DETECTED in the Jt_e!!! (skipping update)" << std::endl;
				std::cout << "--------------------------------------------------------" << std::endl;
				return true;
			}
		}
		return false;
	}

	void Energy::rigid_only(LinearSystem &system) {
		int start = 6; ///< fully rigid

		for (int c = start; c < num_thetas; ++c)
			system.Jt_J.col(c).setZero();
		for (int r = start; r < num_thetas; ++r)
		{
			system.Jt_J.row(r).setZero();
			system.Jt_e.row(r).setZero();
		}
	}

	Eigen::VectorXf Energy::solve(LinearSystem &system) {
		///--- Solve for update dt = (J^T * J + D)^-1 * J^T * r

		//http://eigen.tuxfamily.org/dox/group__LeastSquares.html  for linear least square problem
		Eigen::VectorXf solution = system.Jt_J.colPivHouseholderQr().solve(system.Jt_e);

		///--- Check for NaN
		for (int i = 0; i<solution.size(); i++) {
			if (isnan(solution[i])) {
				std::cout << "-------------------------------------------------------------" << std::endl;
				std::cout << "!!!WARNING: NaN DETECTED in the solution!!! (skipping update)" << std::endl;
				std::cout << "-------------------------------------------------------------" << std::endl;
				return Eigen::VectorXf::Zero(num_thetas, 1);
			}
		}

		if (print_solution)
			std::cout << "delta_thetas: " << solution.transpose()<<std::endl;

		return solution;
	}

} 
