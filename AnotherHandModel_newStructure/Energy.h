#pragma once
#include"Types.h"

namespace energy
{
	class Energy {
	protected:
		bool safety_check = false;
		bool has_nan(LinearSystem &system);

	public:
		static void rigid_only(LinearSystem& system);
		static Eigen::VectorXf solve(LinearSystem& system);
	};
}