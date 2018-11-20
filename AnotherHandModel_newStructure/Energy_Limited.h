#pragma once
#include"Energy.h"
#include"HandModel.h"

namespace energy {

	class Energy_Limited : public Energy {
	public:
		HandModel * model = NULL;
		bool jointlimits_enable = false;
		bool shapelimits_enable = true;
		float jointlimits_weight = 50;

		void init(HandModel * model);
		void track(LinearSystem& sys, bool with_glove);
		void track_shape(LinearSystem& sys);
		void shape_boundLimited(LinearSystem& sys);
		void shape_relativeLimited(LinearSystem& sys);
	};

}