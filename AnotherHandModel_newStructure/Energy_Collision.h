#pragma once
#include "Energy.h"
#include"HandModel.h"

namespace energy {

	class Energy_Collision : public Energy {
	public:
		struct Settings {
			bool collision_enable = true;
			float collision_weight = 10e2;
			float collision_fraction = 0.01f;
			bool collision_new = false;
		} _settings;
		Settings*const settings = &_settings;


	private:
		HandModel * model = NULL;

	public:
		void init(HandModel * model);
		void track(LinearSystem& system);
	};

}
