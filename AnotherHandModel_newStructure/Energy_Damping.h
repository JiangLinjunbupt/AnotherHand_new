#pragma once
#include"Energy.h"
namespace energy {

	class Energy_Damping : public Energy {

	public:
		struct Settings {
			float translation_damping = 1;
			float rotation_damping = 20;
			float abduction_damping = 100;
			float top_phalange_damping = 50;
		} _settings;
		Settings*const settings = &_settings;


		void track(LinearSystem& system);
		void track_shape(LinearSystem& system);
	};

}