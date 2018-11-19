#pragma once
#include"Energy.h"
#include"HandModel.h"

namespace energy {
	class Energy_Temporal : public Energy {
		HandModel * model = NULL;

	public:
		struct Settings {
			bool temporal_coherence1_enable = true;
			bool temporal_coherence2_enable = true;
			float temporal_coherence1_weight = 0.05f;
			float temporal_coherence2_weight = 0.05f;
		} _settings;
		Settings*const settings = &_settings;

	public:
		void init(HandModel * model);
		void track(LinearSystem& system);
		void update(int frame_id);
	private:
		void track(LinearSystem& system, int fid, bool first_order);
		void temporal_coherence_init();
	};

}