#ifndef MAKELIGHT_STRIP_H
#define MAKELIGHT_STRIP_H

#include <NeoPixelBus.h>

#ifdef DEBUG_STRIP
#define STRIPDEBUG Dummy
class Dummy
{
	public:
	int println(String inp)
	{
		return 0;
	}
};
#else
#define STRIPDEBUG Serial
#endif

const uint8_t num_leds = 3;

class CLedStrip : public NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> 
{

	public:
		typedef struct 
		{
			uint8_t old_prog;
			uint16_t min;
			uint16_t max;
			uint16_t ctr;
			int16_t period;
			bool state;
		}prog_param_t, prog_param_ptr;

		CLedStrip(uint8_t count = num_leds, uint8_t prog = 0);

		void init();
		bool update();
		prog_param_t &getConf();
		bool switch_program(uint8_t prog);

		static CLedStrip* getStrip_ptr();

	protected:

		typedef bool(CLedStrip::*prog_fp)(prog_param_t &param);

		uint8_t _prog;
		bool _init;

		prog_param_t _conf;
	
		// calls the progs
		bool caller(prog_param_t &param, prog_fp func);

		// implement the functionality
		bool prog0(prog_param_t &param);
		bool prog1(prog_param_t &param);
		bool prog2(prog_param_t &param);
		bool prog3(prog_param_t &param);
		bool prog4(prog_param_t &param);
		bool prog5(prog_param_t &param);
};

#endif //MAKELIGHT_STRIP_H

