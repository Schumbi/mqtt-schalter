#include "strip.hpp"

static CLedStrip* LedStrip = NULL;

CLedStrip* CLedStrip::getStrip_ptr()
{
	if(LedStrip == NULL)
	{
		LedStrip = new CLedStrip(num_leds, 0);
	}
	return LedStrip;
}

CLedStrip::CLedStrip(uint8_t count, uint8_t prog)
	: NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod>(count),
	  _prog(prog),
	  _init(false)
{}

void CLedStrip::init()
{
	if(!_init)
	{
		this->Begin();
		_init = true;
		_prog = 0;
		this->update();
		// make it global
		LedStrip = this;
	}
}

bool CLedStrip::update()
{
	bool ret = false;
	if(!_init)
		return _init;
	
	//STRIPDEBUG.println("[Debug] Strip update" + String(_prog));
	// choose program
	switch(_prog)
	{
		case 0:
			ret = caller(_conf, &CLedStrip::prog0);
			break;
		case 1:
			ret = caller(_conf, &CLedStrip::prog1);
			break;
		case 2:
			ret = caller(_conf, &CLedStrip::prog2);
			break;
		case 3:
			ret = caller(_conf, &CLedStrip::prog3);
			break;
		case 4:
			ret = caller(_conf, &CLedStrip::prog4);
			break;
		case 5:
			ret = caller(_conf, &CLedStrip::prog5);
			break;
		default:
			ret = caller(_conf, &CLedStrip::prog0);
	}
	// update
	this->Show();

	return ret;
}

bool CLedStrip::switch_program(uint8_t prog)
{
	if(!_init)
		return _init;

	// save old prog
	_conf.old_prog = _prog;
	// prog
	_prog = prog;
	// update program
	update();
	return true;
}

bool CLedStrip::caller(prog_param_t &param, prog_fp func)
{
	bool ret = false;

	if(!this->_init)
		return this->_init;

	// call the prog and return the state
	ret = ((*this).*(func))(param);

	return ret;
}

CLedStrip::prog_param_t &CLedStrip::getConf()
{
	return _conf;
}

