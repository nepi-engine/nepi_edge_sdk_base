#ifndef _CONFIGURABLE_TRIGGER_H
#define _CONFIGURABLE_TRIGGER_H

#include "register.h"

namespace Numurus
{

class ConfigurableTrigger
{
public:
	ConfigurableTrigger(reg_addr_t mask_reg_addr, reg_addr_t delay_reg_addr);
	~ConfigurableTrigger();

	inline bool setMask(reg_val_t mask){return mask_register.setVal(mask);}
	inline bool getMask(reg_val_t *mask_out){return mask_register.getVal(mask_out);}
	inline bool disableTrigger(){return mask_register.setVal(0x0);}

	inline bool setDelay(reg_val_t delay){return mask_register.setVal(delay);}
	inline bool getDelay(reg_val_t *delay_out){return mask_register.getVal(delay_out);}

private:
	Register mask_register;
	Register delay_register;
};

} // namespace Numurus

#endif //_CONFIGURABLE_TRIGGER_H