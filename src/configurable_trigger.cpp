#include <ros/console.h>

#include "configurable_trigger.h"

namespace numurus
{

ConfigurableTrigger::ConfigurableTrigger(reg_addr_t mask_reg_addr, reg_addr_t delay_reg_addr) :
	mask_register{mask_reg_addr},
	delay_register{delay_reg_addr}
{
	// Validate the setup and report error if it's wrong
	if (!mask_register.isReady() || !delay_register.isReady())
	{
		ROS_ERROR("Trigger class with mask addr 0x%x improperly setup", mask_reg_addr);
	}
}

ConfigurableTrigger::~ConfigurableTrigger()
{

}

}