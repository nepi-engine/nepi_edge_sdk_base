#ifndef _TEST_PAT_GENERATOR_H
#define _TEST_PAT_GENERATOR_H

#include <atomic>
#include <thread>

#include "triggerable_node.h"

namespace numurus
{

class TestPatGenerator : public TriggerableNode
{
public:
	TestPatGenerator(const std::string name);
	~TestPatGenerator();

	void run() override;

private:
	void acquireData();

	Register tpat_ctrl;
	Register tpat_stat;
	Register tpat_data;
	Register tpat_tstamp;

	std::atomic_bool keep_running;
	std::thread driver_thread;	
};

} // namespace numurus

#endif //_TEST_PAT_GENERATOR_H