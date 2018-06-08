#ifndef _TEST_PAT_GENERATOR_H
#define _TEST_PAT_GENERATOR_H

#include <atomic>
#include <thread>

#include "triggerable_node.h"

namespace Numurus
{

/**
 * @brief      ROS node to access and control the FPGA test pattern generator (aka devmod1)
 * 			   This class provides a basic TriggerableNode that interfaces with the VDMA and interrupt-
 * 			   enabled Test Pattern Generator FPGA module. It relies on the Numurus-custom driver, and 
 * 			   provides an ROS interface to configure and control test pattern data.
 */
class TestPatGenerator : public TriggerableNode
{
public:
	/**
	 * @brief      
	 * @see TriggerableNode::TriggerableNode()
	 *
	 * @param[in]  name  The name of this node
	 */
	TestPatGenerator();
	~TestPatGenerator();

	/**
	 * @brief      Run the test pattern generator interface node
	 * 
	 * 			   This method overrides the TriggerableNode::run() method, launching a separate thread to wait for interrupts
	 * 			   from the test pattern driver (via char device read() method) and publish resultant messages.
	 */
	void run() override;

private:
	/**
	 * @brief      The run method for the driver interface thread.
	 * 			   This method attempts to read the devmod1_rx character device, and on successful read generates a (TBD) message
	 * 			   with test pattern data contents.
	 */	
	void acquireData();

	/**
	 * Interface for the Test Pattern Control register, which allows reset of test pattern data
	 */
	Register tpat_ctrl;

	/**
	 * Interface for the Test Pattern Status register, which provides the data-ready flag and data output count
	 */
	Register tpat_stat;

	/**
	 * Interface for the Test Pattern Data register
	 */
	Register tpat_data;

	/**
	 * Interface for the Test Pattern Time Stamp
	 */
	Register tpat_tstamp;

	/**
	 * Exit flag for the acquireData() method's main loop
	 */
	std::atomic_bool keep_running;

	/**
	 * Thread to interface with underlying VDMA driver and interrupt system
	 */
	std::thread driver_thread;	
};

} // namespace Numurus

#endif //_TEST_PAT_GENERATOR_H