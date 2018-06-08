#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "num_fpga.h"
#include "test_pat_generator.h"

#include "num_devmod1/num_devmod1.h"

#define NODE_NAME	"test_pat_generator"

namespace Numurus {

TestPatGenerator::TestPatGenerator(const std::string my_name) :
	TriggerableNode{my_name, ADR_TPAT_SW_TRIG_MASK, ADR_TPAT_SW_TRIG_DLY},
	tpat_ctrl{"test_pat_ctrl", ADR_TPAT_CTRL},
	tpat_stat{"test_pat_stat", ADR_TPAT_STAT},
	tpat_data{"test_pat_data", ADR_TPAT_DATA},
	tpat_tstamp{"test_pat_tstamp", ADR_TPAT_TSTAMP},
	keep_running{true},
	driver_thread{&TestPatGenerator::acquireData, this}
{
	configurable_regs.push_back(&tpat_data);
}

TestPatGenerator::~TestPatGenerator()
{
	keep_running = false;
	driver_thread.join();
}

void TestPatGenerator::run()
{
	init();

	if (false == ready())
	{
		ROS_ERROR("%s: Not properly initialized... exiting", name.c_str());
		return;		
	}

	ros::spin();
}

void TestPatGenerator::acquireData()
{
	const char* device_pathname = "/dev/num_devmod1_rx";
	struct dma_proxy_channel_interface *rx_proxy_if_p;
	int rx_proxy_fd;

	ROS_DEBUG("Launching the test pattern generator acquire data thread");

	rx_proxy_fd = open(device_pathname, O_RDWR);
	if (rx_proxy_fd < 1)
	{
		ROS_ERROR("%s: Unable to open %s... was the driver started properly?", name.c_str(), device_pathname);
		return;
	}

	rx_proxy_if_p = (struct dma_proxy_channel_interface *) mmap(NULL, sizeof(struct dma_proxy_channel_interface), PROT_READ | PROT_WRITE, MAP_SHARED, rx_proxy_fd, 0);
	if (MAP_FAILED == rx_proxy_if_p)
	{
		ROS_ERROR("%s: Failed to mmap the DMA", name.c_str());
		return;
	}

	while(keep_running)
	{
		ssize_t count = read(rx_proxy_fd, rx_proxy_if_p, BUF_SIZE);
		if (count < 0)
		{
			ROS_ERROR_THROTTLE(1, "%s: Failed to read mmapped data", name.c_str());
			continue;
		}

		// TODO: Publish the relevant message
		ROS_INFO("Got %u bytes of test pattern data", count);
	}

	// Release resources
}

} // namespace Numurus

int main(int argc, char **argv)
{
	ROS_INFO("Starting the %s Node", NODE_NAME);
	ros::init(argc, argv, NODE_NAME);
	
	// The class instance does the work
	numurus::TestPatGenerator tpat_generator(NODE_NAME);
	tpat_generator.run();

	return 0;
} 

