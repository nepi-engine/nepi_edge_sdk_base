#include <string.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <time.h>

#include <ros/console.h>

#include "register.h"

namespace numurus
{

static void usecsToTimespec(uint32_t usecs, timespec *timespecOut)
{
	static const uint32_t USECS_PER_SEC = 1e6;
	static const uint32_t NSECS_PER_USEC = 1000; 
	if (-1 == clock_gettime(CLOCK_MONOTONIC, timespecOut))
	{
		ROS_ERROR_THROTTLE(1, "Couldn't get the current time (%s)", strerror(errno));
		timespecOut->tv_sec = 0;
		timespecOut->tv_nsec = 0;
	}
	else
	{
		timespecOut->tv_sec += usecs / USECS_PER_SEC;
		timespecOut->tv_nsec += (usecs % USECS_PER_SEC) * NSECS_PER_USEC;
	}
}

Register::Register(reg_addr_t address, bool single_writer) :
	addr{address},
	sem{SEM_FAILED},
	writable{false},
	readable{true},
	reg_mem{static_cast<unsigned char*>(MAP_FAILED)},
	page_base{0},
	page_offset{0}
{
	ROS_DEBUG("Register Constructor (addr:0x%x, single_writer = %d)", addr, single_writer);
	// Construct the semaphore name from the address
	char sem_name[64];
	snprintf(sem_name, 64, "/sem_reg_%X", address);
	
	// Set the initial flags based on whether this is an exclusive-write register
	int sem_oflag = O_CREAT;
	if (true == single_writer)
	{
		sem_oflag |= O_EXCL;
	}
	sem = sem_open(sem_name, sem_oflag, 0644, 1);
	if (SEM_FAILED == sem)
	{
		if (EEXIST == errno) // Caller is not the owner/writer of the register 
		{
			constexpr int NO_FLAGS = 0;
			sem = sem_open(sem_name, NO_FLAGS);
			if (SEM_FAILED == sem)
			{
				ROS_ERROR("Unable to open existing semaphore %s (%s)", sem_name, strerror(errno));
				readable = false;
				return;
			}
			// The state here is readable/not writable
		}
		else // Unexpected sem_open error
		{
			ROS_ERROR("Unable to open semaphore %s (%s)", sem_name, strerror(errno));
			readable = false;
			return;
		}
	}
	else
	{
		writable = true;
	}
	
	// Now mmap the register
	// Truncate addr to a multiple of the page size, or mmap will fail.
    const size_t pagesize = sysconf(_SC_PAGE_SIZE);
    page_base = (addr / pagesize) * pagesize;
    page_offset = addr - page_base;

    int fd = open("/dev/mem", O_SYNC | O_RDWR);
    const int prot = (true == writable)? 
    	PROT_READ | PROT_WRITE : PROT_READ;
    // Must set MAP_SHARED to get writes back to the hardware
    reg_mem = static_cast<unsigned char*>(mmap(NULL, page_offset + sizeof(reg_val_t), prot, MAP_SHARED, fd, page_base));
    if (static_cast<unsigned char*>(MAP_FAILED) == reg_mem) {
        ROS_ERROR("Unable to map register memory for 0x%x (%s)", addr, strerror(errno));
        readable = false;
        writable = false;
    }
    close(fd);

    ROS_DEBUG("%s mem-mapped register interface at 0x%x (read=%d, write=%d)", (true == writable)? "Created" : "Linked to", addr, readable, writable);

    // Debugging
    //waitForLock(WAIT_FOREVER);
    //releaseLock();
}

Register::~Register()
{
	ROS_DEBUG("Register Destructor (addr:0x%x)", addr);
	// Unmap memory
	if (static_cast<unsigned char*>(MAP_FAILED) != reg_mem)
	{
		if (0 != munmap(reg_mem, page_offset + sizeof(reg_val_t)))
		{
			ROS_ERROR("Failed to unmap memory for register at 0x%x (%s)", addr, strerror(errno));
		}
	}
	if (SEM_FAILED != sem)
	{
		if (0 != sem_close(sem))
		{
			ROS_ERROR("Failed to close the semaphore for register at 0x%x (%s)", addr, strerror(errno));
		}
	}
}

bool Register::setVal(reg_val_t val, uint32_t timeout_usecs)
{
	ROS_DEBUG("Setting value %d to 0x%x", val, addr);
	if (false == writable)
	{
		ROS_ERROR_THROTTLE(5, "Register at 0x%x is not writable from this caller", addr);	
		return false;
	} 

	// Get the lock
	if (false == waitForLock(timeout_usecs))
	{
		return false; // Error logged upstream
	}

	// Update via shmem pointer
	memcpy(reg_mem + page_offset, &val, sizeof(reg_val_t));

	// Unlock
	releaseLock();
	return true;
}

bool Register::getVal(reg_val_t *val_out, uint32_t timeout_usecs)
{
	if (false == readable)
	{
		ROS_ERROR_THROTTLE(5, "Register at 0x%x is not readable", addr);
		return false;
	}

	// Get the lock
	if (false == waitForLock(timeout_usecs))
	{
		return false; // Error logged upstream
	}

	// Set value from shmem
	memcpy(val_out, reg_mem + page_offset, sizeof(reg_val_t));

	// Unlock
	releaseLock();
	return true;
}

bool Register::waitForLock(uint32_t timeout_usecs)
{
	int wait_return;
	if (WAIT_FOREVER == timeout_usecs)
	{
		wait_return = sem_wait(sem);
	}
	else if (WAIT_NONBLOCKING == timeout_usecs)
	{
		wait_return = sem_trywait(sem);
		if (EAGAIN == errno)
		{
			return false; // An expected possibility, so don't log an error.
		}
	}
	else // wait with timeout
	{
		timespec timeout;
		usecsToTimespec(timeout_usecs, &timeout);
		wait_return = sem_timedwait(sem, &timeout);
	}
	if (0 != wait_return)
	{
		ROS_ERROR_THROTTLE(1, "Semaphore (%p) wait failed while attempting to write register 0x%x (%s)", sem, addr, strerror(errno));
		return false;
	}
	return true;	
}

} // namespace numurus
