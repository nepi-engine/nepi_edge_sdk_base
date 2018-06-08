#ifndef _REGISTER_H
#define _REGISTER_H

#include <stdint.h>
#include <semaphore.h>

namespace Numurus
{

typedef int32_t reg_val_t;
typedef int32_t reg_addr_t;

/**
 * @brief Represents a single mem-mapped register for Zynq targets. 
 * 
 * 		  Setters and getters interact directly with the hardware in a threadsafe fashion via named semaphores
 */
class Register
{
public:
	/**
	 * @brief      Constructor
	 *
	 * @param[in]  name     A string name for this register
	 * @param[in]  address  Register address, typically a named const from num_fpga.h
	 */
	Register(const std::string &name, reg_addr_t address);
	~Register();

	/**
	 * @brief      Gets the name.
	 *
	 * @return     The name.
	 */
	inline const std::string& getName() const {return name;}
	
	/**
	 * @brief      Determines if ready.
	 *
	 * @return     True if ready, False otherwise.
	 */
	inline bool isReady() const {return ready;}
	
	/**
	 * @brief      Sets the value for the register.
	 *
	 * @param[in]  val            The (32-bit) value to set
	 * @param[in]  timeout_usecs  The timeout in usecs, defaults to no timeout
	 *
	 * @return     true if the call succeeds, false otherwis
	 */
	bool setVal (reg_val_t val, uint32_t timeout_usecs=WAIT_FOREVER);
	
	/**
	 * @brief      Gets the value of the register.
	 *
	 * @param      val_out        Storage for the register value
	 * @param[in]  timeout_usecs  The timeout in usecs, defaults to no timeout
	 *
	 * @return     The value.
	 */
	bool getVal (reg_val_t *val_out, uint32_t timeout_usecs=WAIT_FOREVER);

	/**
	 *  Sentinel timeout val for calls that should block indefinitely
	 */
	static constexpr uint32_t WAIT_FOREVER = 0xffffffff;

	/**
	 * Sentinel timeout val for calls that should not block, 
	 */
	static constexpr uint32_t WAIT_NONBLOCKING = 0xfffffffe;

private:
	/**
	 * @brief      Wait for the semaphore to become available for reading/writing the physical register
	 *
	 * @param[in]  timeout_usecs  The timeout in usecs. @see WAIT_FOREVER and @WAIT_NONBLOCKING for sentinels
	 *
	 * @return     true if lock successfully obtained, false otherwise
	 */
	bool waitForLock(uint32_t timeout_usecs);

	/**
	 * @brief      Post the semaphore to allow other threads to interact with the register
	 */
	inline void releaseLock(void){sem_post(sem);}

	/**
	 * The constructor-supplied name of the register
	 */
	const std::string name;

	/**
	 * The system address for the register, typically a constant from num_fpga.h
	 */
	reg_addr_t addr;

	/**
	 * The binary semaphore (mutex) for register access thread-safety. Named for multi-process access.
	 */
	sem_t *sem;

	/**
	 * Indicates that the register has been set up properly and is ready for (attempted) access
	 */
	bool ready;	

	/**
	 * Mem-mapped buffer for register memory
	 */
	unsigned char* reg_mem;

	/**
	 * Helper for mmap alignment requirements
	 */
	off_t page_base;

	/**
	 * Helper for mmap alignment requirements
	 */
    off_t page_offset;
};

} // namespace Numurus

#endif //_REGISTER_H