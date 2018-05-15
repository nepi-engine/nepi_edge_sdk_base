#ifndef _REGISTER_H
#define _REGISTER_H

#include <stdint.h>
#include <semaphore.h>

namespace numurus
{

typedef uint32_t reg_val_t;
typedef uint32_t reg_addr_t;

class Register
{
public:
	Register(reg_addr_t address);
	~Register();
	
	inline bool isReady() const {return ready;}
	
	bool setVal (reg_val_t val, uint32_t timeout_usecs=WAIT_FOREVER);
	bool getVal (reg_val_t *val_out, uint32_t timeout_usecs=WAIT_FOREVER);

	static constexpr uint32_t WAIT_FOREVER = 0xffffffff;
	static constexpr uint32_t WAIT_NONBLOCKING = 0xfffffffe;
private:
	bool waitForLock(uint32_t timeout_usecs);
	inline void releaseLock(void){sem_post(sem);}

	reg_addr_t addr;
	sem_t *sem;
	bool ready;	
	unsigned char* reg_mem;
	off_t page_base;
    off_t page_offset;
};

} // namespace numurus

#endif //_REGISTER_H