#ifndef _REGISTER_H
#define _REGISTER_H

#include <stdint.h>
#include <semaphore.h>

namespace Numurus
{

typedef int32_t reg_val_t;
typedef int32_t reg_addr_t;

class Register
{
public:
	Register(const std::string &name, reg_addr_t address);
	~Register();

	inline const std::string& getName() const {return name;}
	
	inline bool isReady() const {return ready;}
	
	bool setVal (reg_val_t val, uint32_t timeout_usecs=WAIT_FOREVER);
	bool getVal (reg_val_t *val_out, uint32_t timeout_usecs=WAIT_FOREVER);

	static constexpr uint32_t WAIT_FOREVER = 0xffffffff;
	static constexpr uint32_t WAIT_NONBLOCKING = 0xfffffffe;
private:
	bool waitForLock(uint32_t timeout_usecs);
	inline void releaseLock(void){sem_post(sem);}

	const std::string name;
	reg_addr_t addr;
	sem_t *sem;
	bool ready;	
	unsigned char* reg_mem;
	off_t page_base;
    off_t page_offset;
};

} // namespace Numurus

#endif //_REGISTER_H