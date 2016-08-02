#ifndef _SHAREDMEMORY_SMC_
#define _SHAREDMEMORY_SMC_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void init_sharedMemory_com();

int smc_availableForRead();

int smc_write(uint8_t data);

uint8_t smc_read();

#ifdef __cplusplus
}
#endif

#endif /* _SHAREDMEMORY_SMC_ */
