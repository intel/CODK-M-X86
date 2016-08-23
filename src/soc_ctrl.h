#include <stdint.h>

#define SCSS_SS_CFG_REG (uint32_t*)0xb0800600

#define ARCSTART_DELAY		500000	//500msec

#define SCSS_RSTC   (uint32_t*)0xb0800570

#define RSTC_WARM_RESET	(1 << 1)
#define RSTC_COLD_RESET (1 << 3)

#ifdef __cplusplus
extern "C" {
#endif

void start_arc(unsigned int reset_vector);

void halt_arc();

void soft_reboot();

#ifdef __cplusplus
}
#endif
