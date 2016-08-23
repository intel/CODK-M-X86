#include <soc_ctrl.h>

#include "soc.h"
#include "curie_shared_mem.h"

void start_arc(unsigned int reset_vector)
{

	if (reset_vector != 0) {
		curie_shared_data->arc_start = reset_vector;
	}

	curie_shared_data->flags = 0;
	sys_thread_busy_wait(ARCSTART_DELAY);

	*SCSS_SS_CFG_REG |= ARC_RUN_REQ_A;

	sys_thread_busy_wait(ARCSTART_DELAY);
}

void halt_arc()
{
	*SCSS_SS_CFG_REG |= ARC_HALT_REQ_A;
}

void soft_reboot()
{
	halt_arc();
	*SCSS_RSTC = RSTC_WARM_RESET;
}
