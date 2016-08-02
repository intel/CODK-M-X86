#include <zephyr.h>

#include "curie_shared_mem.h"

#if defined(CONFIG_STDOUT_CONSOLE)
#include <stdio.h>
#define PRINT           printf
#else
#include <misc/printk.h>
#define PRINT           printk
#endif

#define QUARK_BUFF 	curie_shared_data->quark_to_ARC.data
#define QUARK_BUFF_HEAD	curie_shared_data->quark_to_ARC.head
#define QUARK_BUFF_TAIL	curie_shared_data->quark_to_ARC.tail
#define QUARK_BUFF_FLAG	curie_shared_data->quark_to_ARC.flag
#define ARC_BUFF	curie_shared_data->ARC_to_quark.data
#define ARC_BUFF_HEAD	curie_shared_data->ARC_to_quark.head
#define ARC_BUFF_TAIL	curie_shared_data->ARC_to_quark.tail
#define ARC_BUFF_FLAG	curie_shared_data->ARC_to_quark.flag

#define QUARK_CORE

void init_sharedMemory_com()
{
  // initialize pointers to shared memory structure
  curie_shared_data->smc_shared_data_ptr= &curie_shared_data->smc_shared_data_obj;
  curie_shared_data->smc_shared_data_obj.quark_buffer = &curie_shared_data->quark_to_ARC;
  curie_shared_data->smc_shared_data_obj.arc_buffer = &curie_shared_data->ARC_to_quark;

  // initiliaze  some values
  QUARK_BUFF_HEAD = 0;
  QUARK_BUFF_TAIL = 0;
  QUARK_BUFF_FLAG = 2;
  ARC_BUFF_HEAD = 0;
  ARC_BUFF_TAIL = 0;
  ARC_BUFF_FLAG = 2;
}

int smc_availableForRead()
{
	return (int)(SHARED_BUFFER_SIZE + ARC_BUFF_HEAD-ARC_BUFF_TAIL) % SHARED_BUFFER_SIZE;
}

int smc_write(uint8_t data)
{
  //check if buffer is available
  if(QUARK_BUFF_FLAG == 1)
    return 1;

  //lock the buffer
  QUARK_BUFF_FLAG = 0;

  int new_head = (int)(QUARK_BUFF_HEAD+1)%SHARED_BUFFER_SIZE;
  if(new_head != QUARK_BUFF_TAIL)
  {
    QUARK_BUFF[QUARK_BUFF_HEAD] = data;
    QUARK_BUFF_HEAD = new_head;
  }
  else
  {
    return 2; //buffer is full
  }
  //unlock the buffer
  QUARK_BUFF_FLAG = 2;

  return 0;
}

uint8_t smc_read()
{
  //check if buffer is available
  if(ARC_BUFF_FLAG == 1)
    return 0;

  //lock the buffer
  ARC_BUFF_FLAG = 0;
  
  if(smc_availableForRead())
  {
    uint8_t data = ARC_BUFF[ARC_BUFF_TAIL];
    ARC_BUFF_TAIL = (ARC_BUFF_TAIL + 1)%SHARED_BUFFER_SIZE;
    ARC_BUFF_FLAG = 2; //unlock the buffer
    return data;
  }
  else
  {
    ARC_BUFF_FLAG = 2; //unlock the buffer
    return 0;
  }
}



