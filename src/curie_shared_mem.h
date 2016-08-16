/*
 * Copyright (c) Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CURIE_SHARED_MEM_H_
#define CURIE_SHARED_MEM_H_

/* Start of the shared RAM */
#define SHARED_ADDR_START 0xA8000000

#define SERIAL_BUFFER_SIZE 256
#define SHARED_BUFFER_SIZE 64

/**
 * Use the following defines just to make the tips of your finger happier.
 */

#define Rx_BUFF curie_shared_data->cdc_acm_shared_rx_buffer.data
#define Rx_HEAD curie_shared_data->cdc_acm_shared_rx_buffer.head
#define Rx_TAIL curie_shared_data->cdc_acm_shared_rx_buffer.tail
#define Tx_BUFF curie_shared_data->cdc_acm_shared_tx_buffer.data
#define Tx_HEAD curie_shared_data->cdc_acm_shared_tx_buffer.head
#define Tx_TAIL curie_shared_data->cdc_acm_shared_tx_buffer.tail
#define SBS     SERIAL_BUFFER_SIZE

struct cdc_ring_buffer
{
    /** Ring buffer data */
    volatile uint8_t data[SERIAL_BUFFER_SIZE];
    /** Ring buffer head index, modified by producer */
    volatile int head;
    /** Ring buffer tail index, modified by consumer */
    volatile int tail;
};

struct cdc_acm_shared_data {
    /** Ring buffer to pass CDC-ACM data from QRK to ARC */
    struct cdc_ring_buffer *rx_buffer;
    /** Ring buffer to pass CDC-ACM data from ARC to QRK */
    struct cdc_ring_buffer *tx_buffer;
    /** Boolean flag set by QRK when CDC-ACM host connection is opened */
    volatile int host_open;
    /** Boolean flag set by ARC when CDC-ACM endpoint connection is opened */
    volatile int device_open;
};

struct shared_ring_buffer
{
    /** Ring buffer data */
    volatile uint8_t data[SHARED_BUFFER_SIZE];
    /** Ring buffer head index, modified by producer */
    volatile int head;
    /** Ring buffer tail index, modified by consumer */
    volatile int tail;

    /** Buffer status
     * 0 - locked by X86 core
     * 1 - locked by arc core
     * 2 - available to be taken by any core
    **/
    volatile int flag;
};

struct smc_shared_data
{
    struct shared_ring_buffer *quark_buffer;
    struct shared_ring_buffer *arc_buffer;
};

struct platform_shared_block_ {
    /** Arc reset vector */
    unsigned int arc_start;
    /** Port table address */
    void * ports;
    /** Service table address */
    void * services;
    /** Port id of the service manager */
    uint16_t service_mgr_port_id;
    /** ARC boot synchronization flag.
     * This value is set to 0 prior to start ARC, and is polled until set to 1
     * by ARC in order to allow QRK to wait for ARC to be started. Useful for
     * debugging ARC startup code.
     */
    //uint8_t arc_ready;
    uint8_t flags;

    /** used to send suspend resume arc core
     * bit usage
     * [0-7] 	PM_POWERSTATE
     * [8-9] 	ACK
     * [16-31]	Magic number
     */
    uint32_t pm_request;

    /** ARC wakelocks status info variables
     * Used in order to share if any wakelock
     * is taken, on ARC side.
     */
    uint8_t any_arc_wakelock_taken;

    /** QRK wakelocks status info variables
     * Used in order to share if any wakelock
     * is taken, on QRK side.
     */
    uint8_t any_qrk_wakelock_taken;

    /* Pointer to shared structure used by CDC-ACM.
     *
     * The QRK core is responsible for allocating memory and initialising the
     * pointers of this structure.
     * The ARC core counts on QRK to find valid pointers in place.
     */
    struct cdc_acm_shared_data	*cdc_acm_buffers;

    struct cdc_acm_shared_data cdc_acm_buffers_obj;

    struct cdc_ring_buffer cdc_acm_shared_rx_buffer;
    struct cdc_ring_buffer cdc_acm_shared_tx_buffer;

    struct smc_shared_data *smc_shared_data_ptr;

    struct smc_shared_data smc_shared_data_obj;

    struct shared_ring_buffer quark_to_ARC;
    struct shared_ring_buffer ARC_to_quark;
};


#define ARC_READY	(1 << 0)

#define curie_shared_data ((struct platform_shared_block_ *) SHARED_ADDR_START)

#endif
