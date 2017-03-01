/*
 * Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __FACTORY_DATA_H__
#define __FACTORY_DATA_H__


#include <stdint.h>

#define CUSTOMER_DATA_ADDRESS 0xFFFFE200

/**
 * Contains Customer data.
 * size: 512 bytes
 */
typedef struct {
    /** product serial number
     * Up to 16 ASCII characters. If its length is < 16, it is NULL terminated.*/
    uint8_t product_sn[16];

    /** product hardware version
     * hexadecimal value, left-padded */
    uint8_t product_hw_ver[4];

    uint8_t reserved[12];

    uint8_t board_name[32];

    uint8_t vendor_name[32];

    uint32_t product_sn_len;

    uint32_t board_name_len;

    uint32_t vendor_name_len;

    uint16_t product_vid;

    uint16_t product_pid;

    uint8_t reserved_1[384];

    uint32_t patternKeyStart;

    uint32_t blockVersionHi;

    uint32_t blockVersionLow;

    uint32_t patternKeyEnd;

} customer_data_t;


#endif /* __FACTORY_DATA_H__ */

