/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef MODULE_NAME
#define MODULE_NAME "bbblue_pwm_out"
#endif

#include <fcntl.h>
#include <errno.h>
#include <px4_platform_common/log.h>

#include <board_config.h>

#include "board_pwm_out.h"
#include <sys/mman.h>

using namespace pwm_out;

#define PWM_CHAN_COUNT 12

static const uint8_t chan_pru_map[]= {10,8,11,9,7,6,5,4,3,2,1,0};                //chan_pru_map[CHANNEL_NUM] = PRU_REG_R30/31_NUM;

BBBlueRcPWMOut::BBBlueRcPWMOut(int max_num_outputs) : _num_outputs(max_num_outputs)
{
	/*if (_num_outputs > MAX_NUM_PWM) {
		PX4_WARN("number of outputs too large. Setting to %i", MAX_NUM_PWM);
		_num_outputs = MAX_NUM_PWM;
	}*/
}

BBBlueRcPWMOut::~BBBlueRcPWMOut()
{
	//rc_cleaning();
}

int BBBlueRcPWMOut::init()
{
    uint32_t mem_fd;
    mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC);
    sharedMem_cmd = (struct pwm_cmd *) mmap(0, 0x1000, PROT_READ|PROT_WRITE,
                                            MAP_SHARED, mem_fd, RCOUT_PRUSS_SHAREDRAM_BASE);
    close(mem_fd);

    // all outputs default to 50Hz, the top level vehicle code
    // overrides this when necessary
    set_freq(0xFFFFFFFF, 50);
    for (uint8_t i=0;i<PWM_CHAN_COUNT;i++) {
		enable_ch(i);
	}

	return 0;
}

void BBBlueRcPWMOut::set_freq(uint32_t chmask, uint16_t freq_hz)            //LSB corresponds to CHAN_1
{
    uint8_t i;
    unsigned long tick=TICK_PER_S/(unsigned long)freq_hz;

    for (i=0;i<PWM_CHAN_COUNT;i++) {
        if (chmask & (1U<<i)) {
            sharedMem_cmd->periodhi[chan_pru_map[i]][0]=tick;
        }
    }
}


void BBBlueRcPWMOut::enable_ch(uint8_t ch)
{
    sharedMem_cmd->enmask |= 1U<<chan_pru_map[ch];
}

void BBBlueRcPWMOut::disable_ch(uint8_t ch)
{
    sharedMem_cmd->enmask &= !(1U<<chan_pru_map[ch]);
}

int BBBlueRcPWMOut::send_output_pwm(const uint16_t *pwm, int num_outputs)
{
	if (num_outputs > PWM_CHAN_COUNT) {
		num_outputs = PWM_CHAN_COUNT;
	}

	int ret = 0;

	// pwm[ch] is duty_cycle in us
	for (int ch = 0; ch < num_outputs; ++ch) {
//		printf("%u ", pwm[ch]);
		sharedMem_cmd->periodhi[chan_pru_map[ch]][1] = TICK_PER_US*pwm[ch];
	}
//	printf("\n");

	return ret;
}

