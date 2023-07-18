/**
 * @file
 *
 * @brief Ping (shell command) test.
 */

/*
 * Copyright (c) 2013-2014 embedded brains GmbH.  All rights reserved.
 *
 *  embedded brains GmbH
 *  Dornierstr. 4
 *  82178 Puchheim
 *  Germany
 *  <rtems@embedded-brains.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <math.h>

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sysexits.h>

#include <machine/rtems-bsd-commands.h>

#include <rtems/bsd/test/network-config.h>

#include <bsp.h>
#include <stm32h7/hal.h>
#include <rtems/score/armv7m.h>

#define TEST_NAME "LIBBSD PING 1"
#define TEST_WAIT_FOR_LINK NET_CFG_INTERFACE_0

static void
test_ping(void)
{
	int exit_code;
	char *ping[] = {
		"ping",
		"-c",
		"10",
		NET_CFG_PEER_IP,
		NULL
	};

	for (;;) {
          exit_code = rtems_bsd_command_ping(RTEMS_BSD_ARGC(ping), ping);
	  usleep(1000000);
	}
//	assert(exit_code == EXIT_SUCCESS);
}

static void
test_main(void)
{
    printf("Cedric's test @ test_main:\n");
    for (int i = 0; i < 64; i++) {
        volatile uint64_t i1 = ((uint64_t)1) << i;
        volatile double d1 = (double)i1;
        volatile double d2 = pow(2.0, i);
        int OK = d1 == d2;
        printf("2^%d: %s (%.8f vs %.8f)\n", i, OK ? "OK" : "BAD", d1, d2);
    }
  printf("ASPEN: %s\n", (_ARMV7M_SCB->fpccr & FPU_FPCCR_ASPEN_Msk) ? "set" : "not set");
  printf("LSPEN: %s\n", (_ARMV7M_SCB->fpccr & FPU_FPCCR_LSPEN_Msk) ? "set" : "not set");  
      
	printf("start...\n");
	test_ping();
	printf("finish...\n");

	exit(0);
}

#include <rtems/bsd/test/default-network-init.h>
