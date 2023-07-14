/* SPDX-License-Identifier: BSD-2-Clause */

/*
 * Copyright (C) 2023 Karel Gardas
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <machine/rtems-bsd-kernel-space.h>

#include <bsp.h>

#ifdef LIBBSP_ARM_STM32H7_BSP_H

#include <sys/param.h>
#include <sys/types.h>
#include <sys/bus.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/gsb_crc32.h>

#include <net/if.h>
#include <net/ethernet.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_var.h>

#include <machine/bus.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <rtems/bsd/local/miibus_if.h>

#include <stm32h7xx_hal.h>

#include <rtems/bsd/bsd.h>
#include <rtems/irq-extension.h>
#include <rtems/score/armv7m.h>

/*
#include <sys/param.h>
#include <sys/hash.h>
#include <sys/jail.h>
#include <sys/kernel.h>
#include <sys/libkern.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/proc.h>
#include <sys/refcount.h>
#include <sys/queue.h>
#include <sys/smp.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>
#include <sys/types.h>

#include <net/bpf.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_clone.h>
#include <net/if_media.h>
#include <net/if_var.h>
#include <net/if_types.h>
#include <net/netisr.h>
#include <net/vnet.h>
*/

struct slnet_softc {
    struct ifnet *ifp;
    struct ifmedia media; /* fake media as we're working on top of virtual ethernet provided by the firmware API */
};

static int
if_slnet_probe(device_t dev)
{
    printf("if_slnet_probe\n");
    device_set_desc(dev, "SLNET");
    return (BUS_PROBE_DEFAULT);
}

static int
if_slnet_attach(device_t dev)
{
    printf("if_slnet_attach\n");
}

static device_method_t slnet_methods[] = {
	DEVMETHOD(device_probe, if_slnet_probe),
	DEVMETHOD(device_attach, if_slnet_attach),
	DEVMETHOD_END
};

driver_t slnet_driver = {
	"slnet",
	slnet_methods,
	sizeof(struct slnet_softc)
};

static devclass_t slnet_devclass;

DRIVER_MODULE(slnet, nexus, slnet_driver, slnet_devclass, 0, 0);
MODULE_DEPEND(slnet, nexus, 1, 1, 1);
MODULE_DEPEND(slnet, ether, 1, 1, 1);

#endif /* LIBBSP_ARM_STM32H7_BSP_H */
