/* SPDX-License-Identifier: BSD-2-Clause */

/*
 * Copyright (C) 2020 embedded brains Gmb_h (http://www.embedded-brains.de)
 * Copyright (C) 2023 Karel Gardas
 * Copyright (C) 2023 Cedric Berger
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

#include <bios_core.h>

#define MAX_QLEN    64

#define	SLNET_LOCK(sc) mtx_lock(&(sc)->mtx)
#define	SLNET_UNLOCK(sc) mtx_unlock(&(sc)->mtx)

struct slnet_softc {
    uint8_t		 mac_addr[6];
    struct ifnet	*ifp;
    device_t		 miibus;
    struct mii_data	*mii_softc;
    struct mtx		 mtx;
    struct callout	 tick_callout;
};

static void
slnet_tick(void *arg)
{
    struct slnet_softc *sc = arg;
    struct ifnet *ifp = sc->ifp;

    callout_reset(&sc->tick_callout, hz, slnet_tick, sc);
}

static void
slnet_init(void *arg)
{
    struct slnet_softc *sc = arg;
    struct ifnet *ifp = sc->ifp;

    mii_mediachg(sc->mii_softc);
    callout_reset(&sc->tick_callout, hz, slnet_tick, sc);
}

static int
slnet_transmit(struct ifnet *ifp, struct mbuf *m)
{
    return (ENETDOWN);
}

static void
slnet_qflush(struct ifnet *ifp)
{
}

static int
slnet_ioctl(struct ifnet *ifp, ioctl_command_t cmd, caddr_t data)
{
    return (EINVAL);
}

static void
slnet_media_status(struct ifnet *ifp, struct ifmediareq *ifmr_p)
{
	struct slnet_softc *sc = ifp->if_softc;
	struct mii_data *mii;

	SLNET_LOCK(sc);
	mii = sc->mii_softc;
	if (mii != NULL) {
		mii_pollstat(mii);
		ifmr_p->ifm_active = mii->mii_media_active;
		ifmr_p->ifm_status = mii->mii_media_status;
	}
	SLNET_UNLOCK(sc);
}

static int
slnet_media_change(struct ifnet *ifp)
{
	struct slnet_softc *sc = ifp->if_softc;
	int error;

	SLNET_LOCK(sc);
	if (sc->mii_softc != NULL)
		error = mii_mediachg(sc->mii_softc);
	else
		error = ENXIO;
	SLNET_UNLOCK(sc);
	return (error);
}

static int
slnet_probe(device_t dev)
{
    device_set_desc(dev, "SL-3011 Virtual ETH");
    return (BUS_PROBE_DEFAULT);
}

static int
slnet_attach(device_t dev)
{
	struct slnet_softc *sc;
	struct ifnet *ifp;
	int error;

	sc = device_get_softc(dev);
	sc->ifp = ifp = if_alloc(IFT_ETHER);
	ifp->if_softc = sc;

	mtx_init(&sc->mtx, device_get_nameunit(dev), MTX_NETWORK_LOCK, MTX_DEF);
	callout_init_mtx(&sc->tick_callout, &sc->mtx, 0);

	rtems_bsd_get_mac_address(device_get_name(dev), device_get_unit(dev),
	    &sc->mac_addr[0]);

	if_initname(ifp, "ve", device_get_unit(dev));
	ifp->if_flags = IFF_SIMPLEX | IFF_MULTICAST | IFF_BROADCAST;
	ifp->if_capenable = ifp->if_capabilities;
	ifp->if_transmit = slnet_transmit;
	ifp->if_qflush = slnet_qflush;
	ifp->if_ioctl = slnet_ioctl;
	ifp->if_init = slnet_init;

	IFQ_SET_MAXLEN(&ifp->if_snd, MAX_QLEN);
	ifp->if_snd.ifq_drv_maxlen = MAX_QLEN;
	IFQ_SET_READY(&ifp->if_snd);

	error = mii_attach(dev, &sc->miibus, ifp, slnet_media_change,
	    slnet_media_status, BMSR_DEFCAPMASK, MII_PHY_ANY,
	    MII_OFFSET_ANY, 0);
	if (error == 0)
		sc->mii_softc = device_get_softc(sc->miibus);

	ether_ifattach(ifp, &sc->mac_addr[0]);

	return (0);
}

static int
slnet_miibus_read(device_t dev, int phy, int reg)
{
	if (phy < 0 || phy >= BIOS->nveth)
		return (-1);
	if (reg < 0 || reg >= BIOS_VIRTUAL_MII_NREGS)
		return (0);
	return (BIOS->veths[phy].mii->regs[reg]);
}

static int
slnet_miibus_write(device_t dev, int phy, int reg, int val)
{
	return (-1);
}

static device_method_t slnet_methods[] = {
	DEVMETHOD(device_probe, slnet_probe),
	DEVMETHOD(device_attach, slnet_attach),
	DEVMETHOD(miibus_readreg, slnet_miibus_read),
	DEVMETHOD(miibus_writereg, slnet_miibus_write),
	DEVMETHOD_END
};

driver_t slnet_driver = {
	"slnet",
	slnet_methods,
	sizeof(struct slnet_softc)
};

static devclass_t slnet_devclass;

DRIVER_MODULE(slnet, nexus, slnet_driver, slnet_devclass, 0, 0);
DRIVER_MODULE(miibus, slnet, miibus_driver, miibus_devclass, 0, 0);

MODULE_DEPEND(slnet, ether, 1, 1, 1);
MODULE_DEPEND(slnet, miibus, 1, 1, 1);

#endif /* LIBBSP_ARM_STM32H7_BSP_H */
