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

#include <bios_core.h>

MALLOC_DEFINE(M_SLNET, "slnet", "Storage for mbuf bookkeeping");

#define SNLET_BUFSIZE	  (8*1024)	/* must be power of two */
#define SNLET_BUFMASK	  (SNLET_BUFSIZE - 1)

#define SLNET_MAX_QLEN    64
#define SLNET_BMSR_CAPS	(BMSR_10THDX | BMSR_10TFDX | BMSR_100TXHDX | BMSR_100TXFDX /* | BMSR_ANEG */)

#define	SLNET_LOCK(sc) mtx_lock(&(sc)->mtx)
#define	SLNET_UNLOCK(sc) mtx_unlock(&(sc)->mtx)

struct slnet_softc {
	uint8_t			  mac_addr[6];
	struct ifnet		 *ifp;
	struct ifmedia		  media;		/* Media config (fake). */
	struct mtx		  mtx;
	struct callout		  tick_callout;
	int			  iid;
	bios_virtual_eth 	 *veth;
	bios_pkt_queue	 	 *outq;
	bios_pkt_queue	 	 *inq;
	char *			 *obuf;
	uint32_t		  ohead;
	uint32_t		  otail;
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

	callout_reset(&sc->tick_callout, hz, slnet_tick, sc);
}

static void
slnet_qflush(struct ifnet *ifp)
{
}

static int
slnet_do_transmit(struct ifnet *ifp, struct mbuf *m)
{
	struct slnet_softc *sc = ifp->if_softc;
	char	*p = NULL;
	uint32_t ohead = sc->ohead;
	uint32_t otail = sc->otail;

	int mlen = m_length(m, NULL);
	if (mlen < sizeof(struct ether_header) + 20) {
		printf("slnet/tx%d: mbuf too small (%d)\n", sc->iid, mlen);
		return (ENOBUFS);
	}
	if (mlen > SNLET_BUFSIZE) {
		printf("slnet/tx%d: mbuf too big (%d)\n", sc->iid, mlen);
		return (ENOBUFS);
	}
	int oused = otail - ohead;
	BSD_ASSERT(oused <= SNLET_BUFSIZE);
	if (mlen > SNLET_BUFSIZE - oused) {
		printf("slnet/tx%d: no space in obuf (%d > %d)\n", sc->iid, mlen, SNLET_BUFSIZE - oused);
		return (ENOBUFS);
	}
	int ph = ohead & SNLET_BUFMASK;
	int pt = otail & SNLET_BUFMASK;
	if ((ph > pt) ||			/* one contiguous area in the middle */
	    (SNLET_BUFSIZE - pt >= mlen))	/* enough space at end of split buffer */
	{
		p = sc->obuf + pt;
		otail += mlen;
	} else if (ph >= mlen) {		/* enough space at beginning of split buffer */
		p = sc->obuf;
		otail += (SNLET_BUFSIZE - pt);	/* skip past of end of split buffer */
		otail += mlen;
	} else {				/* no contigous area big enough available */
		printf("slnet/tx%d: no contigous area in obuf (%d)\n", sc->iid, mlen);
		return (ENOBUFS);
	}
	    	    
	bios_virtual_eth *veth = sc->veth;
	bios_pkt_queue *outq = sc->outq;
	uint32_t qhead = *outq->head;
	uint32_t qtail = *outq->tail;
	uint32_t qused = qtail - qhead;
	uint32_t qmask = outq->size - 1;
	if (qused == outq->size) {
		printf("slnet/tx%d: outq full (%d/%d)\n", sc->iid, qused, outq->size);
		return (ENOBUFS);
	}
	
	m_copydata(m, 0, mlen, p);
	sc->otail = otail;		/* advance obuf/tail */
	
	outq->pkts[qtail & qmask].buf = p;
	outq->pkts[qtail & qmask].size = mlen;
	BIOS_DSB();
	BIOS_ISB();
	*outq->tail = qtail + 1;	/* advance outq/tail */

	printf("slnet/tx%d: slot %d, 0x%08x + %d\n", sc->iid, qtail, p, mlen);
	for (int i = 0; i < 40; i += 4)
		printf(" 0x%08x", *(uint32_t*)(i + (intptr_t)p));
	printf("\n");
	return (0);
}

static int
slnet_transmit(struct ifnet *ifp, struct mbuf *m)
{
	 int err;
     
	 err = slnet_do_transmit(ifp, m);
	 m_freem(m);
	 if (err)
		 if_inc_counter(ifp, IFCOUNTER_OQDROPS, 1);
}

static int
slnet_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct slnet_softc *sc = ifp->if_softc;
	struct ifreq *ifr = (struct ifreq *)data;
	int error;

	switch (cmd) {
	case SIOCSIFFLAGS:
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		error = 0;
		break;

	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
		error = ifmedia_ioctl(ifp, ifr, &sc->media, cmd);
		break;

	default:
		/* Let the common ethernet handler process this. */
		error = ether_ioctl(ifp, cmd, data);
		break;
	}

	return (error);
}

static int
slnet_media_change(struct ifnet *ifp __unused)
{
	/* do nothing */
	return (0);
}

static void
slnet_media_status(struct ifnet *ifp __unused, struct ifmediareq *imr)
{
	imr->ifm_status = IFM_AVALID | IFM_ACTIVE;
	imr->ifm_active = IFM_ETHER | IFM_100_TX | IFM_FDX;
}

static int
slnet_probe(device_t dev)
{
	int iid = device_get_unit(dev);

	if (iid < 0 || iid >= BIOS->nveth)
		return (ENXIO);
	device_set_desc(dev, "SL-3011 virtual ethernet");
	return (BUS_PROBE_DEFAULT);
}

static int
slnet_attach(device_t dev)
{
	struct slnet_softc *sc;
	struct ifnet *ifp;
	int error, caps;

	sc = device_get_softc(dev);
	sc->ifp = ifp = if_alloc(IFT_ETHER);
	sc->iid = device_get_unit(dev);
	ifp->if_softc = sc;
    
	sc->veth = BIOS->veths + sc->iid;
	sc->outq = sc->veth->outq;
	sc->inq = sc->veth->inq;
	sc->obuf = rtems_cache_coherent_allocate(SNLET_BUFSIZE, CPU_CACHE_LINE_BYTES, 0);
	BSD_ASSERT(sc->obuf != NULL);
	sc->ohead = sc->otail = 0;

	mtx_init(&sc->mtx, device_get_nameunit(dev), MTX_NETWORK_LOCK, MTX_DEF);
	callout_init_mtx(&sc->tick_callout, &sc->mtx, 0);

	rtems_bsd_get_mac_address(device_get_name(dev), sc->iid, &sc->mac_addr[0]);

	/* Initialise pseudo media types. */
	ifmedia_init(&sc->media, 0, slnet_media_change, slnet_media_status);
	ifmedia_add(&sc->media, IFM_ETHER | IFM_100_TX, 0, NULL);
	ifmedia_set(&sc->media, IFM_ETHER | IFM_100_TX);

	if_initname(ifp, "eth", sc->iid);
	ifp->if_dunit = sc->iid;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_capabilities = 0; /* IFCAP_VLAN_MTU ? */
	ifp->if_capenable = 0; /* IFCAP_VLAN_MTU ? */
	ifp->if_transmit = slnet_transmit;
	ifp->if_qflush = slnet_qflush;
	ifp->if_ioctl = slnet_ioctl;
	ifp->if_init  = slnet_init;
	if_setsendqlen(ifp, ifqmaxlen);
	if_setsendqready(ifp);
	ifp->if_baudrate = IF_Mbps(100);
    
	ether_ifattach(ifp, &sc->mac_addr[0]);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	if_link_state_change(ifp, LINK_STATE_UP);

	return (0);
}

static device_method_t slnet_methods[] = {
	DEVMETHOD(device_probe, slnet_probe),
	DEVMETHOD(device_attach, slnet_attach),
	DEVMETHOD_END
};

driver_t slnet_driver = {
	"slnet",
	slnet_methods,
	sizeof(struct slnet_softc)
};

static devclass_t slnet_devclass;

DRIVER_MODULE(slnet, nexus, slnet_driver, slnet_devclass, 0, 0);
MODULE_DEPEND(slnet, ether, 1, 1, 1);

#endif /* LIBBSP_ARM_STM32H7_BSP_H */
