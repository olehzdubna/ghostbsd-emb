/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2013 Ian Lepore <ian@freebsd.org>
 * Copyright (c) 2011 Ben Gray <ben.r.gray@gmail.com>.
 * All rights reserved.
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
 *
 */
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "ti_mcasp_api.h"

#include <sys/types.h>
#include <sys/conf.h> /* cdevsw struct */
#include <sys/uio.h>  /* uio struct */

/*
 * Table of supported FDT compat strings.
 *
 */
static struct ofw_compat_data compat_data[] = {
	{"ti,am33xx-mcasp-audio",1},
	{NULL, 0},
};

/* Function prototypes */
static d_open_t      mcasp_chdev_open;
static d_close_t     mcasp_chdev_close;
static d_read_t      mcasp_chdev_read;
static d_write_t     mcasp_chdev_write;

/* Character device entry points */
static struct cdevsw mcasp_chdev_sw = {
    .d_version = D_VERSION,
    .d_open = mcasp_chdev_open,
    .d_close = mcasp_chdev_close,
    .d_read = mcasp_chdev_read,
    .d_write = mcasp_chdev_write,
    .d_name = "mcasp",
};

/* vars */
static struct cdev *mcasp_chdev;

static int
mcasp_chdev_open(struct cdev *dev __unused, int oflags __unused, int devtype __unused,
    struct thread *td __unused)
{
    int error = 0;

    struct ti_mcasp_softc *sc = device_get_softc((device_t)dev->si_drv1);
    printf("+++ mcasp_chdev_open sc_bsh:uint32_t val[0]: %8.8x \n", ti_mcasp_read_4(sc, 0));

    printf("+++ mcasp_chdev_open mcasp_tx_reset \n");
    mcasp_tx_reset(sc);

    printf("+++ mcasp_chdev_open mcasp_rx_reset \n");
    mcasp_rx_reset(sc);

    uprintf("Opened device \"mcasp\" successfully.\n");
    return (error);
}

static int
mcasp_chdev_close(struct cdev *dev __unused, int fflag __unused, int devtype __unused,
    struct thread *td __unused)
{
    int error = 0;

    uprintf("Closing device \"mcasp\".\n");
    return (error);
}

/*
 * The read function just takes the buf that was saved via
 * echo_write() and returns it to userland for accessing.
 * uio(9)
 */
static int
mcasp_chdev_read(struct cdev *dev __unused, struct uio *uio, int ioflag __unused)
{
    int error = 0;

    uprintf("Reading device \"mcasp\".\n");

    return (error);
}

/*
 * echo_write takes in a character string and saves it
 * to buf for later accessing.
 */
static int
mcasp_chdev_write(struct cdev *dev __unused, struct uio *uio, int ioflag __unused)
{
    int error = 0;

    uprintf("Writing device \"mcasp\".\n");

    return (error);
}

static void
ti_mcasp_intr(void *arg)
{
//	struct ti_mcasp_softc *sc = arg;

//	_mcaspgeneric_intr(&sc->slot);
}

static int
ti_mcasp_detach(device_t dev)
{
    destroy_dev(mcasp_chdev);

    return (EBUSY);
}

static void
ti_mcasp_hw_init(device_t dev)
{
	int err;
	struct ti_mcasp_softc *sc = device_get_softc(dev);
//	uint32_t regval;
//	unsigned long timeout;

	/* Enable the controller and interface/functional clocks */
	if (ti_prcm_clk_enable(sc->mcasp_clk_id) != 0) {
		device_printf(dev, "Error: failed to enable McAsp clock\n");
		return;
	}

	/* Activate the McASP module. */
	err = ti_prcm_clk_enable(sc->mcasp_clk_id);
	if (err) {
		device_printf(dev, "Error: failed to activate McAsp source clock\n");
		return;
	}

//	/* Issue a softreset to the controller */
//	ti_mcasp_write_4(sc, MCASP_SYSCONFIG, MCASP_SYSCONFIG_RESET);
//	timeout = 1000;
//	while (!(ti_mcasp_read_4(sc, MCASP_SYSSTATUS) &
//	    MCASP_SYSSTATUS_RESETDONE)) {
//		if (--timeout == 0) {
//			device_printf(dev,
//			    "Error: Controller reset operation timed out\n");
//			break;
//		}
//		DELAY(100);
//	}
}

static int
ti_mcasp_attach(device_t dev)
{
	struct ti_mcasp_softc *sc = device_get_softc(dev);
	int rid;
	int err;
//	pcell_t prop;
	phandle_t node;

	sc->dev = dev;

	/*
	 * Get the McAsp  device id from FDT.  If it's not there use the newbus
	 * unit number (which will work as long as the devices are in order and
	 * none are skipped in the fdt).  Note that this is a property we made
	 * up and added in freebsd, it doesn't exist in the published bindings.
	 */
	node = ofw_bus_get_node(dev);
	sc->mcasp_clk_id = ti_hwmods_get_clock(dev);
	if (sc->mcasp_clk_id == INVALID_CLK_IDENT) {
		device_printf(dev, "failed to get clock based on hwmods property\n");
	}

	/*
	 * Set the offset from the device's memory start to the McAsp registers.
	 * Also for OMAP4 disable high speed mode due to erratum ID i626.
	 */
	switch (ti_chip()) {
#ifdef SOC_OMAP4
	case CHIP_OMAP_4:
		break;
#endif
#ifdef SOC_TI_AM335X
	case CHIP_AM335X:
		break;
#endif
	default:
		panic("Unknown OMAP device\n");
	}

	/* Resource setup. */
	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->mem_res) {
		device_printf(dev, "cannot allocate memory window\n");
		err = ENXIO;
		goto fail;
	}

	printf("+++ ti_mcasp_attach mcasp mem_res: %p \n", sc->mem_res);

	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (!sc->irq_res) {
		device_printf(dev, "cannot allocate interrupt\n");
		err = ENXIO;
		goto fail;
	}

	printf("+++ ti_mcasp_attach mcasp irq_res: %p \n", sc->irq_res);

	if (bus_setup_intr(dev, sc->irq_res, INTR_TYPE_BIO | INTR_MPSAFE,
	    NULL, ti_mcasp_intr, sc, &sc->intr_cookie)) {
		device_printf(dev, "cannot setup interrupt handler\n");
		err = ENXIO;
		goto fail;
	}


	/* Initialise the McAsp hardware. */
	ti_mcasp_hw_init(dev);

	if(make_dev_p(MAKEDEV_CHECKNAME | MAKEDEV_WAITOK,
	    &mcasp_chdev,
	    &mcasp_chdev_sw,
	    0,
	    UID_ROOT,
	    GID_WHEEL,
	    0600, "mcasp0")) {
	
		device_printf(dev, "cannot allocate character device\n");
		err = ENXIO;
		goto fail;
	}

	mcasp_chdev->si_drv1 = dev;

	printf("+++ ti_mcasp_attach mcasp mcasp_chdev: %p \n", mcasp_chdev);

	return (0);

fail:
	if (sc->intr_cookie)
		bus_teardown_intr(dev, sc->irq_res, sc->intr_cookie);
	if (sc->irq_res)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
	if (sc->mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);

	return (err);
}

static int
ti_mcasp_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "TI McAsp");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static device_method_t ti_mcasp_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		ti_mcasp_probe),
	DEVMETHOD(device_attach,	ti_mcasp_attach),
	DEVMETHOD(device_detach,	ti_mcasp_detach),

//	/* Bus interface */
//	DEVMETHOD(bus_read_ivar,	bus_generic_read_ivar),
//	DEVMETHOD(bus_write_ivar,	bus_generic_write_ivar),

	/* MCASP registers accessors */
//	DEVMETHOD(_mcaspread_4,		ti_mcasp_read_4),
//	DEVMETHOD(_mcaspwrite_4,		ti_mcasp_write_4),

	DEVMETHOD_END
};

static devclass_t ti_mcasp_devclass;

static driver_t ti_mcasp_driver = {
	"mcasp_ti",
	ti_mcasp_methods,
	sizeof(struct ti_mcasp_softc),
};

DRIVER_MODULE(mcasp_ti, simplebus, ti_mcasp_driver, ti_mcasp_devclass, NULL, NULL);
