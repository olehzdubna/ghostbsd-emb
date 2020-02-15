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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/intr.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm/ti/ti_cpuid.h>
#include <arm/ti/ti_prcm.h>
#include <arm/ti/ti_hwmods.h>
#include "gpio_if.h"

struct ti_gpmc_softc {
	device_t		dev;
//	struct sdhci_fdt_gpio * gpio;
	struct resource *	mem_res;
	struct resource *	irq_res;
	void *			intr_cookie;
	clk_ident_t		gpmc_clk_id;
	uint32_t		baseclk_hz;
	uint32_t		gpmc_reg_off;
};

/*
 * Table of supported FDT compat strings.
 *
 * Note that vendor Beaglebone dtsi files use "ti,omap3-hsmmc" for the am335x.
 */
static struct ofw_compat_data compat_data[] = {
	{NULL, 0},
};


/* Forward declarations, CAM-relataed */
// static void ti_sdhci_cam_poll(struct cam_sim *);
// static void ti_sdhci_cam_action(struct cam_sim *, union ccb *);
// static  int ti_sdhci_cam_settran_settings(struct ti_sdhci_softc *sc, union ccb *);

static inline uint32_t
ti_gpmc_read_4(struct ti_gpmc_softc *sc, bus_size_t off)
{
	return (bus_read_4(sc->mem_res, off + sc->gpmc_reg_off));
}

static inline void
ti_gpmc_write_4(struct ti_gpmc_softc *sc, bus_size_t off, uint32_t val)
{

	bus_write_4(sc->mem_res, off + sc->gpmc_reg_off, val);
}

static inline uint32_t
RD4(struct ti_gpmc_softc *sc, bus_size_t off)
{

	return (bus_read_4(sc->mem_res, off + sc->gpmc_reg_off));
}

static inline void
WR4(struct ti_gpmc_softc *sc, bus_size_t off, uint32_t val)
{

	bus_write_4(sc->mem_res, off + sc->gpmc_reg_off, val);
}


static void
ti_gpmc_intr(void *arg)
{
//	struct ti_gpmc_softc *sc = arg;

//	gpmc_generic_intr(&sc->slot);
}

static int
ti_gpmc_detach(device_t dev)
{

	/* gpmc_fdt_gpio_teardown(sc->gpio); */

	return (EBUSY);
}

static void
ti_gpmc_hw_init(device_t dev)
{
	struct ti_gpmc_softc *sc = device_get_softc(dev);
//	uint32_t regval;
//	unsigned long timeout;

	/* Enable the controller and interface/functional clocks */
	if (ti_prcm_clk_enable(sc->gpmc_clk_id) != 0) {
		device_printf(dev, "Error: failed to enable GPIO clock\n");
		return;
	}

	/* Get the frequency of the source clock */
	if (ti_prcm_clk_get_source_freq(sc->gpmc_clk_id,
	    &sc->baseclk_hz) != 0) {
		device_printf(dev, "Error: failed to get source clock freq\n");
		return;
	}

//	/* Issue a softreset to the controller */
//	ti_gpmc_write_4(sc, GPMC_SYSCONFIG, GPMC_SYSCONFIG_RESET);
//	timeout = 1000;
//	while (!(ti_gpmc_read_4(sc, GPMC_SYSSTATUS) &
//	    GPMC_SYSSTATUS_RESETDONE)) {
//		if (--timeout == 0) {
//			device_printf(dev,
//			    "Error: Controller reset operation timed out\n");
//			break;
//		}
//		DELAY(100);
//	}
}

static int
ti_gpmc_attach(device_t dev)
{
	struct ti_gpmc_softc *sc = device_get_softc(dev);
//	int rid;
	int err;
//	pcell_t prop;
	phandle_t node;

	sc->dev = dev;

	/*
	 * Get the gpmc device id from FDT.  If it's not there use the newbus
	 * unit number (which will work as long as the devices are in order and
	 * none are skipped in the fdt).  Note that this is a property we made
	 * up and added in freebsd, it doesn't exist in the published bindings.
	 */
	node = ofw_bus_get_node(dev);
	sc->gpmc_clk_id = ti_hwmods_get_clock(dev);
	if (sc->gpmc_clk_id == INVALID_CLK_IDENT) {
		device_printf(dev, "failed to get clock based on hwmods property\n");
	}

	/*
	 * Set the offset from the device's memory start to the gpmc registers.
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

	/*
	 * The standard GPMC registers are at a fixed offset (the same on all
	 * SoCs) beyond the GPMC registers.
	 */
//	sc->gpmc_reg_off = sc->gpmc_reg_off + GPMC_REG_OFFSET;
//
//	/* Resource setup. */
//	rid = 0;
//	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
//	    RF_ACTIVE);
//	if (!sc->mem_res) {
//		device_printf(dev, "cannot allocate memory window\n");
//		err = ENXIO;
//		goto fail;
//	}
//
//	rid = 0;
//	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
//	    RF_ACTIVE);
//	if (!sc->irq_res) {
//		device_printf(dev, "cannot allocate interrupt\n");
//		err = ENXIO;
//		goto fail;
//	}
//
//	if (bus_setup_intr(dev, sc->irq_res, INTR_TYPE_BIO | INTR_MPSAFE,
//	    NULL, ti_gpmc_intr, sc, &sc->intr_cookie)) {
//		device_printf(dev, "cannot setup interrupt handler\n");
//		err = ENXIO;
//		goto fail;
//	}
//

	/* Initialise the gpmc hardware. */
	ti_gpmc_hw_init(dev);

	return (0);

//fail:
//	if (sc->intr_cookie)
//		bus_teardown_intr(dev, sc->irq_res, sc->intr_cookie);
//	if (sc->irq_res)
//		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
//	if (sc->mem_res)
//		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);

	return (err);
}

static int
ti_gpmc_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "TI GPMC");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static device_method_t ti_gpmc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		ti_gpmc_probe),
	DEVMETHOD(device_attach,	ti_gpmc_attach),
	DEVMETHOD(device_detach,	ti_gpmc_detach),

//	/* Bus interface */
//	DEVMETHOD(bus_read_ivar,	bus_generic_read_ivar),
//	DEVMETHOD(bus_write_ivar,	bus_generic_write_ivar),

	/* GPMC registers accessors */
//	DEVMETHOD(gpmc_read_4,		ti_gpmc_read_4),
//	DEVMETHOD(gpmc_write_4,		ti_gpmc_write_4),

	DEVMETHOD_END
};

static devclass_t ti_gpmc_devclass;

static driver_t ti_gpmc_driver = {
	"gpmc_ti",
	ti_gpmc_methods,
	sizeof(struct ti_gpmc_softc),
};

DRIVER_MODULE(gpmc_ti, simplebus, ti_gpmc_driver, ti_gpmc_devclass, NULL, NULL);
