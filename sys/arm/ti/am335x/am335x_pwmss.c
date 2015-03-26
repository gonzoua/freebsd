/*-
 * Copyright (c) 2013 Oleksandr Tymoshenko <gonzo@freebsd.org>
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/limits.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/fdt/simplebus.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm/ti/ti_hwmods.h>
#include <arm/ti/ti_prcm.h>
#include <arm/ti/ti_scm.h>

#include "am335x_pwm.h"
#include "am335x_scm.h"

#define	PWMSS_IDVER		0x00
#define	PWMSS_SYSCONFIG		0x04
#define	PWMSS_CLKCONFIG		0x08
#define		CLKCONFIG_EPWMCLK_EN	(1 << 8)
#define	PWMSS_CLKSTATUS		0x0C

static device_probe_t am335x_pwmss_probe;
static device_attach_t am335x_pwmss_attach;
static device_detach_t am335x_pwmss_detach;
        
struct am335x_pwmss_softc {
	device_t		sc_dev;
	int			sc_id;
};

static device_method_t am335x_pwmss_methods[] = {
	DEVMETHOD(device_probe,		am335x_pwmss_probe),
	DEVMETHOD(device_attach,	am335x_pwmss_attach),
	DEVMETHOD(device_detach,	am335x_pwmss_detach),

        /* Bus interface */
	DEVMETHOD(bus_alloc_resource, bus_generic_alloc_resource),
	DEVMETHOD(bus_release_resource,	bus_generic_release_resource),
	DEVMETHOD(bus_activate_resource, bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource,bus_generic_deactivate_resource),
	DEVMETHOD(bus_setup_intr, bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr, bus_generic_teardown_intr),
	DEVMETHOD(bus_get_resource_list, bus_generic_get_resource_list),

	/* OFW bus interface */
	DEVMETHOD(ofw_bus_get_devinfo,	ofw_bus_gen_get_devinfo),
	DEVMETHOD(ofw_bus_get_compat,	ofw_bus_gen_get_compat),
	DEVMETHOD(ofw_bus_get_model,	ofw_bus_gen_get_model),
	DEVMETHOD(ofw_bus_get_name,	ofw_bus_gen_get_name),
	DEVMETHOD(ofw_bus_get_node,	ofw_bus_gen_get_node),
	DEVMETHOD(ofw_bus_get_type,	ofw_bus_gen_get_type),

	DEVMETHOD_END
};

static driver_t am335x_pwmss_driver = {
	"am335x_pwmss",
	am335x_pwmss_methods,
	sizeof(struct am335x_pwmss_softc),
};

static devclass_t am335x_pwmss_devclass;

static int
am335x_pwmss_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "ti,am33xx-pwmss"))
		return (ENXIO);

	device_set_desc(dev, "AM335x PWM");

	return (BUS_PROBE_DEFAULT);
}

static int
am335x_pwmss_attach(device_t dev)
{
	struct am335x_pwmss_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	sc->sc_id = ti_hwmods_get_unit(dev, "pwmss");
	if (sc->sc_id < 0) {
		device_printf(dev, "failed to get device id based on ti,hwmods\n");
		return (EINVAL);
	}

	ti_prcm_clk_enable(PWMSS0_CLK + sc->sc_id);
	ti_scm_reg_read_4(SCM_PWMSS_CTRL, &reg);
	reg |= (1 << sc->sc_id);
	ti_scm_reg_write_4(SCM_PWMSS_CTRL, reg);

	simplebus_attach_children(dev);

	return (0);
}

static int
am335x_pwmss_detach(device_t dev)
{

	return (0);
}

DRIVER_MODULE(am335x_pwmss, simplebus, am335x_pwmss_driver, am335x_pwmss_devclass, 0, 0);
MODULE_VERSION(am335x_pwmss, 1);
MODULE_DEPEND(am335x_pwmss, simplebus, 1, 1, 1);
