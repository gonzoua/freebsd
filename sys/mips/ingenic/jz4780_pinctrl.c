/*-
 * Copyright 2015 Alexander Kabaev <kan@FreeBSD.org>
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

/*
 * Ingenic JZ4780 pinctrl driver.
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/resource.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <mips/ingenic/jz4780_regs.h>

#define	JZ4780_CHIP_PINS 32

struct jz4780_pin_chip {
	struct resource	*chip_res;
	uint32_t	 chip_pullups;
	uint32_t	 chip_pulldowns;
	uint32_t	 chip_index;
	phandle_t	 chip_node;
	STAILQ_ENTRY(jz4780_pin_chip) chip_link;
};

struct jz4780_pinctrl_softc {
	device_t			dev;
	struct resource			*res[1];
	struct mtx			mtx;
	STAILQ_HEAD(, jz4780_pin_chip)	chips;
};

static struct resource_spec jz4780_pinctrl_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	{ -1, 0 }
};

#define	PINCTRL_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	PINCTRL_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	PINCTRL_LOCK_INIT(sc)	\
    mtx_init(&(sc)->mtx, device_get_nameunit((sc)->dev),	\
    "pinctrl", MTX_DEF)
#define	PINCTRL_LOCK_DESTROY(sc)	mtx_destroy(&(sc)->mtx);

#define REG_WRITE(sc, reg, val)		bus_write_4((sc)->res[0], (reg), (val))
#define REG_READ(sc, reg, val)		bus_read_4((sc)->res[0], (reg))
#define REG_OFFSET(chip, reg)		(((chip)->chip_index << 8) + (reg))

#define PINCTRL_WRITE(sc, chip, reg, val) \
    REG_WRITE((sc), REG_OFFSET((chip), (reg)), (val))
#define PINCTRL_READ(sc, chip, reg) \
    REG_READ((sc), REG_OFFSET((chip), (reg)))

static int
jz4780_pinctrl_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "ingenic,jz4780-pinctrl"))
		return (ENXIO);

	device_set_desc(dev, "Ingenic JZ4780 GPIO");

	return (BUS_PROBE_DEFAULT);
}

static int
jz4780_pinctrl_attach(device_t dev)
{
	struct jz4780_pinctrl_softc *sc;
	struct jz4780_pin_chip *chip;
	int i;
	phandle_t dt_parent, dt_child;

	sc = device_get_softc(dev);
	sc->dev = dev;

	if (bus_alloc_resources(dev, jz4780_pinctrl_spec, sc->res)) {
		device_printf(dev, "could not allocate resources for device\n");
		return (ENXIO);
	}

	PINCTRL_LOCK_INIT(sc);
	STAILQ_INIT(&sc->chips);

	/* Iterate over this node children, looking for pin controllers */
	dt_parent = ofw_bus_get_node(dev);
	i = 0;
	for (dt_child = OF_child(dt_parent); dt_child != 0;
	    dt_child = OF_peer(dt_child)) {
		if (!OF_hasprop(dt_child, "gpio-controller"))
			continue;

		chip = malloc(sizeof(*chip), M_DEVBUF, M_WAITOK|M_ZERO);
		chip->chip_index = i++;
		chip->chip_node = dt_child;

		OF_getencprop(dt_child, "ingenic,pull-ups",
		    &chip->chip_pullups, sizeof(chip->chip_pullups));
		OF_getencprop(dt_child, "ingenic,pull-downs",
		    &chip->chip_pulldowns, sizeof(chip->chip_pulldowns));

		if (bootverbose)
			device_printf(dev,
			    "Chip controller %u pull-ups 0x%08x pull-downs 0x%08x\n",
			    chip->chip_index, chip->chip_pullups,
			    chip->chip_pulldowns);
		STAILQ_INSERT_TAIL(&sc->chips, chip, chip_link);
	}

	fdt_pinctrl_register(dev, "ingenic,pins");
	fdt_pinctrl_configure_tree(dev);

	return (0);
}

static int
jz4780_pinctrl_detach(device_t dev)
{
	struct jz4780_pinctrl_softc *sc = device_get_softc(dev);
	struct jz4780_pin_chip *chip;

	while ((chip = STAILQ_FIRST(&sc->chips)) != NULL) {
		STAILQ_REMOVE_HEAD(&sc->chips, chip_link);
		free(chip, M_DEVBUF);
	}

	PINCTRL_LOCK_DESTROY(sc);
	bus_release_resources(dev, jz4780_pinctrl_spec, sc->res);

	return (0);
}

struct jx4780_bias_prop {
	const char *name;
	uint32_t    bias;
};

static struct jx4780_bias_prop jx4780_bias_table[] = {
	{ "bias-disable", GPIO_PIN_TRISTATE },
	{ "bias-pull-up", GPIO_PIN_PULLUP },
	{ "bias-pull-down", GPIO_PIN_PULLDOWN },
};

static int
jz4780_pinctrl_parse_pincfg(phandle_t pincfgxref, uint32_t *bias_value)
{
	phandle_t pincfg_node;
	int i;

	pincfg_node = OF_node_from_xref(pincfgxref);
	for (i = 0; i < nitems(jx4780_bias_table); i++) {
		if (OF_hasprop(pincfg_node, jx4780_bias_table[i].name)) {
			*bias_value = jx4780_bias_table[i].bias;
			return 0;
		}
	}

	return -1;
}

static struct jz4780_pin_chip *
jz4780_pinctrl_chip_lookup(struct jz4780_pinctrl_softc *sc, phandle_t chipxref)
{
	struct jz4780_pin_chip *chip;
	phandle_t chip_node;

	chip_node = OF_node_from_xref(chipxref);
	STAILQ_FOREACH(chip, &sc->chips, chip_link) {
		if (chip->chip_node == chip_node)
			return chip;
	}
	return NULL;
}

static void
jz4780_pinctrl_pin_set_func(struct jz4780_pinctrl_softc *sc,
    struct jz4780_pin_chip *chip, uint32_t pin, uint32_t func)
{
	uint32_t mask = (1u << pin);

	PINCTRL_WRITE(sc, chip, JZ_GPIO_INTC, mask);
	PINCTRL_WRITE(sc, chip, JZ_GPIO_MASKC, mask);
	if (func & 2)
		PINCTRL_WRITE(sc, chip, JZ_GPIO_PAT1S, mask);
	else
		PINCTRL_WRITE(sc, chip, JZ_GPIO_PAT1C, mask);
	if (func & 1)
		PINCTRL_WRITE(sc, chip, JZ_GPIO_PAT0S, mask);
	else
		PINCTRL_WRITE(sc, chip, JZ_GPIO_PAT0C, mask);
}

static int
jz4780_pinctrl_pin_set_bias(struct jz4780_pinctrl_softc *sc,
    struct jz4780_pin_chip *chip, uint32_t pin, uint32_t bias)
{
	uint32_t mask = (1u << pin);

	switch (bias) {
	case GPIO_PIN_TRISTATE:
		PINCTRL_WRITE(sc, chip, JZ_GPIO_DPULLS, mask);
		break;
	case GPIO_PIN_PULLUP:
		if (chip->chip_pullups & mask)
			PINCTRL_WRITE(sc, chip, JZ_GPIO_DPULLC, mask);
		else
			return EINVAL;
		break;
	case GPIO_PIN_PULLDOWN:
		if (chip->chip_pulldowns & mask)
			PINCTRL_WRITE(sc, chip, JZ_GPIO_DPULLC, mask);
		else
			return EINVAL;
		break;
	default:
		return ENOTSUP;
	}
	return 0;
}

static int
jz4780_pinctrl_configure_pins(device_t dev, phandle_t cfgxref)
{
	struct jz4780_pinctrl_softc *sc = device_get_softc(dev);
	struct jz4780_pin_chip *chip;
	phandle_t node;
	ssize_t i, len;
	uint32_t *value, *pconf;
	int result;

	node = OF_node_from_xref(cfgxref);

	len = OF_getencprop_alloc(node, "ingenic,pins", sizeof(uint32_t) * 4,
	    (void **)&value);
	if (len < 0) {
		device_printf(dev,
		    "missing ingenic,pins attribute in FDT\n");
		return (ENXIO);
	}

	pconf = value;
	result = EINVAL;
	for (i = 0; i < len; i++, pconf += 4) {
		uint32_t bias;

		/* Validate the configuration */
		/* Lookup the chip that handles this configuration */
		chip = jz4780_pinctrl_chip_lookup(sc, pconf[0]);
		if (chip == NULL) {
			device_printf(dev,
			    "Invalid gpio chip reference in FDT\n");
			goto done;
		}
		if (pconf[1] >= JZ4780_CHIP_PINS) {
			device_printf(dev,
			    "Invalid pin %u in FDT\n", pconf[1]);
			goto done;
		}
		if (pconf[2] > 3) {
			device_printf(dev,
			    "Invalid function %u for pin %u in FDT\n",
			    pconf[2], pconf[1]);
			goto done;
		}
		if (jz4780_pinctrl_parse_pincfg(pconf[3], &bias) != 0) {
			device_printf(dev,
			    "Invalid pin bias for pin %u in FDT\n",
			    pconf[1]);
			goto done;
		}
		if (bootverbose)
			device_printf(sc->dev,
			    "chip %u pin %u function %u bias %u\n",
			    chip->chip_index, pconf[1], pconf[2], bias);
		/* Configure pin function as requested */
		jz4780_pinctrl_pin_set_func(sc, chip, pconf[1], pconf[2]);
		/* Configure pin bias as requested */
		if (jz4780_pinctrl_pin_set_bias(sc, chip, pconf[1], bias) != 0) {
			device_printf(dev,
			    "Unable to set pin bias for chip %u pin %u bias: %u\n",
			    chip->chip_index, pconf[1], bias);
			goto done;
		}
	}

	result = 0;
done:
	free(value, M_OFWPROP);
	return (result);
}


static device_method_t jz4780_pinctrl_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		jz4780_pinctrl_probe),
	DEVMETHOD(device_attach,	jz4780_pinctrl_attach),
	DEVMETHOD(device_detach,	jz4780_pinctrl_detach),

	/* fdt_pinctrl interface */
	DEVMETHOD(fdt_pinctrl_configure, jz4780_pinctrl_configure_pins),

	DEVMETHOD_END
};

static driver_t jz4780_pinctrl_driver = {
	"pinctrl",
	jz4780_pinctrl_methods,
	sizeof(struct jz4780_pinctrl_softc),
};

static devclass_t jz4780_pinctrl_devclass;

EARLY_DRIVER_MODULE(pinctrl, simplebus, jz4780_pinctrl_driver,
    jz4780_pinctrl_devclass, 0, 0,  BUS_PASS_CPU + BUS_PASS_ORDER_LATE);
