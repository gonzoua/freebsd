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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/resource.h>
#include <sys/gpio.h>

#include <machine/bus.h>
#include <machine/intr.h>

#include <dev/gpio/gpiobusvar.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <mips/ingenic/jz4780_regs.h>
#include <gnu/dts/include/dt-bindings/interrupt-controller/irq.h>

#include "jz4780_gpio_if.h"
#include "gpio_if.h"
#include "pic_if.h"

#define JZ4780_GPIO_PINS 32

enum pin_function {
	JZ_FUNC_DEV_0,
	JZ_FUNC_DEV_1,
	JZ_FUNC_DEV_2,
	JZ_FUNC_DEV_3,
	JZ_FUNC_GPIO,
	JZ_FUNC_INTR,
};

struct jz4780_gpio_pin {
	uint32_t pin_caps;
	uint32_t pin_flags;
	enum pin_function pin_func;
	enum intr_trigger intr_trigger;
	enum intr_polarity intr_polarity;
	char pin_name[GPIOMAXNAME];
	struct arm_irqsrc *pin_irqsrc;
};

struct jz4780_gpio_softc {
	device_t		dev;
	device_t		busdev;
	struct resource		*res[2];
	struct mtx		mtx;
	struct jz4780_gpio_pin  pins[JZ4780_GPIO_PINS];
	void			*intrhand;
};

static struct resource_spec jz4780_gpio_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	{ SYS_RES_IRQ,    0, RF_ACTIVE },
	{ -1, 0 }
};

static int jz4780_gpio_probe(device_t dev);
static int jz4780_gpio_attach(device_t dev);
static int jz4780_gpio_detach(device_t dev);
static int jz4780_gpio_intr(void *arg);

#define	JZ4780_GPIO_LOCK(sc)		mtx_lock_spin(&(sc)->mtx)
#define	JZ4780_GPIO_UNLOCK(sc)		mtx_unlock_spin(&(sc)->mtx)
#define	JZ4780_GPIO_LOCK_INIT(sc)	\
    mtx_init(&(sc)->mtx, device_get_nameunit((sc)->dev),	\
    "jz4780_gpio", MTX_SPIN)
#define	JZ4780_GPIO_LOCK_DESTROY(sc)	mtx_destroy(&(sc)->mtx);

#define CSR_WRITE_4(sc, reg, val)	bus_write_4((sc)->res[0], (reg), (val))
#define CSR_READ_4(sc, reg)		bus_read_4((sc)->res[0], (reg))

static int
jz4780_gpio_probe(device_t dev)
{
	phandle_t node;

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	/* We only like particular parent */
	if (!ofw_bus_is_compatible(device_get_parent(dev),
	   "ingenic,jz4780-pinctrl"))
		return (ENXIO);

	/* ... and only specific children os that parent */
	node = ofw_bus_get_node(dev);
	if (!OF_hasprop(node, "gpio-controller"))
		return (ENXIO);

	device_set_desc(dev, "Ingenic JZ4780 GPIO Controller");

	return (BUS_PROBE_DEFAULT);
}

static int
jz4780_gpio_pin_set_func(struct jz4780_gpio_softc *sc, uint32_t pin,
    uint32_t func)
{
	uint32_t mask = (1u << pin);

	if (func > (uint32_t)JZ_FUNC_DEV_3)
		return (EINVAL);

	CSR_WRITE_4(sc, JZ_GPIO_INTC, mask);
	CSR_WRITE_4(sc, JZ_GPIO_MASKC, mask);
	if (func & 2)
		CSR_WRITE_4(sc, JZ_GPIO_PAT1S, mask);
	else
		CSR_WRITE_4(sc, JZ_GPIO_PAT1C, mask);
	if (func & 1)
		CSR_WRITE_4(sc, JZ_GPIO_PAT0S, mask);
	else
		CSR_WRITE_4(sc, JZ_GPIO_PAT0C, mask);

	sc->pins[pin].pin_flags &= ~(GPIO_PIN_INPUT | GPIO_PIN_OUTPUT);
	sc->pins[pin].pin_func = (enum pin_function)func;
	return (0);
}

static int
jz4780_gpio_pin_set_direction(struct jz4780_gpio_softc *sc,
    uint32_t pin, uint32_t dir)
{
	uint32_t mask = (1u << pin);

	switch (dir) {
	case GPIO_PIN_OUTPUT:
		if (sc->pins[pin].pin_caps & dir)
			CSR_WRITE_4(sc, JZ_GPIO_PAT1C, mask);
		else
			return (EINVAL);
		break;
	case GPIO_PIN_INPUT:
		if (sc->pins[pin].pin_caps & dir)
			CSR_WRITE_4(sc, JZ_GPIO_PAT1S, mask);
		else
			return (EINVAL);
		break;
	}

	sc->pins[pin].pin_flags &= ~(GPIO_PIN_INPUT | GPIO_PIN_OUTPUT);
	sc->pins[pin].pin_flags |= dir;
	return (0);
}

static int
jz4780_gpio_pin_set_bias(struct jz4780_gpio_softc *sc,
    uint32_t pin, uint32_t bias)
{
	uint32_t mask = (1u << pin);

	switch (bias) {
	case GPIO_PIN_PULLUP:
	case GPIO_PIN_PULLDOWN:
		if (sc->pins[pin].pin_caps & bias)
			CSR_WRITE_4(sc, JZ_GPIO_DPULLC, mask);
		else
			return (EINVAL);
		break;
	case 0:
		CSR_WRITE_4(sc, JZ_GPIO_DPULLS, mask);
		break;
	default:
		return (ENOTSUP);
	}

	sc->pins[pin].pin_flags &= ~(GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN);
	sc->pins[pin].pin_flags |= bias;
	return (0);
}

/*
 * Decode pin configuration using this map
 */
#if 0
INT MASK PAT1 PAT0
1   x    0    0 /* intr, level, low */
1   x    0    1 /* intr, level, high */
1   x    1    0 /* intr, edge, falling */
1   x    1    1 /* intr, edge, rising */
0   0    0    0 /* function, func 0 */
0   0    0    1 /* function, func 1 */
0   0    1    0 /* function, func 2 */
0   0    1    0 /* function, func 3 */
0   1    0    0 /* gpio, output 0 */
0   1    0    1 /* gpio, output 1 */
0   1    1    x /* gpio, input */
#endif

static void
jz4780_gpio_pin_probe(struct jz4780_gpio_softc *sc, uint32_t pin)
{
	uint32_t mask = (1u << pin);
	uint32_t val;

	/* Clear cached gpio config */
	sc->pins[pin].pin_flags = 0;

	/* First check if pin is in interrupt mode */
	val = CSR_READ_4(sc, JZ_GPIO_INT);
	if (val & mask) {
		/* Pin is in interrupt mode, decode interrupt triggering mode */
		val = CSR_READ_4(sc, JZ_GPIO_PAT1);
		if (val & mask)
			sc->pins[pin].intr_trigger = INTR_TRIGGER_EDGE;
		else
			sc->pins[pin].intr_trigger = INTR_TRIGGER_LEVEL;
		/* Decode interrupt polarity */
		val = CSR_READ_4(sc, JZ_GPIO_PAT0);
		if (val & mask)
			sc->pins[pin].intr_polarity = INTR_POLARITY_HIGH;
		else
			sc->pins[pin].intr_polarity = INTR_POLARITY_LOW;

		sc->pins[pin].pin_func = JZ_FUNC_INTR;
		sc->pins[pin].pin_flags = 0;
		return;
	}
	/* Next check if pin is in gpio mode */
	val = CSR_READ_4(sc, JZ_GPIO_MASK);
	if (val & mask) {
		/* Pin is in gpio mode, decode direction and bias */
		val = CSR_READ_4(sc, JZ_GPIO_PAT1);
		if (val & mask)
			sc->pins[pin].pin_flags |= GPIO_PIN_OUTPUT;
		else
			sc->pins[pin].pin_flags |= GPIO_PIN_OUTPUT;
		/* Check for bias */
		val = CSR_READ_4(sc, JZ_GPIO_DPULL);
		if ((val & mask) == 0)
			sc->pins[pin].pin_flags |= sc->pins[pin].pin_caps &
				(GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN);
		sc->pins[pin].pin_func = JZ_FUNC_GPIO;
		return;
	}
	/* By exclusion, pin is in alternate function mode */
	val = CSR_READ_4(sc, JZ_GPIO_DPULL);
	if ((val & mask) == 0)
		sc->pins[pin].pin_flags = sc->pins[pin].pin_caps &
			(GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN);
	val = ((CSR_READ_4(sc, JZ_GPIO_PAT1) & mask) >> pin) << 1;
	val = val | ((CSR_READ_4(sc, JZ_GPIO_PAT1) & mask) >> pin);
	sc->pins[pin].pin_func = (enum pin_function)val;
}

static int
jz4780_gpio_attach(device_t dev)
{
	struct jz4780_gpio_softc *sc = device_get_softc(dev);
	phandle_t node;
	uint32_t i, pd_pins, pu_pins;

	sc->dev = dev;

	if (bus_alloc_resources(dev, jz4780_gpio_spec, sc->res)) {
		device_printf(dev, "could not allocate resources for device\n");
		return (ENXIO);
	}

	JZ4780_GPIO_LOCK_INIT(sc);

	node = ofw_bus_get_node(dev);
	OF_getencprop(node, "ingenic,pull-ups", &pu_pins, sizeof(pu_pins));
	OF_getencprop(node, "ingenic,pull-downs", &pd_pins, sizeof(pd_pins));

	for (i = 0; i < JZ4780_GPIO_PINS; i++) {
		sc->pins[i].pin_caps |= GPIO_PIN_INPUT | GPIO_PIN_OUTPUT;
		if (pu_pins & (1 << i))
			sc->pins[i].pin_caps |= GPIO_PIN_PULLUP;
		if (pd_pins & (1 << i))
			sc->pins[i].pin_caps |= GPIO_PIN_PULLDOWN;
		sc->pins[i].intr_polarity = INTR_POLARITY_LOW;
		sc->pins[i].intr_trigger = INTR_TRIGGER_LEVEL;

		snprintf(sc->pins[i].pin_name, GPIOMAXNAME - 1, "gpio%c%d",
		    device_get_unit(dev) + 'a', i);
		sc->pins[i].pin_name[GPIOMAXNAME - 1] = '\0';

		jz4780_gpio_pin_probe(sc, i);
	}

	if (arm_pic_register(dev, OF_xref_from_node(node)) != 0) {
		device_printf(dev, "could not register PIC\n");
		goto fail;
	}

	if (bus_setup_intr(dev, sc->res[1], INTR_TYPE_MISC | INTR_MPSAFE,
	    jz4780_gpio_intr, NULL, sc, &sc->intrhand) != 0)
		goto fail_pic;

	sc->busdev = gpiobus_attach_bus(dev);
	if (sc->busdev == NULL)
		goto fail_pic;

	return (0);
fail_pic:
	arm_pic_unregister(dev, OF_xref_from_node(node));
fail:
	if (sc->intrhand != NULL)
		bus_teardown_intr(dev, sc->res[1], sc->intrhand);
	bus_release_resources(dev, jz4780_gpio_spec, sc->res);
	JZ4780_GPIO_LOCK_DESTROY(sc);
	return (ENXIO);
}

static int
jz4780_gpio_detach(device_t dev)
{
	struct jz4780_gpio_softc *sc = device_get_softc(dev);

	bus_release_resources(dev, jz4780_gpio_spec, sc->res);
	JZ4780_GPIO_LOCK_DESTROY(sc);
	return (0);
}

static int
jz4780_gpio_configure_pin(device_t dev, uint32_t pin, uint32_t func,
    uint32_t flags)
{
	struct jz4780_gpio_softc *sc;
	int retval;

	if (pin >= JZ4780_GPIO_PINS)
		return (EINVAL);

	sc = device_get_softc(dev);
	JZ4780_GPIO_LOCK(sc);
	retval = jz4780_gpio_pin_set_func(sc, pin, func);
	if (retval == 0)
		retval = jz4780_gpio_pin_set_bias(sc, pin, flags);
	JZ4780_GPIO_UNLOCK(sc);
	return (retval);
}

static device_t
jz4780_gpio_get_bus(device_t dev)
{
	struct jz4780_gpio_softc *sc;

	sc = device_get_softc(dev);

	return (sc->busdev);
}

static int
jz4780_gpio_pin_max(device_t dev, int *maxpin)
{

	*maxpin = JZ4780_GPIO_PINS - 1;
	return (0);
}

static int
jz4780_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	struct jz4780_gpio_softc *sc;

	if (pin >= JZ4780_GPIO_PINS)
		return (EINVAL);

	sc = device_get_softc(dev);
	JZ4780_GPIO_LOCK(sc);
	*caps = sc->pins[pin].pin_caps;
	JZ4780_GPIO_UNLOCK(sc);

	return (0);
}

static int
jz4780_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct jz4780_gpio_softc *sc;

	if (pin >= JZ4780_GPIO_PINS)
		return (EINVAL);

	sc = device_get_softc(dev);
	JZ4780_GPIO_LOCK(sc);
	*flags = sc->pins[pin].pin_flags;
	JZ4780_GPIO_UNLOCK(sc);

	return (0);
}

static int
jz4780_gpio_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct jz4780_gpio_softc *sc;

	if (pin >= JZ4780_GPIO_PINS)
		return (EINVAL);

	sc = device_get_softc(dev);
	strncpy(name, sc->pins[pin].pin_name, GPIOMAXNAME - 1);
	name[GPIOMAXNAME - 1] = '\0';

	return (0);
}

static int
jz4780_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct jz4780_gpio_softc *sc;
	int retval;

	if (pin >= JZ4780_GPIO_PINS)
		return (EINVAL);

	sc = device_get_softc(dev);
	JZ4780_GPIO_LOCK(sc);
	retval = jz4780_gpio_pin_set_direction(sc, pin,
	    flags & (GPIO_PIN_INPUT | GPIO_PIN_OUTPUT));
	if (retval == 0)
		retval = jz4780_gpio_pin_set_bias(sc, pin,
		    flags & (GPIO_PIN_PULLDOWN | GPIO_PIN_PULLUP));
	JZ4780_GPIO_UNLOCK(sc);

	return (retval);
}

static int
jz4780_gpio_pin_set(device_t dev, uint32_t pin, unsigned int value)
{
	struct jz4780_gpio_softc *sc;
	uint32_t mask;
	int retval;

	if (pin >= JZ4780_GPIO_PINS)
		return (EINVAL);

	retval = EINVAL;
	mask = (1u << pin);
	sc = device_get_softc(dev);
	JZ4780_GPIO_LOCK(sc);
	if (sc->pins[pin].pin_func == JZ_FUNC_GPIO) {
		CSR_WRITE_4(sc, value ? JZ_GPIO_PAT0S : JZ_GPIO_PAT0C, mask);
		retval = 0;
	}
	JZ4780_GPIO_UNLOCK(sc);

	return (retval);
}

static int
jz4780_gpio_pin_get(device_t dev, uint32_t pin, unsigned int *val)
{
	struct jz4780_gpio_softc *sc;
	uint32_t data, mask;

	if (pin >= JZ4780_GPIO_PINS)
		return (EINVAL);

	mask = (1u << pin);
	sc = device_get_softc(dev);
	JZ4780_GPIO_LOCK(sc);
	data = CSR_READ_4(sc, JZ_GPIO_PIN);
	JZ4780_GPIO_UNLOCK(sc);
	*val = (data & mask) ? 1 : 0;

	return (0);
}

static int
jz4780_gpio_pin_toggle(device_t dev, uint32_t pin)
{
	struct jz4780_gpio_softc *sc;
	uint32_t data, mask;
	int retval;

	if (pin >= JZ4780_GPIO_PINS)
		return (EINVAL);

	retval = EINVAL;
	mask = (1u << pin);
	sc = device_get_softc(dev);
	JZ4780_GPIO_LOCK(sc);
	if (sc->pins[pin].pin_func == JZ_FUNC_GPIO &&
	    sc->pins[pin].pin_flags & GPIO_PIN_OUTPUT) {
		data = CSR_READ_4(sc, JZ_GPIO_PIN);
		CSR_WRITE_4(sc, (data & mask) ? JZ_GPIO_PAT0C : JZ_GPIO_PAT0S,
		    mask);
		retval = 0;
	}
	JZ4780_GPIO_UNLOCK(sc);

	return (retval);
}

static int
jz4780_gpio_pic_register(device_t dev, struct arm_irqsrc *isrc, boolean_t *is_percpu)
{
	struct jz4780_gpio_softc *sc;
	uint32_t pin, tripol;

	sc = device_get_softc(dev);


	if (isrc->isrc_ncells != 2) {
		device_printf(sc->dev, "Invalid #interrupt-cells");
		return (EINVAL);
	}

	pin = isrc->isrc_cells[0];
	tripol = isrc->isrc_cells[1];
	if (pin >= JZ4780_GPIO_PINS) {
		device_printf(sc->dev, "Invalid interrupt number %d", pin);
		return (EINVAL);
	}
	switch (tripol)
	{
	case IRQ_TYPE_EDGE_RISING:
		isrc->isrc_trig = INTR_TRIGGER_EDGE;
		isrc->isrc_pol  = INTR_POLARITY_HIGH;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		isrc->isrc_trig = INTR_TRIGGER_EDGE;
		isrc->isrc_pol  = INTR_POLARITY_LOW;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		isrc->isrc_trig = INTR_TRIGGER_LEVEL;
		isrc->isrc_pol  = INTR_POLARITY_HIGH;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		isrc->isrc_trig = INTR_TRIGGER_LEVEL;
		isrc->isrc_pol  = INTR_POLARITY_LOW;
		break;
	default:
		device_printf(sc->dev, "unsupported trigger/polarity 0x%2x\n",
		    tripol);
		return (ENOTSUP);
	}
	isrc->isrc_nspc_type = ARM_IRQ_NSPC_PLAIN;
	isrc->isrc_nspc_num = pin;

	/*
	 * 1. The link between ISRC and controller must be set atomically.
	 * 2. Just do things only once in rare case when consumers
	 *    of shared interrupt came here at the same moment.
	 */
	JZ4780_GPIO_LOCK(sc);
	if (sc->pins[pin].pin_irqsrc != NULL) {
		JZ4780_GPIO_UNLOCK(sc);
		return (sc->pins[pin].pin_irqsrc == isrc ? 0 : EEXIST);
	}
	sc->pins[pin].pin_irqsrc = isrc;
	isrc->isrc_data = pin;
	JZ4780_GPIO_UNLOCK(sc);

	arm_irq_set_name(isrc, "%s,%u", device_get_nameunit(sc->dev), pin);
	return (0);
}

static int
jz4780_gpio_pic_unregister(device_t dev, struct arm_irqsrc *isrc)
{
	struct jz4780_gpio_softc *sc;
	u_int irq;

	sc = device_get_softc(dev);

	JZ4780_GPIO_LOCK(sc);
	irq = isrc->isrc_data;
	if (sc->pins[irq].pin_irqsrc != isrc) {
		JZ4780_GPIO_UNLOCK(sc);
		return (sc->pins[irq].pin_irqsrc == NULL ? 0 : EINVAL);
	}
	sc->pins[irq].pin_irqsrc = NULL;
	isrc->isrc_data = 0;
	JZ4780_GPIO_UNLOCK(sc);

	arm_irq_set_name(isrc, "%s", "");
	return (0);
}

static void
jz4780_gpio_pic_enable_intr(device_t dev, struct arm_irqsrc *isrc)
{
	struct jz4780_gpio_softc *sc;
	uint32_t pin, mask;

	sc = device_get_softc(dev);

	pin = isrc->isrc_data;
	mask = 1u << pin;

	JZ4780_GPIO_LOCK(sc);
	CSR_WRITE_4(sc, JZ_GPIO_MASKS, mask);
	CSR_WRITE_4(sc, JZ_GPIO_INTS, mask);

	if (isrc->isrc_trig == INTR_TRIGGER_LEVEL)
		CSR_WRITE_4(sc, JZ_GPIO_PAT1C, mask);
	else
		CSR_WRITE_4(sc, JZ_GPIO_PAT1S, mask);

	if (isrc->isrc_pol == INTR_POLARITY_LOW)
		CSR_WRITE_4(sc, JZ_GPIO_PAT0C, mask);
	else
		CSR_WRITE_4(sc, JZ_GPIO_PAT0S, mask);

	sc->pins[pin].pin_func = JZ_FUNC_INTR;
	sc->pins[pin].intr_trigger = isrc->isrc_trig;
	sc->pins[pin].intr_polarity = isrc->isrc_pol;

	CSR_WRITE_4(sc, JZ_GPIO_FLAGC, mask);
	CSR_WRITE_4(sc, JZ_GPIO_MASKC, mask);
	JZ4780_GPIO_UNLOCK(sc);
}

static void
jz4780_gpio_pic_enable_source(device_t dev, struct arm_irqsrc *isrc)
{
	struct jz4780_gpio_softc *sc;

	sc = device_get_softc(dev);
	CSR_WRITE_4(sc, JZ_GPIO_MASKC, 1u << isrc->isrc_data);
}

static void
jz4780_gpio_pic_disable_source(device_t dev, struct arm_irqsrc *isrc)
{
	struct jz4780_gpio_softc *sc;

	sc = device_get_softc(dev);
	CSR_WRITE_4(sc, JZ_GPIO_MASKS, 1u << isrc->isrc_data);
}

static void
jz4780_gpio_pic_pre_ithread(device_t dev, struct arm_irqsrc *isrc)
{

	jz4780_gpio_pic_disable_source(dev, isrc);
}

static void
jz4780_gpio_pic_post_ithread(device_t dev, struct arm_irqsrc *isrc)
{

	jz4780_gpio_pic_enable_source(dev, isrc);
}

static void
jz4780_gpio_pic_post_filter(device_t dev, struct arm_irqsrc *isrc)
{
	struct jz4780_gpio_softc *sc;

	sc = device_get_softc(dev);
	CSR_WRITE_4(sc, JZ_GPIO_FLAGC, 1u << isrc->isrc_data);
}

static int
jz4780_gpio_intr(void *arg)
{
	struct jz4780_gpio_softc *sc;
	uint32_t i, interrupts;

	sc = arg;
	interrupts = CSR_READ_4(sc, JZ_GPIO_FLAG);

	for (i = 0; interrupts != 0; i++, interrupts >>= 1) {
		if ((interrupts & 0x1) == 0)
			continue;
		if (sc->pins[i].pin_irqsrc)
			arm_irq_dispatch(sc->pins[i].pin_irqsrc, curthread->td_intr_frame);
		else
			device_printf(sc->dev, "spurious interrupt %d\n", i);
	}

	return (FILTER_HANDLED);
}

static device_method_t jz4780_gpio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		jz4780_gpio_probe),
	DEVMETHOD(device_attach,	jz4780_gpio_attach),
	DEVMETHOD(device_detach,	jz4780_gpio_detach),

	/* GPIO protocol */
	DEVMETHOD(gpio_get_bus,		jz4780_gpio_get_bus),
	DEVMETHOD(gpio_pin_max,		jz4780_gpio_pin_max),
	DEVMETHOD(gpio_pin_getname,	jz4780_gpio_pin_getname),
	DEVMETHOD(gpio_pin_getflags,	jz4780_gpio_pin_getflags),
	DEVMETHOD(gpio_pin_getcaps,	jz4780_gpio_pin_getcaps),
	DEVMETHOD(gpio_pin_setflags,	jz4780_gpio_pin_setflags),
	DEVMETHOD(gpio_pin_get,		jz4780_gpio_pin_get),
	DEVMETHOD(gpio_pin_set,		jz4780_gpio_pin_set),
	DEVMETHOD(gpio_pin_toggle,	jz4780_gpio_pin_toggle),

	/* Custom interface to set pin function */
	DEVMETHOD(jz4780_gpio_configure_pin, jz4780_gpio_configure_pin),

	/* Interrupt controller interface */
	DEVMETHOD(pic_disable_source,	jz4780_gpio_pic_disable_source),
	DEVMETHOD(pic_enable_intr,	jz4780_gpio_pic_enable_intr),
	DEVMETHOD(pic_enable_source,	jz4780_gpio_pic_enable_source),
	DEVMETHOD(pic_post_filter,	jz4780_gpio_pic_post_filter),
	DEVMETHOD(pic_post_ithread,	jz4780_gpio_pic_post_ithread),
	DEVMETHOD(pic_pre_ithread,	jz4780_gpio_pic_pre_ithread),
	DEVMETHOD(pic_register,		jz4780_gpio_pic_register),
	DEVMETHOD(pic_unregister,	jz4780_gpio_pic_unregister),

	DEVMETHOD_END
};

static driver_t jz4780_gpio_driver = {
	"gpio",
	jz4780_gpio_methods,
	sizeof(struct jz4780_gpio_softc),
};

static devclass_t jz4780_gpio_devclass;

EARLY_DRIVER_MODULE(jz4780_gpio, simplebus, jz4780_gpio_driver,
    jz4780_gpio_devclass, 0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
