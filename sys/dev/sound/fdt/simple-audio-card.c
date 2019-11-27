/*-
 * Copyright (c) 2019 Oleksandr Tymoshenko <gonzo@FreeBSD.org>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/clock.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/endian.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

struct simpleaudio_softc {
	device_t	dev;
};

static struct ofw_compat_data compat_data[] = {
	{"simple-audio-card",	1},
	{NULL,			0},
};

static int	simpleaudio_probe(device_t dev);
static int	simpleaudio_attach(device_t dev);
static int	simpleaudio_detach(device_t dev);

static int
simpleaudio_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "simple-audio-card");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
simpleaudio_attach(device_t dev)
{
	struct simpleaudio_softc	*sc;
	
	sc = device_get_softc(dev);
	sc->dev = dev;

	return (0);
}

static int
simpleaudio_detach(device_t dev)
{

	return (0);
}

static device_method_t simpleaudio_methods[] = {
        /* device_if methods */
	DEVMETHOD(device_probe,		simpleaudio_probe),
	DEVMETHOD(device_attach,	simpleaudio_attach),
	DEVMETHOD(device_detach,	simpleaudio_detach),

	DEVMETHOD_END,
};

static driver_t simpleaudio_driver = {
	"simpleaudio",
	simpleaudio_methods,
	sizeof(struct simpleaudio_softc),
};
static devclass_t simpleaudio_devclass;

DRIVER_MODULE(simpleaudiocodec, simplebus, simpleaudio_driver, simpleaudio_devclass, NULL, NULL);
MODULE_VERSION(simpleaudiocodec, 1);
