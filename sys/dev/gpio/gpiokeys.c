/*-
 * Copyright (c) 2015 Oleksandr Tymoshenko <gonzo@freebsd.org>
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
#include "opt_kbd.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/kdb.h>

#include <sys/ioccom.h>
#include <sys/filio.h>
#include <sys/tty.h>
#include <sys/kbio.h>

#include <dev/kbd/kbdreg.h>
#include <dev/kbd/kbdtables.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/gpio/gpiokeys.h>

#define GPIOKEYS_LOCK(_sc)		mtx_lock(&(_sc)->sc_mtx)
#define	GPIOKEYS_UNLOCK(_sc)		mtx_unlock(&(_sc)->sc_mtx)
#define GPIOKEYS_LOCK_INIT(_sc) \
	mtx_init(&_sc->sc_mtx, device_get_nameunit(_sc->sc_dev), \
	    "gpiokeys", MTX_DEF)
#define GPIOKEYS_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->sc_mtx);

#define	KEY_PRESS	  0
#define	KEY_RELEASE	  0x80

#define	SCAN_PRESS	  0
#define	SCAN_RELEASE	  0x80
#define	SCAN_CHAR(c)	((c) & 0x7f)

#define	GPIOKEYS_GLOBAL_NMOD                     8	/* units */
#define	GPIOKEYS_GLOBAL_NKEYCODE                 6	/* units */
#define	GPIOKEYS_GLOBAL_IN_BUF_SIZE  (2*(GPIOKEYS_GLOBAL_NMOD + (2*GPIOKEYS_GLOBAL_NKEYCODE)))	/* bytes */
#define	GPIOKEYS_GLOBAL_IN_BUF_FULL  (GPIOKEYS_GLOBAL_IN_BUF_SIZE / 2)	/* bytes */
#define	GPIOKEYS_GLOBAL_NFKEY        (sizeof(fkey_tab)/sizeof(fkey_tab[0]))	/* units */
#define	GPIOKEYS_GLOBAL_BUFFER_SIZE	      64	/* bytes */

struct gpiokeys_softc 
{
	device_t	sc_dev;
	struct mtx	sc_mtx;

	keyboard_t	sc_kbd;
	keymap_t	sc_keymap;
	accentmap_t	sc_accmap;
	fkeytab_t	sc_fkeymap[GPIOKEYS_GLOBAL_NFKEY];

	uint32_t	sc_input[GPIOKEYS_GLOBAL_IN_BUF_SIZE];	/* input buffer */
	uint32_t	sc_time_ms;
#define	GPIOKEYS_GLOBAL_FLAG_POLLING	0x00000002

	uint32_t	sc_flags;		/* flags */

	int		sc_mode;		/* input mode (K_XLATE,K_RAW,K_CODE) */
	int		sc_state;		/* shift/lock key state */
	int		sc_accents;		/* accent key index (> 0) */
	int		sc_kbd_size;

	uint16_t	sc_inputs;
	uint16_t	sc_inputhead;
	uint16_t	sc_inputtail;

	uint8_t		sc_kbd_id;
};

struct gpiokeys_softc *gpiokeys_sc;

/* gpio-keys device */
static int gpiokeys_probe(device_t);
static int gpiokeys_attach(device_t);
static int gpiokeys_detach(device_t);

/* kbd methods prototypes */
static int	gpiokeys_set_typematic(keyboard_t *, int);
static uint32_t	gpiokeys_read_char(keyboard_t *, int);
static void	gpiokeys_clear_state(keyboard_t *);
static int	gpiokeys_ioctl(keyboard_t *, u_long, caddr_t);
static int	gpiokeys_enable(keyboard_t *);
static int	gpiokeys_disable(keyboard_t *);
static void	gpiokeys_event_keyinput(struct gpiokeys_softc *);

static int
gpiokeys_probe(device_t dev)
{
	if (!ofw_bus_is_compatible(dev, "gpio-keys"))
		return (ENXIO);

	device_set_desc(dev, "GPIO keyboard");

	return (0);
}

static int
gpiokeys_attach(device_t dev)
{
	int unit;
	struct gpiokeys_softc *sc;
	keyboard_t *kbd;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	kbd = &sc->sc_kbd;

	GPIOKEYS_LOCK_INIT(sc);
	unit = device_get_unit(dev);
	kbd_init_struct(kbd, "gpiokeys", KB_OTHER, unit, 0, 0, 0);

	kbd->kb_data = (void *)sc;
	sc->sc_mode = K_XLATE;

	sc->sc_keymap = key_map;
	sc->sc_accmap = accent_map;

	kbd_set_maps(kbd, &sc->sc_keymap, &sc->sc_accmap,
	    sc->sc_fkeymap, GPIOKEYS_GLOBAL_NFKEY);

	KBD_FOUND_DEVICE(kbd);

	gpiokeys_clear_state(kbd);

	KBD_PROBE_DONE(kbd);

	KBD_INIT_DONE(kbd);

	if (kbd_register(kbd) < 0) {
		goto detach;
	}

	KBD_CONFIG_DONE(kbd);

	gpiokeys_enable(kbd);

#ifdef KBD_INSTALL_CDEV
	if (kbd_attach(kbd)) {
		goto detach;
	}
#endif

	if (bootverbose) {
		genkbd_diag(kbd, 1);
	}

	gpiokeys_sc = sc;

	return (0);
detach:
	gpiokeys_detach(dev);
	return (ENXIO);
}

static int
gpiokeys_detach(device_t dev)
{
	struct gpiokeys_softc *sc;

	sc = device_get_softc(dev);
	GPIOKEYS_LOCK_DESTROY(sc);

	return (0);
}

static void
gpiokeys_put_key(struct gpiokeys_softc *sc, uint32_t key)
{
	if (sc->sc_inputs < GPIOKEYS_GLOBAL_IN_BUF_SIZE) {
		sc->sc_input[sc->sc_inputtail] = key;
		++(sc->sc_inputs);
		++(sc->sc_inputtail);
		if (sc->sc_inputtail >= GPIOKEYS_GLOBAL_IN_BUF_SIZE) {
			sc->sc_inputtail = 0;
		}
	} else {
		device_printf(sc->sc_dev, "input buffer is full\n");
	}
}



void
gpiokeys_key_event(uint16_t keycode, int pressed)
{
	uint32_t key;

	key = keycode;
	if (!pressed)
		key |= KEY_RELEASE;

	gpiokeys_put_key(gpiokeys_sc, key);
	gpiokeys_event_keyinput(gpiokeys_sc);
}




/* early keyboard probe, not supported */
static int
gpiokeys_configure(int flags)
{
	return (0);
}

/* detect a keyboard, not used */
static int
gpiokeys__probe(int unit, void *arg, int flags)
{
	return (ENXIO);
}

/* reset and initialize the device, not used */
static int
gpiokeys_init(int unit, keyboard_t **kbdp, void *arg, int flags)
{
	return (ENXIO);
}

/* test the interface to the device, not used */
static int
gpiokeys_test_if(keyboard_t *kbd)
{
	return (0);
}

/* finish using this keyboard, not used */
static int
gpiokeys_term(keyboard_t *kbd)
{
	return (ENXIO);
}

/* keyboard interrupt routine, not used */
static int
gpiokeys_intr(keyboard_t *kbd, void *arg)
{
	return (0);
}

/* lock the access to the keyboard, not used */
static int
gpiokeys_lock(keyboard_t *kbd, int lock)
{
	return (1);
}

/*
 * Enable the access to the device; until this function is called,
 * the client cannot read from the keyboard.
 */
static int
gpiokeys_enable(keyboard_t *kbd)
{
	struct gpiokeys_softc *sc;

	sc = kbd->kb_data;
	GPIOKEYS_LOCK(sc);
	KBD_ACTIVATE(kbd);
	GPIOKEYS_UNLOCK(sc);

	return (0);
}

/* disallow the access to the device */
static int
gpiokeys_disable(keyboard_t *kbd)
{
	struct gpiokeys_softc *sc;

	sc = kbd->kb_data;
	GPIOKEYS_LOCK(sc);
	KBD_DEACTIVATE(kbd);
	GPIOKEYS_UNLOCK(sc);

	return (0);
}

static void
gpiokeys_do_poll(struct gpiokeys_softc *sc, uint8_t wait)
{

	KASSERT((sc->sc_flags & GPIOKEYS_GLOBAL_FLAG_POLLING) != 0,
	    ("gpiokeys_do_poll called when not polling\n"));

	if (!kdb_active && !SCHEDULER_STOPPED()) {
		while (sc->sc_inputs == 0) {
			kern_yield(PRI_UNCHANGED);
			if (!wait)
				break;
		}
		return;
	}

	while ((sc->sc_inputs == 0) && wait) {
		printf("POLL!\n");
	}
}

/* check if data is waiting */
static int
gpiokeys_check(keyboard_t *kbd)
{
	struct gpiokeys_softc *sc = kbd->kb_data;


	if (!KBD_IS_ACTIVE(kbd))
		return (0);

	if (sc->sc_flags & GPIOKEYS_GLOBAL_FLAG_POLLING)
		gpiokeys_do_poll(sc, 0);

	if (sc->sc_inputs > 0) {
		return (1);
	}
	return (0);
}

/* check if char is waiting */
static int
gpiokeys_check_char_locked(keyboard_t *kbd)
{
	if (!KBD_IS_ACTIVE(kbd))
		return (0);

	return (gpiokeys_check(kbd));
}

static int
gpiokeys_check_char(keyboard_t *kbd)
{
	int result;
	struct gpiokeys_softc *sc = kbd->kb_data;

	GPIOKEYS_LOCK(sc);
	result = gpiokeys_check_char_locked(kbd);
	GPIOKEYS_UNLOCK(sc);

	return (result);
}

static int32_t
gpiokeys_get_key(struct gpiokeys_softc *sc, uint8_t wait)
{
	int32_t c;

	KASSERT((!kdb_active && !SCHEDULER_STOPPED())
	    || (sc->sc_flags & GPIOKEYS_GLOBAL_FLAG_POLLING) != 0,
	    ("not polling in kdb or panic\n"));

	if (sc->sc_flags & GPIOKEYS_GLOBAL_FLAG_POLLING)
		gpiokeys_do_poll(sc, wait);

	if (sc->sc_inputs == 0) {
		c = -1;
	} else {
		c = sc->sc_input[sc->sc_inputhead];
		--(sc->sc_inputs);
		++(sc->sc_inputhead);
		if (sc->sc_inputhead >= GPIOKEYS_GLOBAL_IN_BUF_SIZE) {
			sc->sc_inputhead = 0;
		}
	}
	return (c);
}

/* read one byte from the keyboard if it's allowed */
static int
gpiokeys_read(keyboard_t *kbd, int wait)
{
	struct gpiokeys_softc *sc = kbd->kb_data;
	int32_t usbcode;

	if (!KBD_IS_ACTIVE(kbd))
		return (-1);

	/* XXX */
	usbcode = gpiokeys_get_key(sc, (wait == FALSE) ? 0 : 1);
	if (!KBD_IS_ACTIVE(kbd) || (usbcode == -1))
		return (-1);

	++(kbd->kb_count);

	return (usbcode);
}

/* read char from the keyboard */
static uint32_t
gpiokeys_read_char_locked(keyboard_t *kbd, int wait)
{
	struct gpiokeys_softc *sc = kbd->kb_data;
	uint32_t action;
	uint32_t keycode;

	if (!KBD_IS_ACTIVE(kbd))
		return (NOKEY);

next_code:

	/* see if there is something in the keyboard port */
	/* XXX */
	keycode = gpiokeys_get_key(sc, (wait == FALSE) ? 0 : 1);
	++kbd->kb_count;

	/* return the byte as is for the K_RAW mode */
	if (sc->sc_mode == K_RAW) {
		return (keycode);
	}

	/* return the key code in the K_CODE mode */
	/* XXX: keycode |= SCAN_RELEASE; */

	if (sc->sc_mode == K_CODE) {
		return (keycode);
	}

	/* keycode to key action */
	action = genkbd_keyaction(kbd, SCAN_CHAR(keycode),
	    (keycode & SCAN_RELEASE),
	    &sc->sc_state, &sc->sc_accents);
	if (action == NOKEY) {
		goto next_code;
	}

	return (action);
}

/* Currently wait is always false. */
static uint32_t
gpiokeys_read_char(keyboard_t *kbd, int wait)
{
	uint32_t keycode;
	struct gpiokeys_softc *sc = kbd->kb_data;

	GPIOKEYS_LOCK(sc);
	keycode = gpiokeys_read_char_locked(kbd, wait);
	GPIOKEYS_UNLOCK(sc);

	return (keycode);
}

/* some useful control functions */
static int
gpiokeys_ioctl_locked(keyboard_t *kbd, u_long cmd, caddr_t arg)
{
	struct gpiokeys_softc *sc = kbd->kb_data;
#if defined(COMPAT_FREEBSD6) || defined(COMPAT_FREEBSD5) || \
    defined(COMPAT_FREEBSD4) || defined(COMPAT_43)
	int ival;

#endif

	switch (cmd) {
	case KDGKBMODE:		/* get keyboard mode */
		*(int *)arg = sc->sc_mode;
		break;
#if defined(COMPAT_FREEBSD6) || defined(COMPAT_FREEBSD5) || \
    defined(COMPAT_FREEBSD4) || defined(COMPAT_43)
	case _IO('K', 7):
		ival = IOCPARM_IVAL(arg);
		arg = (caddr_t)&ival;
		/* FALLTHROUGH */
#endif
	case KDSKBMODE:		/* set keyboard mode */
		switch (*(int *)arg) {
		case K_XLATE:
			if (sc->sc_mode != K_XLATE) {
				/* make lock key state and LED state match */
				sc->sc_state &= ~LOCK_MASK;
				sc->sc_state |= KBD_LED_VAL(kbd);
			}
			/* FALLTHROUGH */
		case K_RAW:
		case K_CODE:
			if (sc->sc_mode != *(int *)arg) {
				if ((sc->sc_flags & GPIOKEYS_GLOBAL_FLAG_POLLING) == 0)
					gpiokeys_clear_state(kbd);
				sc->sc_mode = *(int *)arg;
			}
			break;
		default:
			return (EINVAL);
		}
		break;

	case KDGETLED:			/* get keyboard LED */
		*(int *)arg = KBD_LED_VAL(kbd);
		break;
#if defined(COMPAT_FREEBSD6) || defined(COMPAT_FREEBSD5) || \
    defined(COMPAT_FREEBSD4) || defined(COMPAT_43)
	case _IO('K', 66):
		ival = IOCPARM_IVAL(arg);
		arg = (caddr_t)&ival;
		/* FALLTHROUGH */
#endif
	case KDSETLED:			/* set keyboard LED */
		KBD_LED_VAL(kbd) = *(int *)arg;
		break;
	case KDGKBSTATE:		/* get lock key state */
		*(int *)arg = sc->sc_state & LOCK_MASK;
		break;
#if defined(COMPAT_FREEBSD6) || defined(COMPAT_FREEBSD5) || \
    defined(COMPAT_FREEBSD4) || defined(COMPAT_43)
	case _IO('K', 20):
		ival = IOCPARM_IVAL(arg);
		arg = (caddr_t)&ival;
		/* FALLTHROUGH */
#endif
	case KDSKBSTATE:		/* set lock key state */
		if (*(int *)arg & ~LOCK_MASK) {
			return (EINVAL);
		}
		sc->sc_state &= ~LOCK_MASK;
		sc->sc_state |= *(int *)arg;
		return (0);

	case KDSETREPEAT:		/* set keyboard repeat rate (new
					 * interface) */
		if (!KBD_HAS_DEVICE(kbd)) {
			return (0);
		}
		if (((int *)arg)[1] < 0) {
			return (EINVAL);
		}
		if (((int *)arg)[0] < 0) {
			return (EINVAL);
		}
		if (((int *)arg)[0] < 200)	/* fastest possible value */
			kbd->kb_delay1 = 200;
		else
			kbd->kb_delay1 = ((int *)arg)[0];
		kbd->kb_delay2 = ((int *)arg)[1];
		return (0);

#if defined(COMPAT_FREEBSD6) || defined(COMPAT_FREEBSD5) || \
    defined(COMPAT_FREEBSD4) || defined(COMPAT_43)
	case _IO('K', 67):
		ival = IOCPARM_IVAL(arg);
		arg = (caddr_t)&ival;
		/* FALLTHROUGH */
#endif
	case KDSETRAD:			/* set keyboard repeat rate (old
					 * interface) */
		return (gpiokeys_set_typematic(kbd, *(int *)arg));

	case PIO_KEYMAP:		/* set keyboard translation table */
	case OPIO_KEYMAP:		/* set keyboard translation table
					 * (compat) */
	case PIO_KEYMAPENT:		/* set keyboard translation table
					 * entry */
	case PIO_DEADKEYMAP:		/* set accent key translation table */
		sc->sc_accents = 0;
		/* FALLTHROUGH */
	default:
		return (genkbd_commonioctl(kbd, cmd, arg));
	}

	return (0);
}

static int
gpiokeys_ioctl(keyboard_t *kbd, u_long cmd, caddr_t arg)
{
	int result;
	struct gpiokeys_softc *sc;

	sc = kbd->kb_data;
	/*
	 * XXX Check if someone is calling us from a critical section:
	 */
	if (curthread->td_critnest != 0)
		return (EDEADLK);

	GPIOKEYS_LOCK(sc);
	result = gpiokeys_ioctl_locked(kbd, cmd, arg);
	GPIOKEYS_UNLOCK(sc);

	return (result);
}


/* clear the internal state of the keyboard */
static void
gpiokeys_clear_state(keyboard_t *kbd)
{
	struct gpiokeys_softc *sc = kbd->kb_data;

	sc->sc_flags &= ~(GPIOKEYS_GLOBAL_FLAG_POLLING);
	sc->sc_state &= LOCK_MASK;	/* preserve locking key state */
	sc->sc_accents = 0;
}

/* get the internal state, not used */
static int
gpiokeys_get_state(keyboard_t *kbd, void *buf, size_t len)
{
	return (len == 0) ? 1 : -1;
}

/* set the internal state, not used */
static int
gpiokeys_set_state(keyboard_t *kbd, void *buf, size_t len)
{
	return (EINVAL);
}

static int
gpiokeys_poll(keyboard_t *kbd, int on)
{
	struct gpiokeys_softc *sc = kbd->kb_data;

	GPIOKEYS_LOCK(sc);
	if (on)
		sc->sc_flags |= GPIOKEYS_GLOBAL_FLAG_POLLING;
	else
		sc->sc_flags &= ~GPIOKEYS_GLOBAL_FLAG_POLLING;
	GPIOKEYS_UNLOCK(sc);

	return (0);
}

static int
gpiokeys_set_typematic(keyboard_t *kbd, int code)
{
	static const int delays[] = {250, 500, 750, 1000};
	static const int rates[] = {34, 38, 42, 46, 50, 55, 59, 63,
		68, 76, 84, 92, 100, 110, 118, 126,
		136, 152, 168, 184, 200, 220, 236, 252,
	272, 304, 336, 368, 400, 440, 472, 504};

	if (code & ~0x7f) {
		return (EINVAL);
	}
	kbd->kb_delay1 = delays[(code >> 5) & 3];
	kbd->kb_delay2 = rates[code & 0x1f];
	return (0);
}

static void
gpiokeys_event_keyinput(struct gpiokeys_softc *sc)
{
	int c;

	if ((sc->sc_flags & GPIOKEYS_GLOBAL_FLAG_POLLING) != 0)
		return;

	if (sc->sc_inputs == 0)
		return;

	if (KBD_IS_ACTIVE(&sc->sc_kbd) &&
	    KBD_IS_BUSY(&sc->sc_kbd)) {
		/* let the callback function process the input */
		(sc->sc_kbd.kb_callback.kc_func) (&sc->sc_kbd, KBDIO_KEYINPUT,
		    sc->sc_kbd.kb_callback.kc_arg);
	} else {
		/* read and discard the input, no one is waiting for it */
		do {
			c = gpiokeys_read_char(&sc->sc_kbd, 0);
		} while (c != NOKEY);
	}
}

static keyboard_switch_t gpiokeyssw = {
	.probe = &gpiokeys__probe,
	.init = &gpiokeys_init,
	.term = &gpiokeys_term,
	.intr = &gpiokeys_intr,
	.test_if = &gpiokeys_test_if,
	.enable = &gpiokeys_enable,
	.disable = &gpiokeys_disable,
	.read = &gpiokeys_read,
	.check = &gpiokeys_check,
	.read_char = &gpiokeys_read_char,
	.check_char = &gpiokeys_check_char,
	.ioctl = &gpiokeys_ioctl,
	.lock = &gpiokeys_lock,
	.clear_state = &gpiokeys_clear_state,
	.get_state = &gpiokeys_get_state,
	.set_state = &gpiokeys_set_state,
	.get_fkeystr = &genkbd_get_fkeystr,
	.poll = &gpiokeys_poll,
	.diag = &genkbd_diag,
};

KEYBOARD_DRIVER(gpiokeys, gpiokeyssw, gpiokeys_configure);

static int
gpiokeys_driver_load(module_t mod, int what, void *arg)
{
	switch (what) {
	case MOD_LOAD:
		kbd_add_driver(&gpiokeys_kbd_driver);
		break;
	case MOD_UNLOAD:
		kbd_delete_driver(&gpiokeys_kbd_driver);
		break;
	}
	return (0);
}

static devclass_t gpiokeys_devclass;

static device_method_t gpiokeys_methods[] = {
	DEVMETHOD(device_probe,		gpiokeys_probe),
	DEVMETHOD(device_attach,	gpiokeys_attach),
	DEVMETHOD(device_detach,	gpiokeys_detach),

	{ 0, 0 }
};

static driver_t gpiokeys_driver = {
	"gpiokeys",
	gpiokeys_methods,
	sizeof(struct gpiokeys_softc),
};

DRIVER_MODULE(gpiokeys, simplebus, gpiokeys_driver, gpiokeys_devclass, gpiokeys_driver_load, 0);
MODULE_VERSION(gpiokeys, 1);
