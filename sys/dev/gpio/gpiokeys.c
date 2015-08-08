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
__FBSDID("$FreeBSD: head/sys/dev/gpio/gpiokey.c 283360 2015-05-24 07:45:42Z ganbold $");

#include "opt_platform.h"

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

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>

#include <dev/gpio/gpiobusvar.h>

#include "gpiobus_if.h"

/*
 * Only one pin for led
 */
#define	GPIOKEYS_PIN	0

#define GPIOKEYS_LOCK(_sc)		mtx_lock(&(_sc)->sc_mtx)
#define	GPIOKEYS_UNLOCK(_sc)		mtx_unlock(&(_sc)->sc_mtx)
#define GPIOKEYS_LOCK_INIT(_sc) \
	mtx_init(&_sc->sc_mtx, device_get_nameunit(_sc->sc_dev), \
	    "gpiokey", MTX_DEF)
#define GPIOKEYS_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->sc_mtx);

#define	GPIOKEYS_GLOBAL_LOCK()	mtx_lock(&Giant)
#define	GPIOKEYS_GLOBAL_UNLOCK()	mtx_unlock(&Giant)

#ifdef	INVARIANTS

/*
 * Assert that the lock is held in all contexts
 * where the code can be executed.
 */
#define	GPIOKEYS_GLOBAL_LOCK_ASSERT()	mtx_assert(&Giant, MA_OWNED)

/*
 * Assert that the lock is held in the contexts
 * where it really has to be so.
 */
#define	GPIOKEYS_GLOBAL_CTX_LOCK_ASSERT()			 	\
	do {						\
		if (!kdb_active && panicstr == NULL)	\
			mtx_assert(&Giant, MA_OWNED);	\
	} while (0)
#else

#define GPIOKEYS_GLOBAL_LOCK_ASSERT()	(void)0
#define GPIOKEYS_GLOBAL_CTX_LOCK_ASSERT()	(void)0

#endif

#define	KEY_ERROR	  0x01

#define	KEY_PRESS	  0
#define	KEY_RELEASE	  0x400
#define	KEY_INDEX(c)	  ((c) & 0xFF)

#define	SCAN_PRESS	  0
#define	SCAN_RELEASE	  0x80
#define	SCAN_PREFIX_E0	  0x100
#define	SCAN_PREFIX_E1	  0x200
#define	SCAN_PREFIX_CTL	  0x400
#define	SCAN_PREFIX_SHIFT 0x800
#define	SCAN_PREFIX	(SCAN_PREFIX_E0  | SCAN_PREFIX_E1 | \
			 SCAN_PREFIX_CTL | SCAN_PREFIX_SHIFT)
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
	struct thread	*sc_poll_thread;

	uint32_t	sc_ntime[GPIOKEYS_GLOBAL_NKEYCODE];
	uint32_t	sc_otime[GPIOKEYS_GLOBAL_NKEYCODE];
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

	uint8_t		sc_buffer[GPIOKEYS_GLOBAL_BUFFER_SIZE];
};

struct gpiokey_softc 
{
	device_t	sc_dev;
	device_t	sc_busdev;
	int		sc_irq_rid;
	struct resource	*sc_irq_res;
	void		*sc_intr_hl;
	struct mtx	sc_mtx;
};

/* Single key device */
static int gpiokey_probe(device_t);
static int gpiokey_attach(device_t);
static int gpiokey_detach(device_t);

/* gpio-keys device */
static int gpiokeys_probe(device_t);
static int gpiokeys_attach(device_t);
static int gpiokeys_detach(device_t);

/* kbd methods prototypes */
static void	gpiokeys_timeout(void *);
static void	gpiokeys_set_leds(struct gpiokeys_softc *, uint8_t);
static int	gpiokeys_set_typematic(keyboard_t *, int);
static uint32_t	gpiokeys_read_char(keyboard_t *, int);
static void	gpiokeys_clear_state(keyboard_t *);
static int	gpiokeys_ioctl(keyboard_t *, u_long, caddr_t);
static int	gpiokeys_enable(keyboard_t *);
static int	gpiokeys_disable(keyboard_t *);
static void	gpiokeys_interrupt(struct gpiokeys_softc *);
static void	gpiokeys_event_keyinput(struct gpiokeys_softc *);

static void
gpiokey_intr(void *arg)
{
	struct gpiokey_softc *sc;

	sc = arg;
	device_printf(sc->sc_dev, "Intr!\n");
}

static void
gpiokey_identify(driver_t *driver, device_t bus)
{
	phandle_t child, leds, root;

	root = OF_finddevice("/");
	if (root == 0)
		return;
	for (leds = OF_child(root); leds != 0; leds = OF_peer(leds)) {
		if (!fdt_is_compatible_strict(leds, "gpio-keys"))
			continue;
		/* Traverse the 'gpio-leds' node and add its children. */
		for (child = OF_child(leds); child != 0; child = OF_peer(child)) {
			if (!OF_hasprop(child, "gpios"))
				continue;
			if (ofw_gpiobus_add_fdt_child(bus, driver->name, child) == NULL)
				continue;
		}
	}
}

static int
gpiokey_probe(device_t dev)
{
	phandle_t node;
	char *name;

	if ((node = ofw_bus_get_node(dev)) == -1)
		return (ENXIO);

	name = NULL;
	if (OF_getprop_alloc(node, "label", 1, (void **)&name) == -1)
		OF_getprop_alloc(node, "name", 1, (void **)&name);

	if (name != NULL) {
		device_set_desc_copy(dev, name);
		free(name, M_OFWPROP);
	}

	return (0);
}

static int
gpiokey_attach(device_t dev)
{
	struct gpiokey_softc *sc;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	sc->sc_busdev = device_get_parent(dev);

	sc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->sc_irq_rid,
	    RF_ACTIVE);
	if (!sc->sc_irq_res) {
		device_printf(dev, "cannot allocate interrupt\n");
		return (ENXIO);
	}

	if (bus_setup_intr(dev, sc->sc_irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
			NULL, gpiokey_intr, sc,
			&sc->sc_intr_hl) != 0) {
		bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid,
		    sc->sc_irq_res);
		device_printf(dev, "Unable to setup the irq handler.\n");
		return (ENXIO);
	}

	GPIOKEYS_LOCK_INIT(sc);

	return (0);
}

static int
gpiokey_detach(device_t dev)
{
	struct gpiokey_softc *sc;

	sc = device_get_softc(dev);
	GPIOKEYS_LOCK_DESTROY(sc);

	return (0);
}

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
	struct gpiokeys_softc *sc;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	GPIOKEYS_LOCK_INIT(sc);

	return (0);
}

static int
gpiokeys_detach(device_t dev)
{
	struct gpiokeys_softc *sc;

	sc = device_get_softc(dev);
	GPIOKEYS_LOCK_DESTROY(sc);

	return (0);
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

	GPIOKEYS_GLOBAL_LOCK();
	KBD_ACTIVATE(kbd);
	GPIOKEYS_GLOBAL_UNLOCK();

	return (0);
}

/* disallow the access to the device */
static int
gpiokeys_disable(keyboard_t *kbd)
{

	GPIOKEYS_GLOBAL_LOCK();
	KBD_DEACTIVATE(kbd);
	GPIOKEYS_GLOBAL_UNLOCK();

	return (0);
}

static void
gpiokeys_do_poll(struct gpiokeys_softc *sc, uint8_t wait)
{

	GPIOKEYS_GLOBAL_CTX_LOCK_ASSERT();
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
}

/* check if data is waiting */
static int
gpiokeys_check(keyboard_t *kbd)
{
	struct gpiokeys_softc *sc = kbd->kb_data;

	GPIOKEYS_GLOBAL_CTX_LOCK_ASSERT();

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
	GPIOKEYS_GLOBAL_CTX_LOCK_ASSERT();

	if (!KBD_IS_ACTIVE(kbd))
		return (0);

	return (gpiokeys_check(kbd));
}

static int
gpiokeys_check_char(keyboard_t *kbd)
{
	int result;

	GPIOKEYS_GLOBAL_LOCK();
	result = gpiokeys_check_char_locked(kbd);
	GPIOKEYS_GLOBAL_UNLOCK();

	return (result);
}

static int32_t
gpiokeys_get_key(struct gpiokeys_softc *sc, uint8_t wait)
{
	int32_t c;

	GPIOKEYS_GLOBAL_CTX_LOCK_ASSERT();
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

	GPIOKEYS_GLOBAL_CTX_LOCK_ASSERT();

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

	GPIOKEYS_GLOBAL_CTX_LOCK_ASSERT();

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

	GPIOKEYS_GLOBAL_LOCK();
	keycode = gpiokeys_read_char_locked(kbd, wait);
	GPIOKEYS_GLOBAL_UNLOCK();

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

	GPIOKEYS_GLOBAL_LOCK_ASSERT();

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

		/* set LEDs and quit */
		return (gpiokeys_ioctl(kbd, KDSETLED, arg));

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

	/*
	 * XXX Check if someone is calling us from a critical section:
	 */
	if (curthread->td_critnest != 0)
		return (EDEADLK);

	/*
	 * XXX KDGKBSTATE, KDSKBSTATE and KDSETLED can be called from any
	 * context where printf(9) can be called, which among other things
	 * includes interrupt filters and threads with any kinds of locks
	 * already held.  For this reason it would be dangerous to acquire
	 * the Giant here unconditionally.  On the other hand we have to
	 * have it to handle the ioctl.
	 * So we make our best effort to auto-detect whether we can grab
	 * the Giant or not.  Blame syscons(4) for this.
	 */
	switch (cmd) {
	case KDGKBSTATE:
	case KDSKBSTATE:
	case KDSETLED:
		if (!mtx_owned(&Giant) && !SCHEDULER_STOPPED())
			return (EDEADLK);	/* best I could come up with */
		/* FALLTHROUGH */
	default:
		GPIOKEYS_GLOBAL_LOCK();
		result = gpiokeys_ioctl_locked(kbd, cmd, arg);
		GPIOKEYS_GLOBAL_UNLOCK();
		return (result);
	}
}


/* clear the internal state of the keyboard */
static void
gpiokeys_clear_state(keyboard_t *kbd)
{
	struct gpiokeys_softc *sc = kbd->kb_data;

	GPIOKEYS_GLOBAL_CTX_LOCK_ASSERT();

	sc->sc_flags &= ~(GPIOKEYS_GLOBAL_FLAG_POLLING);
	sc->sc_state &= LOCK_MASK;	/* preserve locking key state */
	sc->sc_accents = 0;
	memset(&sc->sc_ntime, 0, sizeof(sc->sc_ntime));
	memset(&sc->sc_otime, 0, sizeof(sc->sc_otime));
}

/* save the internal state, not used */
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

	GPIOKEYS_GLOBAL_LOCK();
	if (on) {
		sc->sc_flags |= GPIOKEYS_GLOBAL_FLAG_POLLING;
		sc->sc_poll_thread = curthread;
	} else {
		sc->sc_flags &= ~GPIOKEYS_GLOBAL_FLAG_POLLING;
		/* XXX: gpiokeys_start_timer(sc); */	/* start timer */ 
	}
	GPIOKEYS_GLOBAL_UNLOCK();

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

static devclass_t gpiokey_devclass;

static device_method_t gpiokey_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify,	gpiokey_identify),

	DEVMETHOD(device_probe,		gpiokey_probe),
	DEVMETHOD(device_attach,	gpiokey_attach),
	DEVMETHOD(device_detach,	gpiokey_detach),

	{ 0, 0 }
};

static driver_t gpiokey_driver = {
	"gpiokey",
	gpiokey_methods,
	sizeof(struct gpiokey_softc),
};

DRIVER_MODULE(gpiokey, gpiobus, gpiokey_driver, gpiokey_devclass, 0, 0);

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

DRIVER_MODULE(gpiokeys, simplebus, gpiokeys_driver, gpiokeys_devclass, 0, 0);
