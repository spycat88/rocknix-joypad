// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ROCKNIX joypad driver
 *
 * Copyright (C) 2024 ROCKNIX (https://github.com/ROCKNIX)
 *
 * One driver for two hardware layouts, picked by the compatible string:
 *
 *  "rocknix-joypad"
 *	Every analog axis has its own IIO channel, named in
 *	"io-channel-names" and read as a processed value.
 *
 *  "rocknix-singleadc-joypad"
 *	One IIO channel ("amux_adc") sits behind a 4:1 analog mux driven by
 *	the "amux-a"/"amux-b" (and optional "amux-en") GPIOs, read raw.
 *	This layout can also drive a PWM rumble motor and can take the
 *	sticks from the Miyoo serial controller on /dev/ttyS1 instead.
 *
 * In both layouts the buttons are the GPIOs of the child nodes, and
 * everything is polled at "poll-interval" ms while the device is open.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/consumer.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kstrtox.h>
#include <linux/kthread.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/sysfs.h>
#include <linux/termios.h>
#include <linux/tty.h>
#include <linux/version.h>
#include <linux/workqueue.h>

/*----------------------------------------------------------------------------*/
/* Analog sticks */

/* Full ADC range in mV; axes report +/- half of it, times the scale. */
#define ADC_MAX_VOLTAGE		1800
/* Tuning, in percent, used when an axis has no abs_<axis>-[pn]-tuning. */
#define ADC_TUNING_DEFAULT	180

/* Miyoo serial sticks (rocknix,use-miyoo-serial-joypad) */
#define MIYOO_TTY_PATH			"/dev/ttyS1"
#define MIYOO_FRAME_SIZE		6
#define MIYOO_MAGIC_START		0xFF
#define MIYOO_MAGIC_END			0xFE
#define MIYOO_AXIS_MIN			(-32760)
#define MIYOO_AXIS_MAX			32760
#define MIYOO_AXIS_FUZZ			16
#define MIYOO_AXIS_FLAT			16
/* Raw range of each stick before auto-calibration replaces the centre. */
#define MIYOO_RAW_MIN			85
#define MIYOO_RAW_MAX			200
#define MIYOO_RAW_ZERO			130
/* Frames averaged to find the centre of each stick. */
#define MIYOO_CALIBRATION_FRAMES	50
/* Radial deadzone: 10% of MIYOO_AXIS_MAX, truncated (0.10f * 32760). */
#define MIYOO_DEADZONE			3276
/* Re-open the tty when no valid frame has arrived for this long. */
#define MIYOO_INACTIVITY_TIMEOUT	(5 * HZ)
/* Wait after probe before opening the tty, so boot is never blocked. */
#define MIYOO_START_DELAY		(10 * HZ)

/*
 * Exported by the ROCKNIX adc-keys kernel patch, which sends keycode 316
 * (BTN_MODE) from adc-keys to the joypad. The Makefile enables this only
 * for the device kernels that carry that patch.
 */
#ifdef ROCKNIX_JOYPAD_ADC_KEYS_HOOK
extern struct input_dev *joypad_input_g;
#endif

/*----------------------------------------------------------------------------*/
enum joypad_layout {
	JOYPAD_LAYOUT_MULTI_ADC,	/* "rocknix-joypad" */
	JOYPAD_LAYOUT_SINGLE_ADC,	/* "rocknix-singleadc-joypad" */
};

struct joypad_variant {
	enum joypad_layout layout;
	/* default input name, overridden by "joypad-name" */
	const char *name;
	const char *phys;
	/* default bus type, overridden by "joypad-bustype" */
	u16 bustype;
};

static const struct joypad_variant joypad_variant_multi_adc = {
	.layout		= JOYPAD_LAYOUT_MULTI_ADC,
	.name		= "rocknix-joypad",
	.phys		= "rocknix-joypad/input0",
	.bustype	= BUS_USB,
};

static const struct joypad_variant joypad_variant_single_adc = {
	.layout		= JOYPAD_LAYOUT_SINGLE_ADC,
	.name		= "rocknix-singleadc-joypad",
	.phys		= "rocknix-singleadc-joypad/input0",
	.bustype	= BUS_HOST,
};

struct joypad_axis {
	/* axis name used in property names, e.g. "x" or "ry" */
	const char *name;
	/* ABS_* code */
	int code;
	/* IIO channel to read */
	struct iio_channel *chan;
	/* mux input to select first (single ADC layout only) */
	int mux_sel;
	/* reported range, (ADC_MAX_VOLTAGE / 2) * scale */
	int min, max;
	/* reading taken when the device was opened */
	int cal;
	/* multiplier for each reading */
	int scale;
	/* report the negated value */
	bool invert;
	/* percent applied to positive / negative values */
	int tuning_p, tuning_n;
};

/* An axis name and its ABS_* code, in the order the channels appear. */
struct joypad_axis_map {
	const char *name;
	int code;
};

struct joypad_button {
	const char *label;
	struct gpio_desc *desc;
	int type;
	int code;
	bool old_value;
};

struct joypad_miyoo_cal {
	int min, max, zero;
};

struct joypad_miyoo {
	/* [0] = x, [1] = y */
	struct joypad_miyoo_cal left[2];
	struct joypad_miyoo_cal right[2];

	/* last values parsed by the reader thread */
	int left_x, left_y, right_x, right_y;

	struct file *filp;
	struct task_struct *thread;
	struct delayed_work start_work;
	unsigned long last_data_jiffies;
};

struct joypad {
	struct device *dev;
	const struct joypad_variant *variant;
	struct input_dev *input;

	/* reporting is on while the input device is open */
	bool enable;
	struct mutex lock;

	/* device tree settings */
	int poll_interval;
	bool auto_repeat;
	int adc_fuzz, adc_flat;
	int adc_scale;
	int adc_deadzone;

	/* analog axes, in pairs: [0] and [1] are one stick, and so on */
	int axis_count;
	struct joypad_axis *axes;

	int button_count;
	struct joypad_button *buttons;

	/* single ADC layout: analog mux select and enable lines */
	struct gpio_desc *mux_a, *mux_b, *mux_en;

	/* single ADC layout: PWM rumble */
	bool has_rumble;
	bool rumble_enabled;
	struct pwm_device *pwm;
	struct work_struct play_work;
	u16 level;
	u16 boost_weak, boost_strong;

	/* single ADC layout: Miyoo serial sticks */
	bool use_miyoo_serial;
	struct joypad_miyoo miyoo;
};

static inline bool joypad_is_single_adc(const struct joypad *joypad)
{
	return joypad->variant->layout == JOYPAD_LAYOUT_SINGLE_ADC;
}

/*----------------------------------------------------------------------------*/
/* Rumble */

static int joypad_pwm_apply(struct pwm_device *pwm, const struct pwm_state *state)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
	return pwm_apply_state(pwm, state);
#else
	return pwm_apply_might_sleep(pwm, state);
#endif
}

static int joypad_rumble_start(struct joypad *joypad, u16 level)
{
	struct pwm_state state;
	int error;

	pwm_get_state(joypad->pwm, &state);
	pwm_set_relative_duty_cycle(&state, level, 0xffff);
	state.enabled = true;

	error = joypad_pwm_apply(joypad->pwm, &state);
	if (error) {
		dev_err(joypad->dev, "failed to apply pwm state: %d", error);
		return error;
	}

	return 0;
}

static void joypad_rumble_stop(struct joypad *joypad)
{
	pwm_disable(joypad->pwm);
}

static void joypad_rumble_work(struct work_struct *work)
{
	struct joypad *joypad = container_of(work, struct joypad, play_work);
	u16 level;

	mutex_lock(&joypad->lock);

	if (!READ_ONCE(joypad->rumble_enabled)) {
		joypad_rumble_stop(joypad);
		goto out;
	}

	/* Read the level once, so the check and the duty cycle agree. */
	level = READ_ONCE(joypad->level);
	if (level)
		joypad_rumble_start(joypad, level);
	else
		joypad_rumble_stop(joypad);
out:
	mutex_unlock(&joypad->lock);
}

/*
 * ff-memless calls this in atomic context (dev->event_lock held with
 * interrupts off, and from its effect timer), so it must not sleep and must
 * not take joypad->lock: the work item holds that mutex while it applies
 * the PWM state, and waiting on it here hard-locked the device with the
 * motor on. Only store the level; the work item applies it.
 */
static int joypad_rumble_play(struct input_dev *input, void *data,
			      struct ff_effect *effect)
{
	struct joypad *joypad = data;
	u32 level;

	if (effect->type != FF_RUMBLE)
		return 0;

	if (!READ_ONCE(joypad->rumble_enabled))
		return 0;

	if (effect->u.rumble.strong_magnitude)
		level = effect->u.rumble.strong_magnitude + joypad->boost_strong;
	else
		level = effect->u.rumble.weak_magnitude + joypad->boost_weak;

	WRITE_ONCE(joypad->level, (u16)min_t(u32, level, 0xffff));

	schedule_work(&joypad->play_work);
	return 0;
}

/* /sys/devices/platform/<joypad node>/rumble_enable [rw] */
static ssize_t rumble_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct joypad *joypad = dev_get_drvdata(dev);
	bool enable = simple_strtoul(buf, NULL, 10);
	bool stop = false;

	mutex_lock(&joypad->lock);
	if (enable && !joypad->rumble_enabled) {
		WRITE_ONCE(joypad->rumble_enabled, true);
	} else if (!enable && joypad->rumble_enabled) {
		WRITE_ONCE(joypad->rumble_enabled, false);
		WRITE_ONCE(joypad->level, 0);
		stop = true;
	}
	mutex_unlock(&joypad->lock);

	/*
	 * cancel_work_sync() must run without the lock: the work item takes
	 * it too, so waiting for the work while holding it would never end.
	 */
	if (stop) {
		cancel_work_sync(&joypad->play_work);
		mutex_lock(&joypad->lock);
		joypad_rumble_stop(joypad);
		mutex_unlock(&joypad->lock);
	}

	return count;
}

static ssize_t rumble_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct joypad *joypad = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", joypad->rumble_enabled ? 1 : 0);
}

static DEVICE_ATTR_RW(rumble_enable);

static struct attribute *joypad_rumble_attrs[] = {
	&dev_attr_rumble_enable.attr,
	NULL,
};

static const struct attribute_group joypad_rumble_attr_group = {
	.attrs = joypad_rumble_attrs,
};

static void joypad_remove_rumble_attrs(void *data)
{
	struct device *dev = data;

	sysfs_remove_group(&dev->kobj, &joypad_rumble_attr_group);
}

static int joypad_rumble_setup(struct device *dev, struct joypad *joypad)
{
	struct pwm_state state;
	int error;

	joypad->pwm = devm_pwm_get(dev, "enable");
	if (IS_ERR(joypad->pwm)) {
		dev_err(dev, "rumble get error\n");
		return -EINVAL;
	}

	/* Sync up the PWM state and make sure the motor is off. */
	pwm_init_state(joypad->pwm, &state);
	state.enabled = false;

	error = joypad_pwm_apply(joypad->pwm, &state);
	if (error) {
		dev_err(dev, "failed to apply initial PWM state: %d", error);
		return error;
	}

	dev_info(dev, "rumble setup success!\n");
	return 0;
}

/*----------------------------------------------------------------------------*/
/* Analog sticks */

/* Single ADC layout: route mux input @sel to the ADC. */
static int joypad_mux_select(struct joypad *joypad, int sel)
{
	int a, b;

	if (joypad->mux_en)
		gpiod_set_value_cansleep(joypad->mux_en, 1);

	switch (sel) {
	case 0:
		a = 1;
		b = 1;
		break;
	case 1:
		a = 1;
		b = 0;
		break;
	case 2:
		a = 0;
		b = 1;
		break;
	case 3:
		a = 0;
		b = 0;
		break;
	default:
		if (joypad->mux_en)
			gpiod_set_value_cansleep(joypad->mux_en, 0);
		return -EINVAL;
	}

	gpiod_set_value_cansleep(joypad->mux_a, a);
	gpiod_set_value_cansleep(joypad->mux_b, b);

	/* mux switching speed: 35ns (on) / 9ns (off) */
	usleep_range(10, 20);
	return 0;
}

/* Returns the scaled reading, or 0 when the axis could not be read. */
static int joypad_axis_read(struct joypad *joypad, struct joypad_axis *axis)
{
	int value;

	if (joypad_is_single_adc(joypad)) {
		if (joypad_mux_select(joypad, axis->mux_sel))
			return 0;
		if (iio_read_channel_raw(axis->chan, &value) < 0)
			return 0;
	} else {
		if (iio_read_channel_processed(axis->chan, &value) < 0)
			return 0;
	}

	return value * axis->scale;
}

static void joypad_axis_read_failed(struct joypad *joypad, int index)
{
	/* The single ADC layout has always been quiet about this. */
	if (!joypad_is_single_adc(joypad))
		dev_err(joypad->dev, "saradc channels[%d]!\n", index);
}

static int joypad_axis_tune(const struct joypad_axis *axis, int value)
{
	if (axis->tuning_n && value < 0)
		value = (value * axis->tuning_n) / 100;
	if (axis->tuning_p && value > 0)
		value = (value * axis->tuning_p) / 100;

	if (value > axis->max)
		value = axis->max;
	if (value < axis->min)
		value = axis->min;

	return axis->invert ? -value : value;
}

static void joypad_report_sticks(struct joypad *joypad)
{
	struct input_dev *input = joypad->input;
	int deadzone = joypad->adc_deadzone;
	int i;

	/* Axes come in pairs: X then Y of the same stick. */
	for (i = 0; i + 1 < joypad->axis_count; i += 2) {
		struct joypad_axis *ax = &joypad->axes[i];
		struct joypad_axis *ay = &joypad->axes[i + 1];
		int x, y, mag;

		x = joypad_axis_read(joypad, ax);
		if (!x) {
			joypad_axis_read_failed(joypad, i);
			continue;
		}
		x -= ax->cal;

		y = joypad_axis_read(joypad, ay);
		if (!y) {
			joypad_axis_read_failed(joypad, i + 1);
			continue;
		}
		y -= ay->cal;

		/*
		 * Scaled radial deadzone, see
		 * https://web.archive.org/web/20190129113357/http://www.third-helix.com/2013/04/12/doing-thumbstick-dead-zones-right.html
		 */
		mag = int_sqrt((x * x) + (y * y));
		if (deadzone) {
			if (mag <= deadzone) {
				x = 0;
				y = 0;
			} else {
				/*
				 * Assumes max == -min for both axes. The order
				 * of operations avoids integer overflow.
				 */
				x = (((ax->max * x) / mag) * (mag - deadzone)) /
				    (ax->max - deadzone);
				y = (((ay->max * y) / mag) * (mag - deadzone)) /
				    (ay->max - deadzone);
			}
		}

		input_report_abs(input, ax->code, joypad_axis_tune(ax, x));
		input_report_abs(input, ay->code, joypad_axis_tune(ay, y));
	}
	input_sync(input);
}

static void joypad_calibrate_sticks(struct joypad *joypad)
{
	int i;

	for (i = 0; i < joypad->axis_count; i++) {
		struct joypad_axis *axis = &joypad->axes[i];
		int value = joypad_axis_read(joypad, axis);

		if (!value) {
			dev_err(joypad->dev, "saradc channels[%d]!\n", i);
			continue;
		}
		axis->cal = value;

		if (!joypad_is_single_adc(joypad))
			dev_info(joypad->dev, "adc[%d] adc->cal = %d\n", i,
				 axis->cal);
	}
}

/*----------------------------------------------------------------------------*/
/* Buttons */

static void joypad_report_buttons(struct joypad *joypad)
{
	struct input_dev *input = joypad->input;
	bool single_adc = joypad_is_single_adc(joypad);
	int i;

	for (i = 0; i < joypad->button_count; i++) {
		struct joypad_button *btn = &joypad->buttons[i];
		int value = gpiod_get_value_cansleep(btn->desc);

		if (value != btn->old_value) {
			input_event(input, btn->type, btn->code,
				    single_adc ? value == 1 : value);
			btn->old_value = value;
		}
	}
	input_sync(input);
}

/*----------------------------------------------------------------------------*/
/* Miyoo serial sticks */

static int joypad_miyoo_scale(const struct joypad_miyoo_cal *cal, int raw)
{
	int range_pos = cal->max - cal->zero;
	int range_neg = cal->zero - cal->min;
	int result = 0;

	if (raw > cal->zero) {
		if (range_pos != 0) {
			result = ((raw - cal->zero) * MIYOO_AXIS_MAX) / range_pos;
			if (result > MIYOO_AXIS_MAX)
				result = MIYOO_AXIS_MAX;
		}
	} else if (raw < cal->zero) {
		if (range_neg != 0) {
			result = -((cal->zero - raw) * (-MIYOO_AXIS_MIN)) /
				 range_neg;
			if (result < MIYOO_AXIS_MIN)
				result = MIYOO_AXIS_MIN;
		}
	}

	return result;
}

static void joypad_miyoo_deadzone(int *x, int *y)
{
	long long mag_sq = (long long)*x * *x + (long long)*y * *y;
	long long dead_sq = (long long)MIYOO_DEADZONE * MIYOO_DEADZONE;

	if (mag_sq <= dead_sq) {
		*x = 0;
		*y = 0;
	}
}

/* Frame layout: FF, left Y, left X, right Y, right X, FE */
static void joypad_miyoo_parse(struct joypad *joypad, const u8 *frame)
{
	struct joypad_miyoo *miyoo = &joypad->miyoo;
	int lx, ly, rx, ry;

	if (frame[0] != MIYOO_MAGIC_START || frame[5] != MIYOO_MAGIC_END)
		return;

	/* A valid frame resets the inactivity timer. */
	miyoo->last_data_jiffies = jiffies;

	lx = joypad_miyoo_scale(&miyoo->left[0], frame[2]);
	ly = joypad_miyoo_scale(&miyoo->left[1], frame[1]);
	rx = joypad_miyoo_scale(&miyoo->right[0], frame[4]);
	ry = joypad_miyoo_scale(&miyoo->right[1], frame[3]);

	joypad_miyoo_deadzone(&lx, &ly);
	joypad_miyoo_deadzone(&rx, &ry);

	WRITE_ONCE(miyoo->left_x, lx);
	WRITE_ONCE(miyoo->left_y, ly);
	WRITE_ONCE(miyoo->right_x, rx);
	WRITE_ONCE(miyoo->right_y, ry);
}

static ssize_t joypad_miyoo_read_frame(struct joypad *joypad, u8 *frame)
{
	struct file *filp = joypad->miyoo.filp;

	return kernel_read(filp, frame, MIYOO_FRAME_SIZE, &filp->f_pos);
}

/* Set 9600 8N1, raw, directly on the tty instead of through an ioctl. */
static void joypad_miyoo_set_termios(struct file *filp)
{
	struct tty_file_private *priv = filp->private_data;
	struct ktermios termios;
	struct tty_struct *tty;

	if (!priv || !priv->tty)
		return;
	tty = priv->tty;

	termios = tty->termios;
	termios.c_iflag = 0;
	termios.c_oflag = 0;
	termios.c_cflag = B9600 | CS8 | CREAD | CLOCAL;
	termios.c_lflag = 0;
	memset(termios.c_cc, 0, NCCS);
	termios.c_cc[VMIN] = 1;
	termios.c_cc[VTIME] = 0;

	tty_set_termios(tty, &termios);
}

static int joypad_miyoo_open(struct joypad *joypad)
{
	struct file *filp;

	filp = filp_open(MIYOO_TTY_PATH, O_RDWR | O_NOCTTY, 0);
	if (IS_ERR(filp)) {
		dev_err(joypad->dev, "ERROR: open " MIYOO_TTY_PATH " failed: %ld\n",
			PTR_ERR(filp));
		return PTR_ERR(filp);
	}
	joypad->miyoo.filp = filp;

	joypad_miyoo_set_termios(filp);

	dev_info(joypad->dev,
		 "Opened serial port " MIYOO_TTY_PATH " and set 9600 8N1 (in-kernel).\n");
	return 0;
}

static void joypad_miyoo_close(struct joypad *joypad)
{
	if (!joypad->miyoo.filp)
		return;

	dev_info(joypad->dev, "Closing " MIYOO_TTY_PATH ".\n");
	filp_close(joypad->miyoo.filp, NULL);
	joypad->miyoo.filp = NULL;
}

/*
 * Re-open the tty after a quiet period. The sticks are not calibrated
 * again. Whether or not the open works, the inactivity timer restarts, so
 * a failed open is retried after another quiet period.
 */
static void joypad_miyoo_reopen(struct joypad *joypad)
{
	joypad_miyoo_close(joypad);

	if (joypad_miyoo_open(joypad) == 0)
		dev_info(joypad->dev, "TTY re-init success!\n");
	else
		dev_err(joypad->dev, "Failed to re-open " MIYOO_TTY_PATH "; will retry.\n");

	joypad->miyoo.last_data_jiffies = jiffies;
}

static bool joypad_miyoo_inactive(struct joypad *joypad)
{
	return time_after(jiffies, joypad->miyoo.last_data_jiffies +
				   MIYOO_INACTIVITY_TIMEOUT);
}

/* Average the first frames to find the centre of each stick. */
static int joypad_miyoo_autocal(struct joypad *joypad)
{
	struct joypad_miyoo *miyoo = &joypad->miyoo;
	long sum_yl = 0, sum_xl = 0, sum_yr = 0, sum_xr = 0;
	u8 frame[MIYOO_FRAME_SIZE];
	int frames = 0;

	dev_info(joypad->dev, "Auto-calibration: reading %d frames...",
		 MIYOO_CALIBRATION_FRAMES);

	while (frames < MIYOO_CALIBRATION_FRAMES) {
		ssize_t n = joypad_miyoo_read_frame(joypad, frame);

		if (n < 0) {
			dev_err(joypad->dev, "Auto-cal read error: %zd\n", n);
			msleep(10);
			continue;
		}
		if (n < MIYOO_FRAME_SIZE) {
			msleep(10);
			continue;
		}
		if (frame[0] == MIYOO_MAGIC_START && frame[5] == MIYOO_MAGIC_END) {
			sum_yl += frame[1];
			sum_xl += frame[2];
			sum_yr += frame[3];
			sum_xr += frame[4];
			frames++;
		}
	}

	miyoo->left[0].zero = sum_xl / frames;
	miyoo->left[1].zero = sum_yl / frames;
	miyoo->right[0].zero = sum_xr / frames;
	miyoo->right[1].zero = sum_yr / frames;

	dev_info(joypad->dev, "  Average YL=%d, XL=%d, YR=%d, XR=%d\n",
		 miyoo->left[1].zero, miyoo->left[0].zero,
		 miyoo->right[1].zero, miyoo->right[0].zero);
	dev_info(joypad->dev, "Auto-calibration complete.\n");
	return 0;
}

static int joypad_miyoo_thread(void *data)
{
	struct joypad *joypad = data;
	u8 frame[MIYOO_FRAME_SIZE];

	dev_info(joypad->dev, "Starting Miyoo serial read loop.\n");

	while (!kthread_should_stop()) {
		ssize_t n;

		/* The last re-open failed: wait out the timer and try again. */
		if (!joypad->miyoo.filp) {
			msleep(10);
			if (joypad_miyoo_inactive(joypad))
				joypad_miyoo_reopen(joypad);
			continue;
		}

		n = joypad_miyoo_read_frame(joypad, frame);
		if (n < 0) {
			dev_err(joypad->dev, "Serial read error: %zd\n", n);
			msleep(10);
			continue;
		}
		if (n < MIYOO_FRAME_SIZE) {
			msleep(10);
			continue;
		}

		joypad_miyoo_parse(joypad, frame);

		if (joypad_miyoo_inactive(joypad)) {
			dev_warn(joypad->dev,
				 "No data from " MIYOO_TTY_PATH " for 5s; re-opening TTY.\n");
			joypad_miyoo_reopen(joypad);
		}
	}

	dev_info(joypad->dev, "Miyoo serial thread exiting.\n");
	return 0;
}

static void joypad_miyoo_start(struct work_struct *work)
{
	struct joypad_miyoo *miyoo = container_of(to_delayed_work(work),
						  struct joypad_miyoo,
						  start_work);
	struct joypad *joypad = container_of(miyoo, struct joypad, miyoo);
	struct task_struct *thread;
	int error;

	dev_info(joypad->dev, "Starting " MIYOO_TTY_PATH " init after driver load.\n");

	if (joypad_miyoo_open(joypad))
		return;

	/* The only calibration; re-opening the tty later does not repeat it. */
	error = joypad_miyoo_autocal(joypad);
	if (error)
		dev_err(joypad->dev, "Auto-calibration failed: %d\n", error);

	/* Start the inactivity timer before the reader thread can check it. */
	miyoo->last_data_jiffies = jiffies;

	thread = kthread_run(joypad_miyoo_thread, joypad, "miyoo-serial-thread");
	if (IS_ERR(thread)) {
		dev_err(joypad->dev, "Failed to create Miyoo serial thread.\n");
		joypad_miyoo_close(joypad);
		return;
	}
	miyoo->thread = thread;

	dev_info(joypad->dev, "Miyoo serial logic started successfully!\n");
}

static void joypad_miyoo_init(struct joypad *joypad)
{
	static const struct joypad_miyoo_cal defaults = {
		.min	= MIYOO_RAW_MIN,
		.max	= MIYOO_RAW_MAX,
		.zero	= MIYOO_RAW_ZERO,
	};
	struct joypad_miyoo *miyoo = &joypad->miyoo;

	miyoo->left[0] = defaults;
	miyoo->left[1] = defaults;
	miyoo->right[0] = defaults;
	miyoo->right[1] = defaults;

	miyoo->left_x = 0;
	miyoo->left_y = 0;
	miyoo->right_x = 0;
	miyoo->right_y = 0;
}

static void joypad_report_miyoo(struct joypad *joypad)
{
	struct input_dev *input = joypad->input;

	input_report_abs(input, ABS_X, READ_ONCE(joypad->miyoo.left_x));
	input_report_abs(input, ABS_Y, READ_ONCE(joypad->miyoo.left_y));
	input_report_abs(input, ABS_RX, READ_ONCE(joypad->miyoo.right_x));
	input_report_abs(input, ABS_RY, READ_ONCE(joypad->miyoo.right_y));
	input_sync(input);
}

/*----------------------------------------------------------------------------*/
/* Input device */

static void joypad_poll(struct input_dev *input)
{
	struct joypad *joypad = input_get_drvdata(input);

	if (!joypad->enable)
		return;

	if (joypad->use_miyoo_serial)
		joypad_report_miyoo(joypad);
	else
		joypad_report_sticks(joypad);

	joypad_report_buttons(joypad);
}

static int joypad_open(struct input_dev *input)
{
	struct joypad *joypad = input_get_drvdata(input);
	int i;

	if (joypad_is_single_adc(joypad)) {
		/* Report the current state of every button straight away. */
		for (i = 0; i < joypad->button_count; i++) {
			struct joypad_button *btn = &joypad->buttons[i];
			int value = gpiod_get_value_cansleep(btn->desc);

			if (value < 0)
				value = 0;
			btn->old_value = value;
			input_event(input, btn->type, btn->code, value == 1);
		}
		input_sync(input);
	} else {
		/* Every pressed button is reported by the sync below. */
		for (i = 0; i < joypad->button_count; i++)
			joypad->buttons[i].old_value = 0;
	}

	joypad_calibrate_sticks(joypad);

	/* sync the current state */
	joypad_report_sticks(joypad);
	joypad_report_buttons(joypad);

	mutex_lock(&joypad->lock);
	joypad->enable = true;
	mutex_unlock(&joypad->lock);

	if (!joypad_is_single_adc(joypad))
		dev_info(joypad->dev, "opened\n");

	return 0;
}

static void joypad_close(struct input_dev *input)
{
	struct joypad *joypad = input_get_drvdata(input);

	if (joypad->has_rumble) {
		cancel_work_sync(&joypad->play_work);
		joypad_rumble_stop(joypad);
	}

	mutex_lock(&joypad->lock);
	joypad->enable = false;
	mutex_unlock(&joypad->lock);

	if (!joypad_is_single_adc(joypad))
		dev_info(joypad->dev, "closed\n");
}

static int joypad_input_setup(struct device *dev, struct joypad *joypad)
{
	const struct joypad_variant *variant = joypad->variant;
	struct input_dev *input;
	u32 bustype = variant->bustype;
	u32 vendor = 0, product = 0, revision = 0;
	int i, error;

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "no memory for input device\n");
		return -ENOMEM;
	}

	input_set_drvdata(input, joypad);
	joypad->input = input;

#ifdef ROCKNIX_JOYPAD_ADC_KEYS_HOOK
	if (joypad_is_single_adc(joypad))
		joypad_input_g = input;
#endif

	input->name = variant->name;
	device_property_read_string(dev, "joypad-name", &input->name);
	input->phys = variant->phys;

	device_property_read_u32(dev, "joypad-bustype", &bustype);
	device_property_read_u32(dev, "joypad-vendor", &vendor);
	device_property_read_u32(dev, "joypad-revision", &revision);
	device_property_read_u32(dev, "joypad-product", &product);
	input->id.bustype = (u16)bustype;
	input->id.vendor = (u16)vendor;
	input->id.product = (u16)product;
	input->id.version = (u16)revision;

	/* Analog axes: (0 mV ~ 1800 mV) * scale, centred on zero. */
	if (!joypad_is_single_adc(joypad) || joypad->axis_count > 0 ||
	    joypad->use_miyoo_serial)
		__set_bit(EV_ABS, input->evbit);

	for (i = 0; i < joypad->axis_count; i++) {
		struct joypad_axis *axis = &joypad->axes[i];

		input_set_abs_params(input, axis->code, axis->min, axis->max,
				     joypad->adc_fuzz, joypad->adc_flat);
		dev_info(dev,
			 "axis %s: SCALE = %d, ABS min = %d, max = %d, fuzz = %d, flat = %d, deadzone = %d\n",
			 axis->name, axis->scale, axis->min, axis->max,
			 joypad->adc_fuzz, joypad->adc_flat,
			 joypad->adc_deadzone);
		dev_info(dev,
			 "axis %s: adc tuning_p = %d, adc tuning_n = %d invert = %d\n",
			 axis->name, axis->tuning_p, axis->tuning_n,
			 axis->invert);
	}

	/* The Miyoo sticks always provide all four axes. */
	if (joypad->use_miyoo_serial) {
		input_set_abs_params(input, ABS_X, MIYOO_AXIS_MIN, MIYOO_AXIS_MAX,
				     MIYOO_AXIS_FUZZ, MIYOO_AXIS_FLAT);
		input_set_abs_params(input, ABS_Y, MIYOO_AXIS_MIN, MIYOO_AXIS_MAX,
				     MIYOO_AXIS_FUZZ, MIYOO_AXIS_FLAT);
		input_set_abs_params(input, ABS_RX, MIYOO_AXIS_MIN, MIYOO_AXIS_MAX,
				     MIYOO_AXIS_FUZZ, MIYOO_AXIS_FLAT);
		input_set_abs_params(input, ABS_RY, MIYOO_AXIS_MIN, MIYOO_AXIS_MAX,
				     MIYOO_AXIS_FUZZ, MIYOO_AXIS_FLAT);
	}

	if (joypad->has_rumble) {
		u32 boost_weak = 0, boost_strong = 0;

		device_property_read_u32(dev, "rumble-boost-weak", &boost_weak);
		device_property_read_u32(dev, "rumble-boost-strong", &boost_strong);
		joypad->boost_weak = boost_weak;
		joypad->boost_strong = boost_strong;
		dev_info(dev, "Boost = %d, %d", boost_weak, boost_strong);

		input_set_capability(input, EV_FF, FF_RUMBLE);
		error = input_ff_create_memless(input, joypad, joypad_rumble_play);
		if (error) {
			dev_err(dev, "unable to register rumble, err=%d\n", error);
			return error;
		}
	}

	__set_bit(EV_KEY, input->evbit);
	for (i = 0; i < joypad->button_count; i++)
		input_set_capability(input, joypad->buttons[i].type,
				     joypad->buttons[i].code);

	if (joypad->auto_repeat)
		__set_bit(EV_REP, input->evbit);

	input->open = joypad_open;
	input->close = joypad_close;

	error = input_setup_polling(input, joypad_poll);
	if (error) {
		dev_err(dev, "unable to set up polling, err=%d\n", error);
		return error;
	}
	input_set_poll_interval(input, joypad->poll_interval);

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "unable to register input device, err=%d\n", error);
		return error;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
/* Device tree */

static void joypad_axis_init(struct joypad *joypad, struct joypad_axis *axis,
			     const char *name, int code)
{
	axis->name = name;
	axis->code = code;
	axis->scale = joypad->adc_scale;
	axis->max = ADC_MAX_VOLTAGE / 2;
	axis->min = -(ADC_MAX_VOLTAGE / 2);
	if (axis->scale) {
		axis->max *= axis->scale;
		axis->min *= axis->scale;
	}
}

static void joypad_axis_read_tuning(struct device *dev, struct joypad_axis *axis)
{
	char prop[16];	/* "abs_ry-n-tuning" + NUL */
	u32 val;

	snprintf(prop, sizeof(prop), "abs_%s-p-tuning", axis->name);
	axis->tuning_p = device_property_read_u32(dev, prop, &val) ?
			 ADC_TUNING_DEFAULT : val;

	snprintf(prop, sizeof(prop), "abs_%s-n-tuning", axis->name);
	axis->tuning_n = device_property_read_u32(dev, prop, &val) ?
			 ADC_TUNING_DEFAULT : val;
}

static int joypad_get_voltage_channel(struct device *dev, const char *name,
				      bool may_defer, struct iio_channel **out)
{
	struct iio_channel *chan;
	enum iio_chan_type type;

	chan = devm_iio_channel_get(dev, name);
	if (may_defer && PTR_ERR(chan) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (IS_ERR(chan)) {
		dev_err(dev, "iio channel get error\n");
		return -EINVAL;
	}
	if (!chan->indio_dev)
		return -ENXIO;

	if (iio_get_channel_type(chan, &type))
		return -EINVAL;
	if (type != IIO_VOLTAGE) {
		dev_err(dev, "Incompatible channel type %d\n", type);
		return -EINVAL;
	}

	*out = chan;
	return 0;
}

/*
 * "rocknix-joypad": one IIO channel per axis, in "io-channel-names" order.
 * The axes are right stick first unless "abs-left-first" is set, and each
 * can be inverted with "invert-abs<axis>" (present, or a non-zero u32).
 */
static int joypad_multi_adc_setup(struct device *dev, struct joypad *joypad)
{
	static const struct joypad_axis_map right_first[] = {
		{ "rx", ABS_RX }, { "ry", ABS_RY },
		{ "x", ABS_X }, { "y", ABS_Y },
		{ "rz", ABS_RZ }, { "z", ABS_Z },
	};
	static const struct joypad_axis_map left_first[] = {
		{ "x", ABS_X }, { "y", ABS_Y },
		{ "rx", ABS_RX }, { "ry", ABS_RY },
		{ "z", ABS_Z }, { "rz", ABS_RZ },
	};
	const struct joypad_axis_map *map;
	int i, error;

	if (joypad->axis_count > (int)ARRAY_SIZE(right_first)) {
		dev_err(dev, "io channel count(%d) error!", joypad->axis_count);
		return -EINVAL;
	}

	joypad->axes = devm_kcalloc(dev, joypad->axis_count,
				    sizeof(*joypad->axes), GFP_KERNEL);
	if (!joypad->axes)
		return -ENOMEM;

	map = device_property_read_bool(dev, "abs-left-first") ?
	      left_first : right_first;

	for (i = 0; i < joypad->axis_count; i++) {
		struct joypad_axis *axis = &joypad->axes[i];
		char prop[16];	/* "invert-absry" + NUL */
		u32 invert;

		joypad_axis_init(joypad, axis, map[i].name, map[i].code);

		/*
		 * A bare boolean is not overlay friendly, so a u32 can also
		 * be given: 0 = normal, anything else = inverted. A present
		 * property that is not a u32 means inverted.
		 */
		snprintf(prop, sizeof(prop), "invert-abs%s", axis->name);
		if (device_property_present(dev, prop))
			axis->invert = device_property_read_u32(dev, prop, &invert) ?
				       true : invert != 0;
		else
			axis->invert = false;

		joypad_axis_read_tuning(dev, axis);
	}

	for (i = 0; i < joypad->axis_count; i++) {
		const char *name;

		error = of_property_read_string_index(dev->of_node,
						      "io-channel-names", i, &name);
		if (error < 0) {
			dev_err(dev, "invalid channel name index[%d]\n", i);
			return -EINVAL;
		}

		error = joypad_get_voltage_channel(dev, name, false,
						   &joypad->axes[i].chan);
		if (error)
			return error;
	}

	return 0;
}

/*
 * "rocknix-singleadc-joypad": up to four axes, in the fixed order
 * RY, RX, Y, X, all read from "amux_adc" through the analog mux.
 * "amux-channel-mapping" gives the mux input of each axis, and
 * "invert-abs<axis>" inverts an axis when present, whatever its value.
 */
static int joypad_single_adc_setup(struct device *dev, struct joypad *joypad)
{
	static const struct joypad_axis_map order[] = {
		{ "ry", ABS_RY }, { "rx", ABS_RX },
		{ "y", ABS_Y }, { "x", ABS_X },
	};
	u32 mux_sel[ARRAY_SIZE(order)] = { 0, 1, 2, 3 };
	struct iio_channel *chan;
	int i, error;

	if (device_property_present(dev, "amux-channel-mapping")) {
		error = of_property_read_u32_array(dev->of_node,
						   "amux-channel-mapping",
						   mux_sel, ARRAY_SIZE(mux_sel));
		if (error < 0) {
			dev_err(dev, "invalid channel mapping\n");
			return -EINVAL;
		}
	}

	if (joypad->axis_count > (int)ARRAY_SIZE(order)) {
		dev_err(dev, "amux count(%d) error!", joypad->axis_count);
		return -EINVAL;
	}

	joypad->axes = devm_kcalloc(dev, joypad->axis_count,
				    sizeof(*joypad->axes), GFP_KERNEL);
	if (!joypad->axes)
		return -ENOMEM;

	for (i = 0; i < joypad->axis_count; i++) {
		struct joypad_axis *axis = &joypad->axes[i];
		char prop[16];	/* "invert-absry" + NUL */

		joypad_axis_init(joypad, axis, order[i].name, order[i].code);

		snprintf(prop, sizeof(prop), "invert-abs%s", axis->name);
		axis->invert = device_property_present(dev, prop);

		joypad_axis_read_tuning(dev, axis);
		axis->mux_sel = mux_sel[i];
	}

	error = joypad_get_voltage_channel(dev, "amux_adc", true, &chan);
	if (error)
		return error;
	for (i = 0; i < joypad->axis_count; i++)
		joypad->axes[i].chan = chan;

	joypad->mux_a = devm_gpiod_get(dev, "amux-a", GPIOD_OUT_LOW);
	if (IS_ERR(joypad->mux_a)) {
		error = PTR_ERR(joypad->mux_a);
		dev_err(dev, "failed to request amux-a, error: %d\n", error);
		return error;
	}

	joypad->mux_b = devm_gpiod_get(dev, "amux-b", GPIOD_OUT_LOW);
	if (IS_ERR(joypad->mux_b)) {
		error = PTR_ERR(joypad->mux_b);
		dev_err(dev, "failed to request amux-b, error: %d\n", error);
		return error;
	}

	joypad->mux_en = devm_gpiod_get_optional(dev, "amux-en", GPIOD_OUT_LOW);
	if (IS_ERR(joypad->mux_en)) {
		error = PTR_ERR(joypad->mux_en);
		dev_err(dev, "failed to request amux-en, error: %d\n", error);
		return error;
	}

	return 0;
}

/* One button per available child node: label, gpios, linux,code. */
static int joypad_buttons_setup(struct device *dev, struct joypad *joypad)
{
	struct device_node *child;
	int i = 0;

	if (!dev->of_node)
		return -ENODEV;

	joypad->buttons = devm_kcalloc(dev, joypad->button_count,
				       sizeof(*joypad->buttons), GFP_KERNEL);
	if (!joypad->buttons)
		return -ENOMEM;

	/*
	 * button_count came from device_get_child_node_count(), which only
	 * counts available nodes, so walk the same set.
	 * TODO: use for_each_available_child_of_node_scoped() once kernels
	 * older than 6.11 are no longer supported.
	 */
	for_each_available_child_of_node(dev->of_node, child) {
		struct joypad_button *btn;
		u32 val;

		if (i >= joypad->button_count) {
			of_node_put(child);
			break;
		}
		btn = &joypad->buttons[i++];

		btn->label = of_get_property(child, "label", NULL);

		btn->desc = devm_fwnode_gpiod_get(dev, of_fwnode_handle(child),
						  NULL, GPIOD_IN, btn->label);
		if (IS_ERR(btn->desc)) {
			int error = PTR_ERR(btn->desc);

			dev_err(dev, "Failed to request GPIO %s, error %d\n",
				btn->label, error);
			of_node_put(child);
			return error;
		}

		if (of_property_read_u32(child, "linux,code", &val)) {
			dev_err(dev, "Button without keycode: %s\n", btn->label);
			of_node_put(child);
			return -EINVAL;
		}
		btn->code = val;

		btn->type = of_property_read_u32(child, "linux,input-type", &val) ?
			    EV_KEY : val;
	}

	joypad->button_count = i;
	if (i == 0)
		return -EINVAL;

	return 0;
}

static int joypad_parse_dt(struct device *dev, struct joypad *joypad)
{
	u32 val;
	int error;

	if (!device_property_read_u32(dev, "button-adc-fuzz", &val))
		joypad->adc_fuzz = val;
	if (!device_property_read_u32(dev, "button-adc-flat", &val))
		joypad->adc_flat = val;
	if (!device_property_read_u32(dev, "button-adc-scale", &val))
		joypad->adc_scale = val;
	if (!device_property_read_u32(dev, "button-adc-deadzone", &val))
		joypad->adc_deadzone = val;
	if (!device_property_read_u32(dev, "poll-interval", &val))
		joypad->poll_interval = val;

	joypad->auto_repeat = device_property_present(dev, "autorepeat");
	joypad->button_count = device_get_child_node_count(dev);

	if (joypad_is_single_adc(joypad)) {
		if (!device_property_read_u32(dev, "amux-count", &val))
			joypad->axis_count = val;

		dev_info(dev,
			 "invert-absx = %d, invert-absy = %d, invert-absrx = %d, invert-absry = %d\n",
			 device_property_present(dev, "invert-absx"),
			 device_property_present(dev, "invert-absy"),
			 device_property_present(dev, "invert-absrx"),
			 device_property_present(dev, "invert-absry"));

		/* Not fatal: the Miyoo serial sticks need no ADC. */
		if (joypad->axis_count == 0 && joypad->button_count == 0)
			dev_err(dev, "adc key = %d, gpio key = %d error!",
				joypad->axis_count, joypad->button_count);

		if (joypad->axis_count > 0) {
			error = joypad_single_adc_setup(dev, joypad);
			if (error)
				return error;
		}
	} else {
		joypad->axis_count = of_property_count_strings(dev->of_node,
							       "io-channel-names");
		if (joypad->axis_count <= 0 || joypad->button_count == 0) {
			dev_err(dev, "adc key = %d, gpio key = %d error!",
				joypad->axis_count, joypad->button_count);
			return -EINVAL;
		}

		error = joypad_multi_adc_setup(dev, joypad);
		if (error)
			return error;
	}

	error = joypad_buttons_setup(dev, joypad);
	if (error)
		return error;

	dev_info(dev, "adc key cnt = %d, gpio key cnt = %d\n",
		 joypad->axis_count, joypad->button_count);

	/* Rumble and the Miyoo sticks only exist on the single ADC layout. */
	if (joypad_is_single_adc(joypad)) {
		joypad->has_rumble = device_property_present(dev, "pwm-names");
		if (joypad->has_rumble)
			dev_info(dev, "has rumble\n");

		joypad->use_miyoo_serial =
			device_property_present(dev, "rocknix,use-miyoo-serial-joypad");
		if (joypad->use_miyoo_serial) {
			dev_info(dev, "using Miyoo Serial Joypad logic\n");
			joypad_miyoo_init(joypad);
		}
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
/* Power management */

static int joypad_suspend(struct device *dev)
{
	struct joypad *joypad = dev_get_drvdata(dev);

	if (joypad->has_rumble) {
		cancel_work_sync(&joypad->play_work);
		if (joypad->level)
			joypad_rumble_stop(joypad);
	}

	return 0;
}

static int joypad_resume(struct device *dev)
{
	struct joypad *joypad = dev_get_drvdata(dev);
	u16 level;

	if (joypad->has_rumble) {
		/*
		 * The level can be left non-zero after rumble was disabled (a
		 * play callback racing the rumble_enable store), so check both.
		 */
		level = READ_ONCE(joypad->level);
		if (READ_ONCE(joypad->rumble_enabled) && level)
			joypad_rumble_start(joypad, level);
	}

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(joypad_pm_ops, joypad_suspend, joypad_resume);

/*----------------------------------------------------------------------------*/
static int joypad_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct joypad *joypad;
	int error;

	joypad = devm_kzalloc(dev, sizeof(*joypad), GFP_KERNEL);
	if (!joypad)
		return -ENOMEM;

	joypad->dev = dev;
	joypad->variant = device_get_match_data(dev);
	if (!joypad->variant)
		return -ENODEV;

	error = joypad_parse_dt(dev, joypad);
	if (error) {
		dev_err(dev, "dt parse error!(err = %d)\n", error);
		return error;
	}

	mutex_init(&joypad->lock);
	if (joypad->has_rumble)
		INIT_WORK(&joypad->play_work, joypad_rumble_work);
	platform_set_drvdata(pdev, joypad);

	error = joypad_input_setup(dev, joypad);
	if (error) {
		dev_err(dev, "input setup failed!(err = %d)\n", error);
		return error;
	}

	if (joypad->has_rumble) {
		error = sysfs_create_group(&dev->kobj, &joypad_rumble_attr_group);
		if (error) {
			dev_err(dev, "create sysfs group fail, error: %d\n", error);
			return error;
		}
		error = devm_add_action_or_reset(dev, joypad_remove_rumble_attrs, dev);
		if (error)
			return error;

		error = joypad_rumble_setup(dev, joypad);
		if (error) {
			dev_err(dev, "rumble setup failed!(err = %d)\n", error);
			return error;
		}

		joypad->rumble_enabled = true;
	}

	if (joypad->use_miyoo_serial) {
		/* Open the tty later, so probe and boot are never blocked. */
		INIT_DELAYED_WORK(&joypad->miyoo.start_work, joypad_miyoo_start);
		schedule_delayed_work(&joypad->miyoo.start_work, MIYOO_START_DELAY);
	}

	dev_info(dev, "probe success\n");
	return 0;
}

static const struct of_device_id joypad_of_match[] = {
	{ .compatible = "rocknix-joypad", .data = &joypad_variant_multi_adc },
	{ .compatible = "rocknix-singleadc-joypad", .data = &joypad_variant_single_adc },
	{ }
};
MODULE_DEVICE_TABLE(of, joypad_of_match);

static struct platform_driver joypad_driver = {
	.probe = joypad_probe,
	.driver = {
		.name = "rocknix-joypad",
		.pm = pm_sleep_ptr(&joypad_pm_ops),
		.of_match_table = joypad_of_match,
	},
};

static int __init joypad_init(void)
{
	return platform_driver_register(&joypad_driver);
}
late_initcall(joypad_init);

static void __exit joypad_exit(void)
{
	platform_driver_unregister(&joypad_driver);
}
module_exit(joypad_exit);

MODULE_AUTHOR("ROCKNIX");
MODULE_DESCRIPTION("ROCKNIX joypad driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rocknix-joypad");
MODULE_ALIAS("platform:rocknix-singleadc-joypad");
/* lets "modprobe rocknix-singleadc-joypad" keep working */
MODULE_ALIAS("rocknix-singleadc-joypad");
MODULE_INFO(intree, "Y");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 13, 0)
MODULE_IMPORT_NS("IIO_CONSUMER");
#endif
