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
 *	This layout can also drive a PWM rumble motor, and gives its input
 *	device to the ROCKNIX adc-keys patch when the kernel carries it.
 *
 * In both layouts the buttons are the GPIOs of the child nodes, and
 * everything is polled at "poll-interval" ms while the device is open.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/consumer.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kstrtox.h>
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
#include <linux/version.h>
#include <linux/workqueue.h>

/*----------------------------------------------------------------------------*/
/* Analog sticks */

/* Full ADC range in mV; axes report +/- half of it, times the scale. */
#define ADC_MAX_VOLTAGE		1800
/* Tuning, in percent, used when an axis has no abs_<axis>-[pn]-tuning. */
#define ADC_TUNING_DEFAULT	180

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

	/* single ADC layout: input device handed to the adc-keys patch */
	bool adc_keys_hooked;
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
/* Input device */

/*
 * The ROCKNIX adc-keys kernel patch exports "joypad_input_g" and reports
 * keycode 316 (BTN_MODE) through the input device stored in it, so that the
 * key comes from the joypad. Only the single ADC layout takes part.
 *
 * The symbol is looked up at run time, so the module also loads on kernels
 * without the patch. Since Linux 6.6 that lookup only finds symbols exported
 * with EXPORT_SYMBOL_GPL(), so the patch has to export it that way.
 */
static void joypad_adc_keys_hook(struct joypad *joypad)
{
	struct input_dev **slot;

	if (!joypad_is_single_adc(joypad) || joypad->adc_keys_hooked)
		return;

	slot = __symbol_get("joypad_input_g");
	if (!slot)
		return;

	*slot = joypad->input;
	joypad->adc_keys_hooked = true;
}

static void joypad_poll(struct input_dev *input)
{
	struct joypad *joypad = input_get_drvdata(input);

	if (!joypad->enable)
		return;

	joypad_report_sticks(joypad);
	joypad_report_buttons(joypad);
}

static int joypad_open(struct input_dev *input)
{
	struct joypad *joypad = input_get_drvdata(input);
	int i;

	/* In case adc-keys is a module that was loaded after the probe. */
	joypad_adc_keys_hook(joypad);

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

	joypad_adc_keys_hook(joypad);

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
	if (!joypad_is_single_adc(joypad) || joypad->axis_count > 0)
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

		if (joypad->axis_count == 0 && joypad->button_count == 0) {
			dev_err(dev, "adc key = %d, gpio key = %d error!",
				joypad->axis_count, joypad->button_count);
			return -EINVAL;
		}

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

	/* Rumble only exists on the single ADC layout. */
	if (joypad_is_single_adc(joypad)) {
		joypad->has_rumble = device_property_present(dev, "pwm-names");
		if (joypad->has_rumble)
			dev_info(dev, "has rumble\n");
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
