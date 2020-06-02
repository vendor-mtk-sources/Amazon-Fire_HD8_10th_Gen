/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef LINUX_POWER_CHARGER_CLASS_H
#define LINUX_POWER_CHARGER_CLASS_H

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>


struct charger_properties {
	const char *alias_name;
};

/* Data of notifier from charger device */
struct chgdev_notify {
	bool vbusov_stat;
};

struct charger_device {
	struct charger_properties props;
	struct chgdev_notify noti;
	const struct charger_ops *ops;
	struct mutex ops_lock;
	struct device dev;
	struct srcu_notifier_head evt_nh;
	void	*driver_data;
	bool is_polling_mode;
};

struct charger_ops {
	int (*suspend)(struct charger_device *cdev, pm_message_t p);
	int (*resume)(struct charger_device *cdev);

	/* cable plug in/out */
	int (*plug_in)(struct charger_device *cdev);
	int (*plug_out)(struct charger_device *cdev);

	/* enable/disable charger */
	int (*enable)(struct charger_device *cdev, bool en);

	int (*is_enabled)(struct charger_device *cdev, bool *en);

	/* enable/disable chip */
	int (*enable_chip)(struct charger_device *cdev, bool en);
	int (*is_chip_enabled)(struct charger_device *cdev, bool *en);

	/* get/set charging current*/
	int (*get_charging_current)(struct charger_device *cdev, u32 *uA);
	int (*set_charging_current)(struct charger_device *cdev, u32 uA);
	int (*get_min_charging_current)(struct charger_device *cdev, u32 *uA);

	/* set cv */
	int (*set_constant_voltage)(struct charger_device *cdev, u32 uV);
	int (*get_constant_voltage)(struct charger_device *cdev, u32 *uV);

	/* set input_current */
	int (*get_input_current)(struct charger_device *cdev, u32 *uA);
	int (*set_input_current)(struct charger_device *cdev, u32 uA);
	int (*get_min_input_current)(struct charger_device *cdev, u32 *uA);

	/* set termination current */
	int (*get_eoc_current)(struct charger_device *cdev, u32 *uA);
	int (*set_eoc_current)(struct charger_device *cdev, u32 uA);

	/* kick wdt */
	int (*kick_wdt)(struct charger_device *cdev);

	int (*event)(struct charger_device *cdev, u32 event, u32 args);

	/* PE+/PE+2.0 */
	int (*send_ta_current_pattern)(
	struct charger_device *cdev, bool is_inc);
	int (*send_ta20_current_pattern)(struct charger_device *cdev, u32 uV);
	int (*reset_ta)(struct charger_device *cdev);
	int (*enable_cable_drop_comp)(struct charger_device *cdev, bool en);

	int (*set_mivr)(struct charger_device *cdev, u32 uV);
	int (*get_mivr_state)(struct charger_device *cdev, bool *in_loop);

	/* enable/disable powerpath */
	int (*is_powerpath_enabled)(struct charger_device *cdev, bool *en);
	int (*enable_powerpath)(struct charger_device *cdev, bool en);

	/* enable/disable vbus ovp */
	int (*enable_vbus_ovp)(struct charger_device *cdev, bool en);

	/* enable/disable charging safety timer */
	int (*is_safety_timer_enabled)(struct charger_device *cdev, bool *en);
	int (*enable_safety_timer)(struct charger_device *cdev, bool en);

	/* enable term */
	int (*enable_termination)(struct charger_device *cdev, bool en);

	/* direct charging */
	int (*enable_direct_charging)(struct charger_device *cdev, bool en);
	int (*kick_direct_charging_wdt)(struct charger_device *cdev);
	int (*set_direct_charging_ibusoc)(struct charger_device *cdev, u32 uA);
	int (*set_direct_charging_vbusov)(struct charger_device *cdev, u32 uV);

	/* OTG */
	int (*enable_otg)(struct charger_device *cdev, bool en);
	int (*enable_discharge)(struct charger_device *cdev, bool en);
	int (*set_boost_current_limit)(struct charger_device *cdev, u32 uA);

	/* charger type detection */
	int (*enable_chg_type_det)(struct charger_device *cdev, bool en);

	/* run AICL */
	int (*run_aicl)(struct charger_device *cdev, u32 *uA);

	/* reset EOC state */
	int (*reset_eoc_state)(struct charger_device *cdev);

	int (*safety_check)(struct charger_device *cdev);

	int (*is_charging_done)(struct charger_device *cdev, bool *done);
	int (*set_pe20_efficiency_table)(struct charger_device *cdev);
	int (*dump_registers)(struct charger_device *cdev);

	int (*get_ibus_adc)(struct charger_device *cdev, u32 *ibus);
	int (*get_tchg_adc)(struct charger_device *cdev, int *tchg_min,
		int *tchg_max);
	int (*get_zcv)(struct charger_device *cdev, u32 *uV);
};

static inline
void *charger_dev_get_drvdata(const struct charger_device *chg_dev)
{
	return chg_dev->driver_data;
}

static inline void charger_dev_set_drvdata(struct charger_device *chg_dev,
					void *data)
{
	chg_dev->driver_data = data;
}

extern struct charger_device *charger_device_register(const char *name,
	struct device *parent, void *devdata, const struct charger_ops *ops,
	const struct charger_properties *props);
extern void charger_device_unregister(struct charger_device *chg_dev);
extern struct charger_device *get_charger_by_name(const char *name);

#define to_charger_device(obj) container_of(obj, struct charger_device, dev)

static inline void *charger_get_data(struct charger_device *chg_dev)
{
	return dev_get_drvdata(&chg_dev->dev);
}

extern int charger_dev_enable(struct charger_device *chg_dev, bool en);
extern int charger_dev_is_enabled(struct charger_device *chg_dev, bool *en);
extern int charger_dev_plug_in(struct charger_device *chg_dev);
extern int charger_dev_plug_out(struct charger_device *chg_dev);
extern int charger_dev_set_charging_current(struct charger_device *chg_dev,
					u32 uA);
extern int charger_dev_get_charging_current(struct charger_device *chg_dev,
					u32 *uA);
extern int charger_dev_get_min_charging_current(struct charger_device *chg_dev,
					u32 *uA);
extern int charger_dev_set_input_current(struct charger_device *chg_dev,
					u32 uA);
extern int charger_dev_get_input_current(struct charger_device *chg_dev,
					u32 *uA);
extern int charger_dev_get_min_input_current(struct charger_device *chg_dev,
					u32 *uA);
extern int charger_dev_set_eoc_current(struct charger_device *chg_dev, u32 uA);
extern int charger_dev_get_eoc_current(struct charger_device *chg_dev, u32 *uA);
extern int charger_dev_kick_wdt(struct charger_device *chg_dev);
extern int charger_dev_set_constant_voltage(struct charger_device *chg_dev,
					u32 uV);
extern int charger_dev_get_constant_voltage(struct charger_device *chg_dev,
					u32 *uV);
extern int charger_dev_dump_registers(struct charger_device *chg_dev);
extern int charger_dev_enable_vbus_ovp(struct charger_device *chg_dev, bool en);
extern int charger_dev_set_mivr(struct charger_device *chg_dev, u32 uV);
extern int charger_dev_get_mivr_state(struct charger_device *chg_dev,
					bool *in_loop);
extern int charger_dev_do_event(struct charger_device *chg_dev, u32 event,
					u32 args);
extern int charger_dev_is_powerpath_enabled(struct charger_device *chg_dev,
					bool *en);
extern int charger_dev_is_safety_timer_enabled(struct charger_device *chg_dev,
					bool *en);
extern int charger_dev_enable_termination(struct charger_device *chg_dev,
					bool en);
extern int charger_dev_is_charging_done(struct charger_device *chg_dev,
					bool *done);
extern int charger_dev_enable_powerpath(struct charger_device *chg_dev,
					bool en);
extern int charger_dev_enable_safety_timer(struct charger_device *chg_dev,
					bool en);
extern int charger_dev_enable_chg_type_det(struct charger_device *chg_dev,
					bool en);
extern int charger_dev_enable_otg(struct charger_device *chg_dev, bool en);
extern int charger_dev_enable_discharge(struct charger_device *chg_dev,
					bool en);
extern int charger_dev_set_boost_current_limit(struct charger_device *chg_dev,
					u32 uA);
extern int charger_dev_get_zcv(struct charger_device *chg_dev, u32 *uV);
extern int charger_dev_run_aicl(struct charger_device *chg_dev, u32 *uA);
extern int charger_dev_reset_eoc_state(struct charger_device *chg_dev);
extern int charger_dev_safety_check(struct charger_device *chg_dev);

/* PE+/PE+2.0 */
extern int charger_dev_send_ta_current_pattern(struct charger_device *chg_dev,
					bool is_increase);
extern int charger_dev_send_ta20_current_pattern(struct charger_device *chg_dev,
					u32 uV);
extern int charger_dev_reset_ta(struct charger_device *chg_dev);
extern int
charger_dev_set_pe20_efficiency_table(struct charger_device *chg_dev);
extern int charger_dev_enable_cable_drop_comp(struct charger_device *chg_dev,
					bool en);

extern int charger_dev_enable_chip(struct charger_device *chg_dev, bool en);
extern int charger_dev_is_chip_enabled(struct charger_device *chg_dev,
					bool *en);
extern int charger_dev_get_ibus(struct charger_device *chg_dev, u32 *ibus);
extern int charger_dev_get_temperature(struct charger_device *chg_dev,
					int *tchg_min, int *tchg_max);


extern int register_charger_device_notifier(struct charger_device *chg_dev,
			      struct notifier_block *nb);
extern int unregister_charger_device_notifier(struct charger_device *chg_dev,
				struct notifier_block *nb);
extern int charger_dev_notify(struct charger_device *chg_dev, int event);
/* WPC */
extern int wireless_charger_dev_get_online(
	struct charger_device *chg_dev, bool *stat);
extern int wireless_charger_dev_do_algorithm(
	struct charger_device *chg_dev, void *data);
#endif /*LINUX_POWER_CHARGER_CLASS_H*/
