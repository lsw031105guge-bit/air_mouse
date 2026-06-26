#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/sys/printk.h>

#include "hog.h"

/*------------------------------------------------------------------*/
/* Advertising Data                                                 */
/*------------------------------------------------------------------*/

static const struct bt_data ad[] = {
	BT_DATA_BYTES(
		BT_DATA_FLAGS,
		(BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(
		BT_DATA_UUID16_ALL,
		BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL)),
};

static const struct bt_data sd[] = {
	BT_DATA(
		BT_DATA_NAME_COMPLETE,
		CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/*------------------------------------------------------------------*/
/* Connection Callbacks                                             */
/*------------------------------------------------------------------*/
static void connected(struct bt_conn *conn, uint8_t err)
{
    int sec_err;

    if (err) {
        printk("Connection failed (%u)\n", err);
        return;
    }

    printk("Connected\n");

    sec_err = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (sec_err) {
        printk("Failed to request security (%d)\n", sec_err);
    }
}
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (%u)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/*------------------------------------------------------------------*/
/* Main                                                             */
/*------------------------------------------------------------------*/

int main(void)
{
	int err;

	printk("XIAO BLE HID Air Mouse\n");

	printk("Enabling Bluetooth...\n");
	err = bt_enable(NULL);

	if (err) {
		printk("Bluetooth enable failed (%d)\n", err);
		return err;
	}

	printk("Bluetooth ready\n");

	hog_init();

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		printk("Loading settings...\n");
		err = settings_load();
		if (err) {
			printk("Settings load failed (%d), continuing\n", err);
		}
	}

	err = bt_le_adv_start(
		BT_LE_ADV_CONN_FAST_1,
		ad,
		ARRAY_SIZE(ad),
		sd,
		ARRAY_SIZE(sd));

	if (err) {
		printk("Advertising failed (%d)\n", err);
		return err;
	}

	printk("Advertising started\n");

	/* Never returns */
	hog_mouse_loop();

	return 0;
}
