#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/regulator.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* HID Info / Report Reference structures                              */
/* ------------------------------------------------------------------ */

enum {
    HIDS_REMOTE_WAKE       = BIT(0),
    HIDS_NORMALLY_CONNECTABLE = BIT(1),
};

struct hids_info {
    uint16_t version; /* HID version (BCD) */
    uint8_t  code;    /* Country code       */
    uint8_t  flags;
} __packed;

struct hids_report {
    uint8_t id;   /* Report ID   */
    uint8_t type; /* Report type */
} __packed;

static struct hids_info info = {
    .version = 0x0000,
    .code    = 0x00,
    .flags   = HIDS_NORMALLY_CONNECTABLE,
};

enum {
    HIDS_INPUT = 0x01,
};

static struct hids_report input = {
    .id   = 0x01,
    .type = HIDS_INPUT,
};

static uint8_t simulate_input; /* set by CCC callback when host enables notify */
static uint8_t ctrl_point;

/* ------------------------------------------------------------------ */
/* HID Report Descriptor                                               */
/*                                                                     */
/* Mouse, Report ID 1:                                                 */
/*   Byte 0  – buttons (3 bits) + padding (5 bits)                    */
/*   Byte 1  – X  (int8, relative)                                    */
/*   Byte 2  – Y  (int8, relative)                                    */
/*   Total   = 3 bytes                                                 */
/* ------------------------------------------------------------------ */
static uint8_t report_map[] = {
    0x05, 0x01,  /* Usage Page (Generic Desktop)    */
    0x09, 0x02,  /* Usage (Mouse)                   */
    0xA1, 0x01,  /* Collection (Application)        */
    0x85, 0x01,  /*   Report ID (1)                 */
    0x09, 0x01,  /*   Usage (Pointer)               */
    0xA1, 0x00,  /*   Collection (Physical)         */

    /* Buttons: 3 × 1-bit */
    0x05, 0x09,  /*     Usage Page (Button)         */
    0x19, 0x01,  /*     Usage Minimum (1)           */
    0x29, 0x03,  /*     Usage Maximum (3)           */
    0x15, 0x00,  /*     Logical Minimum (0)         */
    0x25, 0x01,  /*     Logical Maximum (1)         */
    0x95, 0x03,  /*     Report Count (3)            */
    0x75, 0x01,  /*     Report Size (1)             */
    0x81, 0x02,  /*     Input (Data, Var, Abs)      */

    /* Padding: 5 bits */
    0x95, 0x01,  /*     Report Count (1)            */
    0x75, 0x05,  /*     Report Size (5)             */
    0x81, 0x03,  /*     Input (Const)               */

    /* X, Y: 2 × int8 relative */
    0x05, 0x01,  /*     Usage Page (Generic Desktop)*/
    0x09, 0x30,  /*     Usage (X)                   */
    0x09, 0x31,  /*     Usage (Y)                   */
    0x15, 0x81,  /*     Logical Minimum (-127)      */
    0x25, 0x7F,  /*     Logical Maximum (127)       */
    0x75, 0x08,  /*     Report Size (8)             */
    0x95, 0x02,  /*     Report Count (2)            */
    0x81, 0x06,  /*     Input (Data, Var, Rel)      */

    0xC0,        /*   End Collection (Physical)     */
    0xC0,        /* End Collection (Application)    */
};

/* ------------------------------------------------------------------ */
/* GATT read / write callbacks                                         */
/* ------------------------------------------------------------------ */

static ssize_t read_info(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             attr->user_data, sizeof(struct hids_info));
}

static ssize_t read_report_map(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             report_map, sizeof(report_map));
}

static ssize_t read_report(struct bt_conn *conn,
                           const struct bt_gatt_attr *attr,
                           void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             attr->user_data, sizeof(struct hids_report));
}

static void input_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    simulate_input = (value == BT_GATT_CCC_NOTIFY);
    printk("CCC changed: notifications %s\n",
           simulate_input ? "enabled" : "disabled");
}

static ssize_t read_input_report(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, NULL, 0);
}

static ssize_t write_ctrl_point(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                const void *buf, uint16_t len,
                                uint16_t offset, uint8_t flags)
{
    uint8_t *value = attr->user_data;

    if (offset + len > sizeof(ctrl_point)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(value + offset, buf, len);
    return len;
}

/* ------------------------------------------------------------------ */
/* GATT Service Definition                                             */
/*                                                                     */
/* Attribute index map (used by bt_gatt_notify below):                */
/*   [0]  Primary Service                                             */
/*   [1]  Char declaration  – HID Info                               */
/*   [2]  HID Info value                                              */
/*   [3]  Char declaration  – Report Map                              */
/*   [4]  Report Map value                                            */
/*   [5]  Char declaration  – Report (Input)                         */
/*   [6]  Report value      ← notify target                          */
/*   [7]  CCC descriptor                                              */
/*   [8]  Report Reference descriptor                                 */
/*   [9]  Char declaration  – HID Control Point                      */
/*   [10] HID Control Point value                                     */
/* ------------------------------------------------------------------ */
#define HOG_REPORT_ATTR_IDX 6

BT_GATT_SERVICE_DEFINE(hog_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),

    BT_GATT_CHARACTERISTIC(
        BT_UUID_HIDS_INFO,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        read_info, NULL, &info),

    BT_GATT_CHARACTERISTIC(
        BT_UUID_HIDS_REPORT_MAP,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        read_report_map, NULL, NULL),

    /*
     * FIX: Report characteristic and its CCC MUST use _ENCRYPT
     * permissions. HID hosts (Windows, Android, iOS) enforce this
     * and will disconnect with reason 0x13 if plain READ is used.
     */
    BT_GATT_CHARACTERISTIC(
        BT_UUID_HIDS_REPORT,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,          /* <-- was BT_GATT_PERM_READ */
        read_input_report, NULL, NULL),

    BT_GATT_CCC(
        input_ccc_changed,
        BT_GATT_PERM_READ_ENCRYPT |         /* <-- was BT_GATT_PERM_READ  */
        BT_GATT_PERM_WRITE_ENCRYPT),        /* <-- was BT_GATT_PERM_WRITE */

    BT_GATT_DESCRIPTOR(
        BT_UUID_HIDS_REPORT_REF,
        BT_GATT_PERM_READ,
        read_report, NULL, &input),

    BT_GATT_CHARACTERISTIC(
        BT_UUID_HIDS_CTRL_POINT,
        BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE,
        NULL, write_ctrl_point, &ctrl_point)
);

/* ------------------------------------------------------------------ */
/* IMU helpers                                                         */
/* ------------------------------------------------------------------ */

#define IMU_NODE DT_ALIAS(imu0)

#if DT_NODE_HAS_STATUS(DT_NODELABEL(imu_vdd), okay)
static const struct device *const imu_vdd_dev =
    DEVICE_DT_GET(DT_NODELABEL(imu_vdd));
#define HAS_IMU_VDD 1
#else
#define HAS_IMU_VDD 0
#endif

static float sv_to_float(const struct sensor_value *val)
{
    return (float)val->val1 + ((float)val->val2 / 1000000.0f);
}

/* Scale gyro rad/s → mouse delta (-127..127). Tune GYRO_SCALE as needed. */
#define GYRO_SCALE 20.0f

static int8_t clamp8(float v)
{
    if (v >  127.0f) return  127;
    if (v < -127.0f) return -127;
    return (int8_t)v;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void hog_init(void)
{
    printk("HID over GATT service initialised\n");
}

void hog_mouse_loop(void)
{
    const struct device *imu = DEVICE_DT_GET(IMU_NODE);
    int ret;

    printk("Powering up IMU...\n");

#if HAS_IMU_VDD
    ret = regulator_enable(imu_vdd_dev);
    if (ret < 0 && ret != -EALREADY) {
        printk("imu_vdd enable failed (%d)\n", ret);
        return;
    }
    k_sleep(K_MSEC(20)); /* wait for rail to stabilise */
#endif

    if (!device_is_ready(imu)) {
        ret = device_init(imu);
        if (ret < 0 && ret != -EALREADY) {
            printk("device_init failed (%d)\n", ret);
            return;
        }
    }

    if (!device_is_ready(imu)) {
        printk("IMU not ready after init\n");
        return;
    }

    printk("IMU ready\n");

    /*
     * Set gyro ODR at runtime (104 Hz).
     * Do NOT set accel ODR here – CONFIG_LSM6DSL_ACCEL_ODR fixes it
     * at build time and runtime sensor_attr_set is unsupported for it.
     */
    struct sensor_value odr = { .val1 = 104, .val2 = 0 };
    ret = sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ,
                          SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
    if (ret < 0) {
        printk("Gyro ODR set failed (%d), continuing\n", ret);
    }

    while (1) {
        if (simulate_input) {
            struct sensor_value gx, gy;

            ret = sensor_sample_fetch_chan(imu, SENSOR_CHAN_GYRO_XYZ);
            if (ret == 0) {
                sensor_channel_get(imu, SENSOR_CHAN_GYRO_X, &gx);
                sensor_channel_get(imu, SENSOR_CHAN_GYRO_Y, &gy);
            } else {
                printk("sensor fetch failed (%d)\n", ret);
                gx.val1 = gx.val2 = 0;
                gy.val1 = gy.val2 = 0;
            }

            /*
             * Report layout (3 bytes, Report ID 1):
             *   [0] buttons (bit0=left, bit1=right, bit2=middle) + 5-bit pad
             *   [1] X delta  (int8, relative)
             *   [2] Y delta  (int8, relative)
             *
             * FIX: report is 3 bytes, matching the descriptor above.
             * Gyro X → mouse Y (tilt forward/back moves cursor up/down).
             * Gyro Z → mouse X (rotate left/right moves cursor left/right).
             * Swap / negate axes to suit your physical mounting.
             */
            int8_t report[3];
            report[0] = 0;  /* no buttons pressed */
            report[1] = clamp8( sv_to_float(&gy) * GYRO_SCALE);  /* X */
            report[2] = clamp8(-sv_to_float(&gx) * GYRO_SCALE);  /* Y */

            /*
             * FIX: use HOG_REPORT_ATTR_IDX (6) – the Report *value*
             * attribute, not the characteristic declaration at [5].
             */
            bt_gatt_notify(NULL,
                           &hog_svc.attrs[HOG_REPORT_ATTR_IDX],
                           report,
                           sizeof(report));
        }

        k_sleep(K_MSEC(10)); /* ~100 Hz report rate */
    }
} 