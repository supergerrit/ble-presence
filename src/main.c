#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

#define N 2

static void start_scan(void);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_ALIAS(led1_green), gpios);
static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(DT_ALIAS(led1_red), gpios);
static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(DT_ALIAS(led1_blue), gpios);

struct scan_result
{
	bt_addr_le_t *addr;
	int8_t rssi;
};

static double pow(double x, double y)
{
	double result = 1;

	if (y < 0)
	{
		y = -y;
		while (y--)
		{
			result /= x;
		}
	}
	else
	{
		while (y--)
		{
			result *= x;
		}
	}

	return result;
}

static int abs(int n)
{
	if (n < 0)
	{
		return -n;
	}
	else
	{
		return n;
	}
}

double calculate_distance(int rssi, int rssi_1m)
{
	double ratio = rssi * 1.0 / rssi_1m;
	double distance = pow(10, (abs(ratio) - 0.4) / 2.0);
	return distance;
}

void blink_work_handler(struct k_work *work)
{
	gpio_pin_set_dt(&led, 1);
	k_msleep(200);
	gpio_pin_set_dt(&led, 0);
	k_msleep(100);
}

K_WORK_DEFINE(blink_work, blink_work_handler);


// Array met de UUID's van de sensoren
uint8_t uuids[4][14] = {
    {0x33, 0x1f, 0x8d, 0x59, 0xc2, 0xe5, 0x4a, 0xa4, 0xbe, 0x17, 0xe0, 0xa1, 0x66, 0x3f}, // Jarno
    {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88},
    {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee},
    {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54}
};


static bool eir_found(struct bt_data *data, void *user_data)
{
	// Controleer of het data type overeenkomt met de gewenste iBeacon data
	if (data->type != BT_DATA_MANUFACTURER_DATA)
	{
		return true;
	}

	// Vergelijk de advertentiedata met de gewenste iBeacon UUID
	for (int i = 0; i < 4; i++)
	{
		if (memcmp(&data->data[4], &uuids[i], sizeof(uuids[i]) - 4) == 0)
		{
			// Blink led on beacon found
			k_work_submit(&blink_work);

			struct scan_result *res = user_data;
			uint8_t rssi_1m = data->data[24];
			int16_t r = 0xFF00 + rssi_1m;

			// double distance = calculate_distance(res->rssi, rssi_1m);

			if (res->rssi > r)
			{
				gpio_pin_set_dt(&led_blue, 1);
			}
			else
			{
				gpio_pin_set_dt(&led_blue, 0);
			}

			return false;
		}
	}

	return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad)
{

	if (type == BT_GAP_ADV_TYPE_ADV_SCAN_IND)
	{
		struct scan_result res = {
			.addr = addr,
			.rssi = rssi,
		};

		bt_data_parse(ad, eir_found, &res);
	}
}

static void start_scan(void)
{
	int err;

	struct bt_le_scan_param scan_param = {
		.type = BT_LE_SCAN_TYPE_PASSIVE,
		.options = BT_LE_SCAN_OPT_NONE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
	};

	err = bt_le_scan_start(&scan_param, device_found);
	if (err)
	{
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

void main(void)
{
	int err;
	err = bt_enable(NULL);

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_ACTIVE);

	gpio_pin_set_dt(&led, 0);
	gpio_pin_set_dt(&led_blue, 1);
	gpio_pin_set_dt(&led_green, 0);
	gpio_pin_set_dt(&led_red, 0);

	k_msleep(1000);
	gpio_pin_set_dt(&led_blue, 0);
	start_scan();
}
