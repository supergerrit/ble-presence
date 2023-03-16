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

#include <hal/nrf_aar.h>
#include <hal/nrf_nvmc.h>

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

// 33 1f 8d 59 c2 e5 4a a4 be 17 e0 a1 66 3f 0f 4c
uint8_t uuid[18] = {0x33, 0x1f, 0x8d, 0x59, 0xc2, 0xe5, 0x4a, 0xa4, 0xbe, 0x17, 0xe0, 0xa1, 0x66, 0x3f, 64, 00, 01, 00};

// iPad: 08 64 bc 2b 98 22 66 9d 0d 3c db f0 84 c3 20 e5
static const uint8_t irks[3][16] = {
	{0x08, 0x64, 0xbc, 0x2b, 0x98, 0x22, 0x66, 0x9d, 0x0d, 0x3c, 0xdb, 0xf0, 0x84, 0xc3, 0x20, 0xe5},
	{0xe5, 0x20, 0xc3, 0x84, 0xf0, 0xdb, 0x3c, 0x0d, 0x9d, 0x66, 0x22, 0x98, 0x2b, 0xbc, 0x64, 0x08},
	{0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0xA7, 0xB8, 0xC9, 0xD1, 0xE2, 0xF3, 0xA4, 0xB5, 0xC6, 0xD7},
};

uint8_t ipad[6] = {0x46, 0x90, 0xe7, 0xd4, 0xc7, 0x8b};

uint8_t scratch_data[16];

// Define a callback function for the AAR event handler
void aar_event_handler(nrf_aar_event_t event)
{
	switch (event)
	{
	case NRF_AAR_EVENT_RESOLVED:
		// Do something when address is resolved
		nrf_aar_event_clear(NRF_AAR, NRF_AAR_EVENT_RESOLVED);
		break;
	default:
		break;
	}
}

void resolve_address_init()
{
	nrf_aar_enable(NRF_AAR);
	nrf_aar_scratch_pointer_set(NRF_AAR, &scratch_data);

	// Set up IRK pointer and number
	nrf_aar_irk_pointer_set(NRF_AAR, &irks);
	nrf_aar_irk_number_set(NRF_AAR, 3);

	// Enable interrupt for RESOLVED event
	// nrf_aar_int_enable(NRF_AAR, NRF_AAR_INT_RESOLVED_MASK);

	// Register callback function with NVIC
	// NVIC_SetPriority(18, 3);
	// NVIC_SetVector(18, (uint32_t)aar_event_handler);
	// NVIC_EnableIRQ(18);
}

bool resolve_address(uint8_t const *addr_ptr)
{
	// Set up address pointer
	nrf_aar_addr_pointer_set(NRF_AAR, addr_ptr);

	// Start address resolution procedure
	nrf_aar_task_trigger(NRF_AAR, NRF_AAR_TASK_START);

	// Wait for resolution result
	while (!nrf_aar_event_check(NRF_AAR, NRF_AAR_EVENT_END))
		;

	// Check if address was resolved
	if (nrf_aar_event_check(NRF_AAR, NRF_AAR_EVENT_RESOLVED))
	{
		// Get index of matching IRK
		uint8_t irk_index = nrf_aar_resolution_status_get(NRF_AAR);

		// Clear RESOLVED event
		nrf_aar_event_clear(NRF_AAR, NRF_AAR_EVENT_RESOLVED);
		return true;
	}
	else if (nrf_aar_event_check(NRF_AAR, NRF_AAR_EVENT_NOTRESOLVED))
	{
		// Clear NOTRESOLVED event
		nrf_aar_event_clear(NRF_AAR, NRF_AAR_EVENT_NOTRESOLVED);
		return false;
	}
}

static bool eir_found(struct bt_data *data, void *user_data)
{
	// Controleer of het data typeBT_DATA_UUID128_SOME of BT_DATA_UUID128_ALL is
	if (data->type != BT_DATA_MANUFACTURER_DATA)
	{
		return true;
	}

	struct scan_result *res = user_data;
	bt_addr_le_t *addr = res->addr;

	if (addr->type == BT_ADDR_LE_RANDOM_ID || addr->type == BT_ADDR_LE_RANDOM)
	{

		nrf_aar_addr_pointer_set(NRF_AAR, &addr->a.val);
		nrf_aar_task_trigger(NRF_AAR, NRF_AAR_TASK_START);

		// Wait for resolution result
		while (!nrf_aar_event_check(NRF_AAR, NRF_AAR_EVENT_END))
			;

		// Check if address was resolved
		if (nrf_aar_event_check(NRF_AAR, NRF_AAR_EVENT_RESOLVED))
		{
			// Get index of matching IRK
			uint8_t irk_index = nrf_aar_resolution_status_get(NRF_AAR);
			// Clear RESOLVED event
			nrf_aar_event_clear(NRF_AAR, NRF_AAR_EVENT_RESOLVED);
			gpio_pin_set_dt(&led_red, 1);
			return false;
		}
		else if (nrf_aar_event_check(NRF_AAR, NRF_AAR_EVENT_NOTRESOLVED))
		{
			// Clear NOTRESOLVED event
			nrf_aar_event_clear(NRF_AAR, NRF_AAR_EVENT_NOTRESOLVED);
			return true;
		}
	}

	if (memcmp(&addr->a.val, &ipad, sizeof(ipad)) == 0)
	{
		gpio_pin_set_dt(&led_blue, 1);
	}

	// Vergelijk de advertentiedata met de gewenste iBeacon UUID
	if (memcmp(&data->data[4], &uuid, sizeof(uuid) - 4) == 0)
	{
		gpio_pin_set_dt(&led, 1);
		k_msleep(200);
		gpio_pin_set_dt(&led, 0);
		k_msleep(100);

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

	return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad)
{
	if (type == BT_GAP_ADV_TYPE_ADV_SCAN_IND || type == BT_GAP_ADV_TYPE_ADV_IND)
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

	resolve_address_init();

	uint8_t addr_1[6] = {0x4c, 0x93, 0x95, 0x8d, 0x66, 0xf9};
	uint8_t addr_2[6] = {0xf9, 0x66, 0x8d, 0x95, 0x93, 0x4c};
	uint8_t addr_3[6] = {0x46, 0x90, 0xe7, 0xd4, 0xc7, 0x8b};

	if (resolve_address(addr_1) || resolve_address(addr_3) || resolve_address(&addr_3))
	{
		gpio_pin_set_dt(&led_green, 1);
	}

	k_msleep(1000);
	gpio_pin_set_dt(&led_blue, 0);
	start_scan();
}
