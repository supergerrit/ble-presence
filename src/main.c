#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

#include <psa/crypto.h>
#include <psa/crypto_extra.h>

#define NUM_IRKS 3
#define RSSI_1M_APPLE -60

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_ALIAS(led1_green), gpios);
static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(DT_ALIAS(led1_red), gpios);
static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(DT_ALIAS(led1_blue), gpios);

struct device *dev;
int64_t last_beacon = 0;
int64_t last_beacon_in_range = 0;
bool in_range = false;

// Work handlers
void blink_work_handler(struct k_work *work)
{
	gpio_pin_set_dt(&led, 1);
	k_msleep(100);
	gpio_pin_set_dt(&led, 0);
	k_msleep(50);
}

void check_beacon_timeout_work_handler(struct k_work *work)
{
	int64_t now = k_uptime_get();
	if (now - last_beacon_in_range > 10000 && in_range)
	{
		in_range = false;
		gpio_pin_set_dt(&led_red, 0);
		// gpio_pin_set(dev, DETECT_PIN, 0);
	}
}

// Work definitions
K_WORK_DEFINE(blink_work, blink_work_handler);
K_WORK_DEFINE(check_beacon_timeout_work, check_beacon_timeout_work_handler);

// Timer definitions
void beacon_timeout_timer_handler(struct k_timer *dummy)
{
	k_work_submit(&check_beacon_timeout_work);
}

K_TIMER_DEFINE(beacon_timeout_timer, beacon_timeout_timer_handler, NULL);

void setup_usb()
{
	const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

	if (usb_enable(NULL))
	{
		return;
	}

	/* Poll if the DTR flag was set */
	while (!dtr)
	{
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}
}

void print_hex(uint8_t *data, size_t len)
{
	for (size_t i = 0; i < len; i++)
	{
		printk("%02x:", data[i]);
	}
	printk("\n");
}

void reverse(uint8_t *p_data, uint8_t len)
{
	uint8_t temp;
	for (uint32_t i = 0; i < len / 2; i++)
	{
		temp = p_data[i];
		p_data[i] = p_data[len - 1 - i];
		p_data[len - 1 - i] = temp;
	}
}

static void start_scan(void);

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

// iBeacons
uint8_t uuid[18] = {0x33, 0x1f, 0x8d, 0x59, 0xc2, 0xe5, 0x4a, 0xa4, 0xbe, 0x17, 0xe0, 0xa1, 0x66, 0x3f, 64, 00, 01, 00};

// IRKs
uint8_t irks[NUM_IRKS][16] = {
	{0x08, 0x64, 0xbc, 0x2b, 0x98, 0x22, 0x66, 0x9d, 0x0d, 0x3c, 0xdb, 0xf0, 0x84, 0xc3, 0x20, 0xe5},
	{0xe5, 0x20, 0xc3, 0x84, 0xf0, 0xdb, 0x3c, 0x0d, 0x9d, 0x66, 0x22, 0x98, 0x2b, 0xbc, 0x64, 0x08},
	{0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0xA7, 0xB8, 0xC9, 0xD1, 0xE2, 0xF3, 0xA4, 0xB5, 0xC6, 0xD7},
};

psa_key_id_t key_ids[NUM_IRKS];

void setup_key(uint8_t const *key, mbedtls_svc_key_id_t *key_id)
{
	psa_status_t status;
	psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
	psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
	psa_set_key_algorithm(&attributes, PSA_ALG_ECB_NO_PADDING);
	psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attributes, 16 * 8);
	psa_set_key_lifetime(&attributes, PSA_KEY_LIFETIME_VOLATILE);

	uint8_t ecb_key[16];
	for (uint32_t i = 0; i < 16; i++)
	{
		ecb_key[i] = key[16 - 1 - i];
	}

	status = psa_import_key(&attributes, ecb_key, 16, key_id);
	if (status != PSA_SUCCESS)
	{
		printk("PSA key import failed\n");
		return;
	}
}

// Function to perform ECB encryption
void encrypt_ecb(uint8_t *input, uint8_t *output, mbedtls_svc_key_id_t key_id)
{
	size_t ciphertext_size;
	psa_status_t status;

	status = psa_cipher_encrypt(key_id, PSA_ALG_ECB_NO_PADDING, input, 16, output, 16, &ciphertext_size);
	if (status != PSA_SUCCESS)
	{
		printk("PSA encryption failed\n");
		return;
	}
}

void ah(mbedtls_svc_key_id_t key_id, uint8_t *p_r, uint8_t *p_local_hash)
{
	// uint8_t ecb_key[16];
	uint8_t ecb_plaintext[16];
	uint8_t ecb_ciphertext[16];

	memset(ecb_plaintext, 0, 16 - 3);

	for (uint32_t i = 0; i < 3; i++)
	{
		ecb_plaintext[16 - 1 - i] = p_r[i];
	}

	encrypt_ecb(ecb_plaintext, ecb_ciphertext, key_id);

	for (uint32_t i = 0; i < 3; i++)
	{
		p_local_hash[i] = ecb_ciphertext[16 - 1 - i];
	}
}

bool resolve_address(uint8_t *p_addr)
{

	uint8_t hash[3];
	uint8_t local_hash[3];
	uint8_t prand[3];

	memcpy(hash, p_addr, 3);
	memcpy(prand, &p_addr[3], 3);

	for (int i = 0; i < NUM_IRKS; i++)
	{
		ah(key_ids[i], prand, local_hash);
		if (memcmp(hash, local_hash, 3) == 0)
		{
			printk("Match id: %d\n", i);
			return true;
		}
	}
	return false;
}

static bool eir_found(struct bt_data *data, void *user_data)
{
	// Controleer of het data type overeenkomt met het type dat we zoeken
	if (data->type != BT_DATA_MANUFACTURER_DATA)
	{
		return true;
	}

	struct scan_result *res = user_data;
	bt_addr_le_t *addr = res->addr;

	if (addr->type == BT_ADDR_LE_RANDOM_ID || addr->type == BT_ADDR_LE_RANDOM)
	{
		// printk("Resolving...: %02x:%02x:%02x:%02x:%02x:%02x\n", addr->a.val[0], addr->a.val[1], addr->a.val[2], addr->a.val[3], addr->a.val[4], addr->a.val[5]);

		if (resolve_address(addr->a.val))
		{
			// gpio_pin_set_dt(&led_red, 1);
			printk("Found resolved address: ");
			print_hex(addr->a.val, 6);

			// Blink led on beacon found
			k_work_submit(&blink_work);
			last_beacon = k_uptime_get();

			if (res->rssi > RSSI_1M_APPLE)
			{
				gpio_pin_set_dt(&led_blue, 1);
				gpio_pin_set_dt(&led_red, 1);
				last_beacon_in_range = k_uptime_get();
				in_range = true;
				printk("RSSI: %d\n", res->rssi);
			}
			else
			{
				gpio_pin_set_dt(&led_blue, 0);
			}

			return false;
		}
	}

	// Vergelijk de advertentiedata met de gewenste iBeacon UUID
	if (memcmp(&data->data[4], &uuid, sizeof(uuid) - 4) == 0)
	{
		// Blink led on beacon found
		k_work_submit(&blink_work);
		last_beacon = k_uptime_get();

		uint8_t rssi_1m = data->data[24];
		int16_t r = 0xFF00 + rssi_1m;

		// double distance = calculate_distance(res->rssi, rssi_1m);

		if (res->rssi > r)
		{
			gpio_pin_set_dt(&led_blue, 1);
			gpio_pin_set_dt(&led_red, 1);
			last_beacon_in_range = k_uptime_get();
			in_range = true;
			printk("Found iBeacon: %02x:%02x:%02x:%02x:%02x:%02x\n", addr->a.val[0], addr->a.val[1], addr->a.val[2], addr->a.val[3], addr->a.val[4], addr->a.val[5]);
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
		.options = BT_LE_SCAN_OPT_FILTER_ACCEPT_LIST,
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
	setup_usb();
	psa_crypto_init();

	for (int i = 0; i < NUM_IRKS; i++)
	{
		reverse(irks[i], 16);
		setup_key(irks[i], &key_ids[i]);
	}

	bt_enable(NULL);

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

	last_beacon = k_uptime_get();
	last_beacon_in_range = k_uptime_get();
	k_timer_start(&beacon_timeout_timer, K_SECONDS(1), K_SECONDS(1));

	printk("Device started successfully\n");
}