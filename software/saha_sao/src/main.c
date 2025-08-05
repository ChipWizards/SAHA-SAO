/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
//#include <zephyr/bluetooth/gatt.h>
//#include <zephyr/bluetooth/hci.h>
//#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/bluetooth/services/bas.h>

#include <zephyr/settings/settings.h>

#include <zephyr/kernel.h>
#include <stdlib.h>

#define STACKSIZE 1024
#define PRIORITY 7

#define RUN_LED_BLINK_INTERVAL 1000

//pled
#define I2C0_NODE DT_NODELABEL(leddriver)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);

// TIMER
#define TIME_TO_WAIT_MS 5000UL

#define HRS_QUEUE_SIZE 16

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
		(CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
		(CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_HRS_VAL)), /* Heart Rate Service */
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME)
};

static struct bt_conn *central_conn;
static struct k_work adv_work;
static struct k_timer my_timer;

// PUT THIS IN ANOTHER SRC
// register stuff
static const uint8_t REG_CNTRL1 = 0x00;
static const uint8_t REG_CNTRL2 = 0x01;
static const uint8_t REG_RATIO_MSB = 0x02;
static const uint8_t REG_RATIO_LSB = 0x03;
static const uint8_t REG_OUTPUT_ONOFF_MSB = 0x04;
static const uint8_t REG_OUTPUT_ONOFF_LSB = 0x05;

// Per LED control channels - fader channel assig, log dimming enable, temperature compensation
static const uint8_t REG_D1_CTRL = 0x06;
static const uint8_t REG_D2_CTRL = 0x07;
static const uint8_t REG_D3_CTRL = 0x08;
static const uint8_t REG_D4_CTRL = 0x09;
static const uint8_t REG_D5_CTRL = 0x0a;
static const uint8_t REG_D6_CTRL = 0x0b;
static const uint8_t REG_D7_CTRL = 0x0c;
static const uint8_t REG_D8_CTRL = 0x0d;
static const uint8_t REG_D9_CTRL = 0x0e;

// 0x0f to 0x15 reserved

// Direct PWM control registers
static const uint8_t REG_D1_PWM  = 0x16;
static const uint8_t REG_D2_PWM  = 0x17;
static const uint8_t REG_D3_PWM  = 0x18;
static const uint8_t REG_D4_PWM  = 0x19;
static const uint8_t REG_D5_PWM  = 0x1a;
static const uint8_t REG_D6_PWM  = 0x1b;
static const uint8_t REG_D7_PWM  = 0x1c;
static const uint8_t REG_D8_PWM  = 0x1d;
static const uint8_t REG_D9_PWM  = 0x1e;

// 0x1f to 0x25 reserved

// Drive current registers
static const uint8_t REG_D1_I_CTL = 0x26;
static const uint8_t REG_D2_I_CTL  = 0x27;
static const uint8_t REG_D3_I_CTL  = 0x28;
static const uint8_t REG_D4_I_CTL  = 0x29;
static const uint8_t REG_D5_I_CTL  = 0x2a;
static const uint8_t REG_D6_I_CTL  = 0x2b;
static const uint8_t REG_D7_I_CTL  = 0x2c;
static const uint8_t REG_D8_I_CTL  = 0x2d;
static const uint8_t REG_D9_I_CTL  = 0x2e;

// 0x2f to 0x35 reserved

static const uint8_t REG_MISC     = 0x36;
static const uint8_t REG_PC1      = 0x37;
static const uint8_t REG_PC2      = 0x38;
static const uint8_t REG_PC3      = 0x39;
static const uint8_t REG_STATUS_IRQ = 0x3A;
static const uint8_t REG_INT_GPIO   = 0x3B;
static const uint8_t REG_GLOBAL_VAR = 0x3C;
static const uint8_t REG_RESET      = 0x3D;
static const uint8_t REG_TEMP_CTL   = 0x3E;
static const uint8_t REG_TEMP_READ  = 0x3F;
static const uint8_t REG_TEMP_WRITE = 0x40;
static const uint8_t REG_TEST_CTL   = 0x41;
static const uint8_t REG_TEST_ADC   = 0x42;

// 0x43 to 0x44 reserved

static const uint8_t REG_ENGINE_A_VAR = 0x45;
static const uint8_t REG_ENGINE_B_VAR = 0x46;
static const uint8_t REG_ENGINE_C_VAR = 0x47;

static const uint8_t REG_MASTER_FADE_1 = 0x48;
static const uint8_t REG_MASTER_FADE_2 = 0x49;
static const uint8_t REG_MASTER_FADE_3 = 0x4A;

// 0x4b Reserved

static const uint8_t REG_PROG1_START = 0x4C;
static const uint8_t REG_PROG2_START = 0x4D;
static const uint8_t REG_PROG3_START = 0x4E;
static const uint8_t REG_PROG_PAGE_SEL = 0x4f;

// Memory is more confusing - there are 6 pages, sel by addr 4f
static const uint8_t REG_PROG_MEM_BASE = 0x50;
//static const uint8_t REG_PROG_MEM_SIZE = 0x;//
static const uint8_t REG_PROG_MEM_END  = 0x6f;

static const uint8_t REG_ENG1_MAP_MSB = 0x70;
static const uint8_t REG_ENG1_MAP_LSB = 0x71;
static const uint8_t REG_ENG2_MAP_MSB = 0x72;
static const uint8_t REG_ENG2_MAP_LSB = 0x73;
static const uint8_t REG_ENG3_MAP_MSB = 0x74;
static const uint8_t REG_ENG3_MAP_LSB = 0x75;

static const uint8_t REG_GAIN_CHANGE = 0x76;
// PUT THIS IN ANOTHER SRC 

static const uint16_t led_program[] =
{
	0x001E,
	0x0002,
	0x0010,
	0x0004,
	0x0008,
	0x0012,
	0x000C,
	0x9C00,
	0x9C86,
	0x16BE,
	0x7E00,
	0x17BE,
	0x7E00,
	0x9D80,
	0xA002
};

//Should reposition the whole led library here, or move it into its own file.
static void pled_write_byte(uint8_t addr, uint8_t data)
{
	uint8_t config[2] = {addr,data};
	int ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
	if(ret != 0){
		printk("Failed to write to I2C device address %x at reg. %x \n\r", dev_i2c.addr,addr);
	}
}

static int8_t calculate_light_cycle(int8_t rssi) 
{
	int8_t level = rssi;
	level = abs(level) / 6;

	if (level > 13) {
		//return 1;
		return 30;
	}
	if (level < 11) {
		//return 5;
		return 190;
	}

	switch (level) {
		case 13:
			//return 2;
			return 70;
		case 12:
			//return 3;
			return 110;
		case 11:
			//return 4;
			return 150;
	}
	// anywhere from -49 to -90
	// I want -59 and up to be 100%, -80 and below 20%
	return -1;
}

static int scan_start(void)
{
	int err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);

	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
	}

	return err;
}

static void adv_work_handler(struct k_work *work)
{
	// If you want to change how often the SAO advertises create a custom BT_LE_ADV_PARAM and insert it here
	int err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	printk("Filters matched. Address: %s connectable: %d\n",
		   addr, connectable);
	printk("RSSI is %d\n", device_info->recv_info->rssi);
	printk("Calculated power level is %d\n", calculate_light_cycle(device_info->recv_info->rssi));
	pled_write_byte(REG_D1_PWM,calculate_light_cycle(device_info->recv_info->rssi)); // d1 duty cycle
	k_timer_start(&my_timer, K_MSEC(TIME_TO_WAIT_MS), K_FOREVER);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	printk("Connecting failed\n");
}

static void scan_connecting(struct bt_scan_device_info *device_info,
			    struct bt_conn *conn)
{
	central_conn = bt_conn_ref(conn);
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL,
		scan_connecting_error, scan_connecting);

static void scan_init(void)
{
	int err;

	struct bt_scan_init_param param = {
		.scan_param = NULL, // If you want to change how often the SAO scans create a custom BT_LE_SCAN_PARAM
		.conn_param = BT_LE_CONN_PARAM_DEFAULT,
		.connect_if_match = 0
	};

	bt_scan_init(&param);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_HRS);
	if (err) {
		printk("Scanning filters cannot be set (err %d)\n", err);
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		printk("Filters cannot be turned on (err %d)\n", err);
	}
}

static void timer_handler(struct k_timer *dummy)
{
    printk("Timer finished.\n");
	pled_write_byte(REG_D1_PWM,0x00); // d1 duty cycle
}

static uint8_t pled_read_byte(uint8_t addr)
{
	uint8_t val;
	int ret;

	ret = i2c_reg_read_byte_dt(&dev_i2c, addr, &val);
	if (ret != 0) {
		printk("Failed to read i2c register");
	}
	return val;
}

static int pled_load_prgm(const uint16_t* prog, uint8_t len)
{
	// ALMOST ALL THIS CODE IS MODELED AFTER https://github.com/sparkfun/SparkFun_LP55231_Arduino_Library/blob/master/src/lp55231.cpp#L306

	/*
	Every Instruction is 16-bit width.
	The LP55231 can store 96 instructions. Each
	instruction consists of 16 bits. Because one
	register has only 8 bits, one instruction requires
	two register addresses. In order to reduce
	program load time the LP55231 supports address
	auto-incrementation. Register address is
	incremented after each 8 data bits. Thus the
	whole program memory page can be written in
	one serial bus write sequence.

	^ I can optimize these load times if I have time down the road.
	*/

	uint8_t val;
	uint8_t page;

	// max instr 96
	if (len >= 96)
	{
	return -1;
	}

	pled_write_byte(REG_CNTRL2, 0x00); // Disable engines
	pled_write_byte(REG_CNTRL2, 0x15); // Set all engines to load mode

	// Whole pages, my tester is 12 instr so..
	for(page = 0; page < (len/16); page++)
	{
		pled_write_byte(REG_PROG_PAGE_SEL, page);

		for(uint8_t i = 0; i < 16; i++)
		{
			uint8_t config[3] = {REG_PROG_MEM_BASE + (i*2),
								(prog[(i + (page*16))]>> 8) & 0xff,
								prog[i + (page*16)] & 0xff};
			i2c_write_dt(&dev_i2c, config, sizeof(config));
		}
	}

	page = len/16;
	pled_write_byte(REG_PROG_PAGE_SEL, page);
	for(uint8_t i = 0; i < (len%16); i++) 
	{
		uint8_t config[3] = {REG_PROG_MEM_BASE + (i*2),
							(prog[(i + (page*16))]>> 8) & 0xff,
							prog[i + (page*16)] & 0xff};
		i2c_write_dt(&dev_i2c, config, sizeof(config));
	}

	pled_write_byte(REG_CNTRL2, 0x00); // Disable engines
	return 0;
}

static int set_engine_entry(uint8_t engine, uint8_t addr)
{
	// sanity check engine #
	pled_write_byte(REG_PROG1_START + engine, addr);

	return 0;
}

static int set_engine_PC(uint8_t engine, uint8_t addr)
{
	uint8_t control1_val, control2_val, temp;
	
	// Sanity check engine #
	// Sanity check PC #

	control1_val = pled_read_byte(REG_CNTRL1);
	control2_val = pled_read_byte(REG_CNTRL2);

	temp = (control1_val & ~(0x30 >> (engine * 2)));

	pled_write_byte(REG_CNTRL2, 0xff); // halt engines , code might be dumb? TODO
	pled_write_byte(REG_CNTRL1, temp); // engine into load mode

	pled_write_byte(REG_PC1 + engine, addr);

	//restore prev
	pled_write_byte(REG_CNTRL1, control1_val);
	pled_write_byte(REG_CNTRL2, control2_val);

	return 0;
}

static int set_engine_free(uint8_t engine)
{
	uint8_t val;

	//sanity check engine # here

	val = pled_read_byte(REG_CNTRL1);
	val &= ~(0x30 >> (engine * 2));
	val |= (0x20>> (engine * 2));
	pled_write_byte(REG_CNTRL1, val);

	return 0;
}

static int set_engine_running(uint8_t engine)
{
	uint8_t val;

	//sanity check engine # here

	val = pled_read_byte(REG_CNTRL2);
	val &= ~(0x30 >> (engine * 2));
	val |= (0x20>> (engine * 2));
	pled_write_byte(REG_CNTRL2, val);

	return 0;
}

static void pled_init(void)
{	
	uint8_t test;
	
	pled_write_byte(REG_RESET,0xFF); // RESET
	pled_write_byte(REG_CNTRL1,0x40); // Chip enable
	
	pled_write_byte(REG_MISC,0x7b); // was 0x53 for forced to 1.5x, now 0x5B for automatic selection, 0x43 for charge pump off, 7b
	pled_write_byte(REG_D1_CTRL,0x20); // logarithmic dimming for d1
	pled_write_byte(REG_D2_CTRL,0x20); // logarithmic dimming for d2
	pled_write_byte(REG_D3_CTRL,0x20); // logarithmic dimming for d3
	pled_write_byte(REG_D4_CTRL,0x20); // logarithmic dimming for d4
	pled_write_byte(REG_D5_CTRL,0x20); // logarithmic dimming for d5
	pled_write_byte(REG_OUTPUT_ONOFF_LSB,0x1F); // was 0x03, enable d1,d2,d3,d4,d5
	//pled_write_byte(REG_D1_PWM,0x7F); // d1 duty cycle
	//pled_write_byte(REG_D2_PWM,0xbf); // set d2 duty cycle
	//pled_write_byte(REG_D3_PWM,0xbf); // set d3 duty cycle
	//pled_write_byte(REG_D4_PWM,0xbf); // set d4 duty cycle
	//pled_write_byte(REG_D5_PWM,0xbf); // set d5 duty cycle
	

	test = pled_read_byte(REG_MISC);
	printk("REG_MISC = %d\n", test);
	
	
	int ret = pled_load_prgm(led_program, sizeof(led_program));
	if(ret != 0) {
		printk("Failed to upload led driver program");
	}
	
	// Both of these need to point to the first valid instruction in the led driver program
	set_engine_entry(0,7);
  	set_engine_PC(0,7);

	set_engine_free(0);
	set_engine_running(0);
}

int main(void)
{
	int err;

	printk("Starting SAHA SAO V2!\n");

	err = bt_enable(NULL);
	if (err) {
		return 0;
	}

	if (!device_is_ready(dev_i2c.bus)) {
 		printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
 		return 0;
	}
	
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	settings_load();
	scan_init();

	err = scan_start();
	if (err) {
		return 0;
	}

	printk("Scanning started\n");

	pled_init();

	k_work_init(&adv_work, adv_work_handler);
	advertising_start();

	k_timer_init(&my_timer, timer_handler, NULL);

	for (;;) {
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL)); // Can I put this to sleep forever instead? Will the BLE interrupts awaken it?
	}

	return 0;
}
