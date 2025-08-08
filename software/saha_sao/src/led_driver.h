#include <stdlib.h>

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
