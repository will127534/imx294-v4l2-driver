// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony imx294 cameras.
 *
 * Based on Sony imx477 camera driver
 * Copyright (C) 2019-2020 Raspberry Pi (Trading) Ltd
 */
#include <asm/unaligned.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <linux/moduleparam.h>



int debug = 0;
module_param(debug, int, 0660);
MODULE_PARM_DESC(debug, "Debug flag");

#define DEBUG_PRINTK(fmt, ...) do { if (debug) printk(KERN_DEBUG "%s: " fmt, __this_module.name, ##__VA_ARGS__); } while(0)


/* Chip ID */
#define IMX294_REG_CHIP_ID		0x3000
#define IMX294_CHIP_ID			0x0000

#define IMX294_REG_MODE_SELECT		0x3000
#define IMX294_MODE_STANDBY		0x01
#define IMX294_MODE_STREAMING		0x00

#define IMX294_XCLK_FREQ		24000000

/* VMAX internal VBLANK*/
#define IMX294_REG_VMAX		0x30A9
#define IMX294_VMAX_MAX		0xfffff

/* HMAX internal HBLANK*/
#define IMX294_REG_HMAX		0x30AC
#define IMX294_HMAX_MAX		0xffff

#define IMX294_REG_HCOUNT1     0x3084
#define IMX294_REG_HCOUNT2     0x3086
#define IMX294_REG_PSSLVS1 0x332C
#define IMX294_REG_PSSLVS2 0x334A
#define IMX294_REG_PSSLVS3 0x35B6
#define IMX294_REG_PSSLVS4 0x35B8
#define IMX294_REG_PSSLVS0 0x36BC


/* SHR internal */
#define IMX294_REG_SHR		0x302C
#define IMX294_SHR_MIN		11

/* Exposure control */
#define IMX294_EXPOSURE_MIN			52
#define IMX294_EXPOSURE_STEP		1
#define IMX294_EXPOSURE_DEFAULT		1000
#define IMX294_EXPOSURE_MAX		49865

/* Analog gain control */
#define IMX294_REG_ANALOG_GAIN		0x300A
#define IMX294_ANA_GAIN_MIN		0
#define IMX294_ANA_GAIN_MAX		1957
#define IMX294_ANA_GAIN_STEP		1
#define IMX294_ANA_GAIN_DEFAULT		0x0

/* Embedded metadata stream structure */
#define IMX294_EMBEDDED_LINE_WIDTH 16384
#define IMX294_NUM_EMBEDDED_LINES 1

enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

/* imx294 native and active pixel array size. */
#define IMX294_NATIVE_WIDTH		3792U
#define IMX294_NATIVE_HEIGHT		2840U
#define IMX294_PIXEL_ARRAY_LEFT	40U
#define IMX294_PIXEL_ARRAY_TOP		26U
#define IMX294_PIXEL_ARRAY_WIDTH	3840U
#define IMX294_PIXEL_ARRAY_HEIGHT	2160U

struct imx294_reg {
	u16 address;
	u8 val;
};

struct IMX294_reg_list {
	unsigned int num_of_regs;
	const struct imx294_reg *regs;
};

/* Mode : resolution and related config&values */
struct imx294_mode {
	/* Frame width */
	unsigned int width;

	/* Frame height */
	unsigned int height;

	/* minimum H-timing */
	uint64_t min_HMAX;

	/* minimum V-timing */
	uint64_t min_VMAX;

	/* default H-timing */
	uint64_t default_HMAX;

	/* default V-timing */
	uint64_t default_VMAX;

    /* V-timing Scaling*/
    uint64_t VMAX_scale;

	/* minimum SHR */
	uint64_t min_SHR;

    unsigned int integration_offset;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Default register values */
	struct IMX294_reg_list reg_list;
};

static const struct imx294_reg mode_common_regs[] = {

    {0x3033,0x30},
    {0x303C,0x01},

    {0x31E8,0x20}, //PLRD1
    {0x31E9,0x01},

    {0x3122,0x02}, //PLRD2
    {0x3129,0x90}, //PLRD3
    {0x312A,0x02}, //PLRD4

    {0x311F,0x00}, //PLRD10
    {0x3123,0x00}, //PLRD11
    {0x3124,0x00}, //PLRD12
    {0x3125,0x01}, //PLRD13
    {0x3127,0x02}, //PLRD14
    {0x312D,0x02}, //PLRD15

    {0x3000,0x12}, //STANDBY = 0 STBLOGIC register = 1h, STBMIPI register = 0h, STBDV register = 1h
    {0x310B,0x00}, //PLL release

    {0x3047,0x01}, //PLSTMG11
    {0x304E,0x0B}, //PLSTMG12
    {0x304F,0x24}, //PLSTMG13
    {0x3062,0x25}, //PLSTMG14
    {0x3064,0x78}, //PLSTMG15
    {0x3065,0x33}, //PLSTMG16
    {0x3067,0x71}, //PLSTMG17
    {0x3088,0x75}, //PLSTMG18
    {0x308A,0x09}, //PLSTMG19
    {0x308B,0x01}, //PLSTMG19
    {0x308C,0x61}, //PLSTMG20
    {0x3146,0x00}, //PLSTMG10
    {0x3234,0x32}, //PLSTMG21
    {0x3235,0x00}, //PLSTMG21
    {0x3248,0xBC}, //PLSTMG22
    {0x3249,0x00}, //PLSTMG22
    {0x3250,0xBC}, //PLSTMG23
    {0x3251,0x00}, //PLSTMG23
    {0x3258,0xBC}, //PLSTMG24
    {0x3259,0x00}, //PLSTMG24
    {0x3260,0xBC}, //PLSTMG25
    {0x3261,0x00}, //PLSTMG25
    {0x3274,0x13}, //PLSTMG26
    {0x3275,0x00}, //PLSTMG26
    {0x3276,0x1F}, //PLSTMG27
    {0x3277,0x00}, //PLSTMG27
    {0x3278,0x30}, //PLSTMG28
    {0x3279,0x00}, //PLSTMG28
    {0x327C,0x13}, //PLSTMG29
    {0x327D,0x00}, //PLSTMG29
    {0x327E,0x1F}, //PLSTMG30
    {0x327F,0x00}, //PLSTMG30
    {0x3280,0x30}, //PLSTMG31
    {0x3281,0x00}, //PLSTMG31
    {0x3284,0x13}, //PLSTMG32
    {0x3285,0x00}, //PLSTMG32
    {0x3286,0x1F}, //PLSTMG33
    {0x3287,0x00}, //PLSTMG33
    {0x3288,0x30}, //PLSTMG34
    {0x3289,0x00}, //PLSTMG34
    {0x328C,0x13}, //PLSTMG35
    {0x328D,0x00}, //PLSTMG35
    {0x328E,0x1F}, //PLSTMG36
    {0x328F,0x00}, //PLSTMG36
    {0x3290,0x30}, //PLSTMG37
    {0x3291,0x00}, //PLSTMG37
    {0x32AE,0x00}, //PLSTMG38
    {0x32AF,0x00}, //PLSTMG39
    {0x32CA,0x5A}, //PLSTMG40
    {0x32CB,0x00}, //PLSTMG40
    {0x332F,0x00}, //PLSTMG41
    {0x334C,0x01}, //PLSTMG09
    {0x335A,0x79}, //PLSTMG43
    {0x335B,0x00}, //PLSTMG43
    {0x335E,0x56}, //PLSTMG44
    {0x335F,0x00}, //PLSTMG44
    {0x3360,0x6A}, //PLSTMG45
    {0x3361,0x00}, //PLSTMG45
    {0x336A,0x56}, //PLSTMG46
    {0x336B,0x00}, //PLSTMG46
    {0x33D6,0x79}, //PLSTMG47
    {0x33D7,0x00}, //PLSTMG47
    {0x340C,0x6E}, //PLSTMG48
    {0x340D,0x00}, //PLSTMG48
    {0x3448,0x7E}, //PLSTMG49
    {0x3449,0x00}, //PLSTMG49
    {0x348E,0x6F}, //PLSTMG50
    {0x348F,0x00}, //PLSTMG50
    {0x3492,0x11}, //PLSTMG51
    {0x34C4,0x5A}, //PLSTMG52
    {0x34C5,0x00}, //PLSTMG52
    {0x3506,0x56}, //PLSTMG53
    {0x3507,0x00}, //PLSTMG53
    {0x350C,0x56}, //PLSTMG54
    {0x350D,0x00}, //PLSTMG54
    {0x350E,0x58}, //PLSTMG55
    {0x350F,0x00}, //PLSTMG55
    {0x3549,0x04}, //PLSTMG56
    {0x355D,0x03}, //PLSTMG57
    {0x355E,0x03}, //PLSTMG58
    {0x3574,0x56}, //PLSTMG59
    {0x3575,0x00}, //PLSTMG59
    {0x3587,0x01}, //PLSTMG60
    {0x35D0,0x5E}, //PLSTMG61
    {0x35D1,0x00}, //PLSTMG61
    {0x35D4,0x63}, //PLSTMG62
    {0x35D5,0x00}, //PLSTMG62
    {0x366A,0x1A}, //PLSTMG63
    {0x366B,0x16}, //PLSTMG64
    {0x366C,0x10}, //PLSTMG65
    {0x366D,0x09}, //PLSTMG66
    {0x366E,0x00}, //PLSTMG67
    {0x366F,0x00}, //PLSTMG68
    {0x3670,0x00}, //PLSTMG69
    {0x3671,0x00}, //PLSTMG70
    {0x3676,0x83}, //PLSTMG73
    {0x3677,0x03}, //PLSTMG73
    {0x3678,0x00}, //PLSTMG74
    {0x3679,0x04}, //PLSTMG74
    {0x367A,0x2C}, //PLSTMG75
    {0x367B,0x05}, //PLSTMG75
    {0x367C,0x00}, //PLSTMG76
    {0x367D,0x06}, //PLSTMG76
    {0x367E,0x00}, //PLSTMG77
    {0x367F,0x07}, //PLSTMG77
    {0x3680,0x4B}, //PLSTMG78
    {0x3681,0x07}, //PLSTMG78
    {0x3690,0x27}, //PLSTMG79
    {0x3691,0x00}, //PLSTMG79
    {0x3692,0x65}, //PLSTMG80
    {0x3693,0x00}, //PLSTMG80
    {0x3694,0x4F}, //PLSTMG81
    {0x3695,0x00}, //PLSTMG81
    {0x3696,0xA1}, //PLSTMG82
    {0x3697,0x00}, //PLSTMG82
    {0x382B,0x68}, //PLSTMG83
    {0x3C00,0x01}, //PLSTMG84
    {0x3C01,0x01}, //PLSTMG85
    {0x3686,0x00}, //PLSTMG101
    {0x3687,0x00}, //PLSTMG101
    {0x36BE,0x01}, //PLSTMG102
    {0x36BF,0x00}, //PLSTMG102
    {0x36C0,0x01}, //PLSTMG103
    {0x36C1,0x00}, //PLSTMG103
    {0x36C2,0x01}, //PLSTMG104
    {0x36C3,0x00}, //PLSTMG104
    {0x36C4,0x01}, //PLSTMG105
    {0x36C5,0x01}, //PLSTMG106
    {0x36C6,0x01}, //PLSTMG107


    {0x3134,0xAF}, //tclkpost
    {0x3135,0x00},
    {0x3136,0xC7}, //thszero
    {0x3137,0x00},
    {0x3138,0x7F}, //thsprepare
    {0x3139,0x00},
    {0x313A,0x6F}, //tclktrail
    {0x313B,0x00},
    {0x313C,0x6F}, //thstrail
    {0x313D,0x00},
    {0x313E,0xCF}, //tclkzero
    {0x313F,0x01},
    {0x3140,0x77}, //tclkprepare
    {0x3141,0x00},
    {0x3142,0x5F}, //tlpx
    {0x3143,0x00},

    {0x3004,0x1A}, //MDSEL1 
    {0x3005,0x06}, //MDSEL2 
    {0x3006,0x00}, //MDSEL3 
    {0x3007,0xA0}, //MDSEL4 
    {0x3019,0x00}, //MDVREV 
    {0x3030,0x77}, //MDSEL5 
    {0x3034,0x00}, //HOPBOUT_EN 
    {0x3035,0x01}, //HTRIMMING_EN 
    {0x3036,0x30}, //HTRIMMING_START 
    {0x3037,0x00}, //HTRIMMING_START 
    {0x3038,0x60}, //HTRIMMING_END 
    {0x3039,0x10}, //HTRIMMING_END 
    {0x3068,0x1A}, //MDSEL15 
    {0x3069,0x00}, //MDSEL15 
    {0x3080,0x00}, //MDSEL6 
    {0x3081,0x01}, //MDSEL7 
    {0x30A8,0x02}, //MDSEL8 
    {0x30E2,0x00}, //VCUTMODE 
    {0x312F,0x08}, //OPB_SIZE_V 
    {0x3130,0x88}, //WRITE_VSIZE 
    {0x3131,0x08}, //WRITE_VSIZE 
    {0x3132,0x80}, //OUT_SIZE 
    {0x3133,0x08}, //Y_OUT_SIZE 
    {0x357F,0x0C}, //MDSEL11 
    {0x3580,0x0A}, //MDSEL12 
    {0x3581,0x08}, //MDSEL13 
    {0x3583,0x72}, //MDSEL14 
    {0x3600,0x90}, //MDSEL16 
    {0x3601,0x00}, //MDSEL16 
    {0x3846,0x00}, //MDSEL9 
    {0x3847,0x00}, //MDSEL9 
    {0x384A,0x00}, //MDSEL10 
    {0x384B,0x00}, //MDSEL10



    //SVR = 0
    {0x300E,0x00},
    {0x300F,0x00},

    //SHR = 100
    {0x302C,0x10},
    {0x302D,0x00},

    //VMAX = 5000
    {0x30A9,0x88},
    {0x30AA,0x13},
    {0x30AB,0x00},

    //HMAX = 1200
    {0x30AC,0xB0},
    {0x30AD,0x04},
    //HCOUNT1 Set the same value as HMAX 
    {0x3084,0xB0},
    {0x3085,0x04},
    //HCOUNT2 Set the same value as HMAX 
    {0x3086,0xB0},
    {0x3087,0x04},

    {0x332C,0x00}, //PSSLVS1 = VBLK = VMAX × (SVR value + 1) - minimum VMAX setting = VMAX - minimum VMAX
    {0x332D,0x00}, //
    {0x334A,0x00}, //PSSLVS2 = VBLK 
    {0x334B,0x00}, //
    {0x35B6,0x00}, //PSSLVS3 = VBLK
    {0x35B7,0x00}, //
    {0x35B8,0x00}, //PSSLVS4 = VBLK - 5 
    {0x35B9,0x00}, //
    {0x36BC,0x00}, //PSSLVS0 = VBLK
    {0x36BD,0x00}, //

    //delay 10ms
    {0xFFFE,0x0A},

    {0x3000,0x02}, //(STANDBY register = 0h, STBLOGIC register = 1h, STBMIPI register = 0h, STBDV register = 0h
    {0x35E5,0x92},
    {0x35E5,0x9A},
    {0x3000,0x00}, //(STANDBY register = 0h, STBLOGIC register = 0h, STBMIPI register = 0h, STBDV register = 0h

    
    //delay 10ms
    {0xFFFE,0x0A},

    {0x3033,0x20},
    {0x3017,0xA8},
};


/* 3704 x 2778 readout mode 0 - 12bit */
static const struct imx294_reg mode_00_regs[] = {
    {0x3004,0x00}, //MDSEL1 
    {0x3005,0x06}, //MDSEL2 
    {0x3006,0x02}, //MDSEL3 
    {0x3007,0xA0}, //MDSEL4 
    {0x3019,0x00}, //MDVREV 
    {0x3030,0x77}, //MDSEL5 
    {0x3034,0x00}, //HOPBOUT_EN 
    {0x3035,0x01}, //HTRIMMING_EN 
    {0x3036,0x30}, //HTRIMMING_START 
    {0x3037,0x00}, //HTRIMMING_START 
    {0x3038,0x00}, //HTRIMMING_END 
    {0x3039,0x0F}, //HTRIMMING_END 
    {0x3068,0x1A}, //MDSEL15 
    {0x3069,0x00}, //MDSEL15 
    {0x3080,0x00}, //MDSEL6 
    {0x3081,0x01}, //MDSEL7 
    {0x30A8,0x02}, //MDSEL8 
    {0x30E2,0x00}, //VCUTMODE 
    {0x312F,0x10}, //OPB_SIZE_V 
    {0x3130,0x18}, //WRITE_VSIZE 
    {0x3131,0x0B}, //WRITE_VSIZE 
    {0x3132,0x08}, //OUT_SIZE 
    {0x3133,0x0B}, //Y_OUT_SIZE 
    {0x357F,0x0C}, //MDSEL11 
    {0x3580,0x0A}, //MDSEL12 
    {0x3581,0x08}, //MDSEL13 
    {0x3583,0x72}, //MDSEL14 
    {0x3600,0x90}, //MDSEL16 
    {0x3601,0x00}, //MDSEL16 
    {0x3846,0x00}, //MDSEL9 
    {0x3847,0x00}, //MDSEL9 
    {0x384A,0x00}, //MDSEL10 
    {0x384B,0x00}, //MDSEL10
};


/* 4096 x 2160 readout mode 1 */
static const struct imx294_reg mode_01_regs[] = {
    {0x3004,0x1A}, //MDSEL1 
    {0x3005,0x06}, //MDSEL2 
    {0x3006,0x00}, //MDSEL3 
    {0x3007,0xA0}, //MDSEL4 
    {0x3019,0x00}, //MDVREV 
    {0x3030,0x77}, //MDSEL5 
    {0x3034,0x00}, //HOPBOUT_EN 
    {0x3035,0x01}, //HTRIMMING_EN 
    {0x3036,0x30}, //HTRIMMING_START 
    {0x3037,0x00}, //HTRIMMING_START 
    {0x3038,0x60}, //HTRIMMING_END 
    {0x3039,0x10}, //HTRIMMING_END 
    {0x3068,0x1A}, //MDSEL15 
    {0x3069,0x00}, //MDSEL15 
    {0x3080,0x00}, //MDSEL6 
    {0x3081,0x01}, //MDSEL7 
    {0x30A8,0x02}, //MDSEL8 
    {0x30E2,0x00}, //VCUTMODE 
    {0x312F,0x08}, //OPB_SIZE_V 
    {0x3130,0x88}, //WRITE_VSIZE 
    {0x3131,0x08}, //WRITE_VSIZE 
    {0x3132,0x80}, //OUT_SIZE 
    {0x3133,0x08}, //Y_OUT_SIZE 
    {0x357F,0x0C}, //MDSEL11 
    {0x3580,0x0A}, //MDSEL12 
    {0x3581,0x08}, //MDSEL13 
    {0x3583,0x72}, //MDSEL14 
    {0x3600,0x90}, //MDSEL16 
    {0x3601,0x00}, //MDSEL16 
    {0x3846,0x00}, //MDSEL9 
    {0x3847,0x00}, //MDSEL9 
    {0x384A,0x00}, //MDSEL10 
    {0x384B,0x00}, //MDSEL10
};

/* 4096 x 2160 low noise readout mode 1A */
static const struct imx294_reg mode_01A_regs[] = {
    {0x3004,0x01}, //MDSEL1 
    {0x3005,0x06}, //MDSEL2 
    {0x3006,0x00}, //MDSEL3 
    {0x3007,0xA0}, //MDSEL4 
    {0x3019,0x00}, //MDVREV 
    {0x3030,0x77}, //MDSEL5 
    {0x3034,0x00}, //HOPBOUT_EN 
    {0x3035,0x01}, //HTRIMMING_EN 
    {0x3036,0x30}, //HTRIMMING_START 
    {0x3037,0x00}, //HTRIMMING_START 
    {0x3038,0x80}, //HTRIMMING_END 
    {0x3039,0x10}, //HTRIMMING_END 
    {0x3068,0x1A}, //MDSEL15 
    {0x3069,0x00}, //MDSEL15 
    {0x3080,0x01}, //MDSEL6 
    {0x3081,0x01}, //MDSEL7 
    {0x30A8,0x02}, //MDSEL8 
    {0x30E2,0x00}, //VCUTMODE 
    {0x312F,0x08}, //OPB_SIZE_V 
    {0x3130,0x88}, //WRITE_VSIZE 
    {0x3131,0x08}, //WRITE_VSIZE 
    {0x3132,0x80}, //OUT_SIZE 
    {0x3133,0x08}, //Y_OUT_SIZE 
    {0x357F,0x0C}, //MDSEL11 
    {0x3580,0x0A}, //MDSEL12 
    {0x3581,0x08}, //MDSEL13 
    {0x3583,0x72}, //MDSEL14 
    {0x3600,0x7D}, //MDSEL16 
    {0x3601,0x00}, //MDSEL16 
    {0x3846,0x00}, //MDSEL9 
    {0x3847,0x00}, //MDSEL9 
    {0x384A,0x00}, //MDSEL10 
    {0x384B,0x00}, //MDSEL10
};

/* 3840 x 2160 readout mode 1B */
static const struct imx294_reg mode_01B_regs[] = {
    {0x3004,0x02}, //MDSEL1 
    {0x3005,0x06}, //MDSEL2 
    {0x3006,0x01}, //MDSEL3 
    {0x3007,0xA0}, //MDSEL4 
    {0x3019,0x00}, //MDVREV 
    {0x3030,0x77}, //MDSEL5 
    {0x3034,0x00}, //HOPBOUT_EN 
    {0x3035,0x01}, //HTRIMMING_EN 
    {0x3036,0x30}, //HTRIMMING_START 
    {0x3037,0x00}, //HTRIMMING_START 
    {0x3038,0x50}, //HTRIMMING_END 
    {0x3039,0x0F}, //HTRIMMING_END 
    {0x3068,0x1A}, //MDSEL15 
    {0x3069,0x00}, //MDSEL15 
    {0x3080,0x00}, //MDSEL6 
    {0x3081,0x01}, //MDSEL7 
    {0x30A8,0x02}, //MDSEL8 
    {0x30E2,0x00}, //VCUTMODE 
    {0x312F,0x08}, //OPB_SIZE_V 
    {0x3130,0x88}, //WRITE_VSIZE 
    {0x3131,0x08}, //WRITE_VSIZE 
    {0x3132,0x80}, //OUT_SIZE 
    {0x3133,0x08}, //Y_OUT_SIZE 
    {0x357F,0x0C}, //MDSEL11 
    {0x3580,0x0A}, //MDSEL12 
    {0x3581,0x08}, //MDSEL13 
    {0x3583,0x72}, //MDSEL14 
    {0x3600,0x90}, //MDSEL16 
    {0x3601,0x00}, //MDSEL16 
    {0x3846,0x00}, //MDSEL9 
    {0x3847,0x00}, //MDSEL9 
    {0x384A,0x00}, //MDSEL10 
    {0x384B,0x00}, //MDSEL10
};

/* Mode configs */
static const struct imx294_mode supported_modes_12bit[] = {
	{
		/* 4096 x 2160 readout mode 1 */
		.width = 4144,
		.height = 2184,
		.min_HMAX = 1122,
		.min_VMAX = 1111,
		.default_HMAX = 1200,
		.default_VMAX = 2500, //24 FPS
        .VMAX_scale = 2,
		.min_SHR = 5,
        .integration_offset = 256,
		.crop = {
			.left = 36,
			.top = 20,
			.width = 4096,
			.height = 2160,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_01_regs),
			.regs = mode_01_regs,
		},
	},
    {
        /* 4096 x 2160 low noise readout mode 1A */
        .width = 4176,
        .height = 2184,
        .min_HMAX = 1192,
        .min_VMAX = 1111,
        .default_HMAX = 1200,
        .default_VMAX = 2500, //24 FPS
        .VMAX_scale = 2,
        .min_SHR = 5,
        .integration_offset = 361,
        .crop = {
            .left = 36,
            .top = 20,
            .width = 4096,
            .height = 2160,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_01A_regs),
            .regs = mode_01A_regs,
        },
    },
    {
        /* 3840 x 2160 low noise readout mode 1B */
        .width = 3872,
        .height = 2180,
        .min_HMAX = 1055,
        .min_VMAX = 1111,
        .default_HMAX = 1200,
        .default_VMAX = 2500, //50 FPS
        .VMAX_scale = 2,
        .min_SHR = 5,
        .integration_offset = 256,
        .crop = {
            .left = 20,
            .top = 20,
            .width = 3840,
            .height = 2160,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_01B_regs),
            .regs = mode_01B_regs,
        },
    },
    {
        /* 3740 x 2778 readout mode 0 */
        .width = 3792,
        .height = 2840,
        .min_HMAX = 1024,
        .min_VMAX = 1444,
        .default_HMAX = 1875,
        .default_VMAX = 1600, //24 FPS
        .VMAX_scale = 2,
        .min_SHR = 5,
        .integration_offset = 551,
        .crop = {
            .left = 40,
            .top = 24,
            .width = 3704,
            .height = 2778,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_00_regs),
            .regs = mode_00_regs,
        },
    },
};


/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 codes[] = {
	/* 12-bit modes. */
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,

};

/* regulator supplies */
static const char * const imx294_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.05V) supply */
	"VDDL",  /* IF (1.8V) supply */
};

#define imx294_NUM_SUPPLIES ARRAY_SIZE(imx294_supply_name)

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby), given by T7 in the
 * datasheet is 8ms.  This does include I2C setup time as well.
 *
 * Note, that delay between XCLR low->high and reading the CCI ID register (T6
 * in the datasheet) is much smaller - 600us.
 */
#define imx294_XCLR_MIN_DELAY_US	100000
#define imx294_XCLR_DELAY_RANGE_US	1000

struct imx294_compatible_data {
	unsigned int chip_id;
	struct IMX294_reg_list extra_regs;
};

struct imx294 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	unsigned int fmt_code;

	struct clk *xclk;
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[imx294_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct imx294_mode *mode;

	uint16_t HMAX;
	uint32_t VMAX;
	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	/* Rewrite common registers on stream on? */
	bool common_regs_written;

	/* Any extra information related to different compatible sensors */
	const struct imx294_compatible_data *compatible_data;
};

static inline struct imx294 *to_imx294(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx294, sd);
}

static inline void get_mode_table(unsigned int code,
				  const struct imx294_mode **mode_list,
				  unsigned int *num_modes)
{
	switch (code) {
	/* 12-bit */
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		*mode_list = supported_modes_12bit;
		*num_modes = ARRAY_SIZE(supported_modes_12bit);
		break;
	default:
		*mode_list = NULL;
		*num_modes = 0;
	}
}

/* Read registers up to 2 at a time */
static int imx294_read_reg(struct imx294 *imx294, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx294->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/* Write registers 1 byte at a time */
static int imx294_write_reg_1byte(struct imx294 *imx294, u16 reg, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx294->sd);
	u8 buf[3];

	put_unaligned_be16(reg, buf);
	buf[2]  = val;
	if (i2c_master_send(client, buf, 3) != 3)
		return -EIO;

	return 0;
}

/* Write registers 2 byte at a time */
static int imx294_write_reg_2byte(struct imx294 *imx294, u16 reg, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx294->sd);
	u8 buf[4];

	put_unaligned_be16(reg, buf);
	buf[2]  = val;
	buf[3]  = val>>8;
	if (i2c_master_send(client, buf, 4) != 4)
		return -EIO;

	return 0;
}

/* Write registers 3 byte at a time */
static int imx294_write_reg_3byte(struct imx294 *imx294, u16 reg, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx294->sd);
	u8 buf[5];

	put_unaligned_be16(reg, buf);
	buf[2]  = val;
	buf[3]  = val>>8;
	buf[4]  = val>>16;
	if (i2c_master_send(client, buf, 5) != 5)
		return -EIO;

	return 0;
}

/* Write a list of 1 byte registers */
static int imx294_write_regs(struct imx294 *imx294,
			     const struct imx294_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx294->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		if (regs[i].address == 0xFFFE) {
			usleep_range(regs[i].val*1000,(regs[i].val+1)*1000);
		}
		else{
			ret = imx294_write_reg_1byte(imx294, regs[i].address, regs[i].val);
			if (ret) {
				dev_err_ratelimited(&client->dev,
						    "Failed to write reg 0x%4.4x. error = %d\n",
						    regs[i].address, ret);

				return ret;
			}
		}
	}

	return 0;
}

/* Get bayer order based on flip setting. */
static u32 imx294_get_format_code(struct imx294 *imx294, u32 code)
{
	unsigned int i;
	lockdep_assert_held(&imx294->mutex);
	for (i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == code)
			break;

	return codes[i];
}

static void imx294_set_default_format(struct imx294 *imx294)
{
	/* Set default mode to max resolution */
	imx294->mode = &supported_modes_12bit[0];
	imx294->fmt_code = MEDIA_BUS_FMT_SGBRG12_1X12;
}

static int imx294_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx294 *imx294 = to_imx294(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_get_try_format(sd, fh->state, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_get_try_format(sd, fh->state, METADATA_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&imx294->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt_img->width = supported_modes_12bit[0].width;
	try_fmt_img->height = supported_modes_12bit[0].height;
	try_fmt_img->code = imx294_get_format_code(imx294,
						   MEDIA_BUS_FMT_SGBRG12_1X12);
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = IMX294_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = IMX294_NUM_EMBEDDED_LINES;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;

	/* Initialize try_crop */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, IMAGE_PAD);
	try_crop->left = IMX294_PIXEL_ARRAY_LEFT;
	try_crop->top = IMX294_PIXEL_ARRAY_TOP;
	try_crop->width = IMX294_PIXEL_ARRAY_WIDTH;
	try_crop->height = IMX294_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&imx294->mutex);

	return 0;
}


static u64 calculate_v4l2_cid_exposure(u64 hmax, u64 vmax, u64 shr, u64 svr, u64 offset) {
    u64 numerator;
    numerator = (vmax * (svr + 1) - shr) * hmax + offset;

    do_div(numerator, hmax);
    numerator = clamp_t(uint32_t, numerator, 0, 0xFFFFFFFF);
    return numerator;
}

static void calculate_min_max_v4l2_cid_exposure(u64 hmax, u64 vmax, u64 min_shr, u64 svr, u64 offset, u64 *min_exposure, u64 *max_exposure) {
    u64 max_shr = (svr + 1) * vmax - 4;
    max_shr = min_t(uint64_t, max_shr, 0xFFFF);

    *min_exposure = calculate_v4l2_cid_exposure(hmax, vmax, max_shr, svr, offset);
    *max_exposure = calculate_v4l2_cid_exposure(hmax, vmax, min_shr, svr, offset);
}


/*
Integration Time [s] = [{VMAX × (SVR + 1) – (SHR)}
 × HMAX + offset] / (72 × 10^6)

Integration Time [s] = exposure * HMAX / (72 × 10^6)
*/

static uint32_t calculate_shr(uint32_t exposure, uint32_t hmax, uint64_t vmax, uint32_t svr, uint32_t offset) {
    uint64_t temp;
    uint32_t shr;

    temp = ((uint64_t)exposure * hmax - offset);
    do_div(temp, hmax);
    shr = (uint32_t)(vmax * (svr + 1) - temp);

    return shr;
}

static int imx294_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx294 *imx294 =
		container_of(ctrl->handler, struct imx294, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx294->sd);
	const struct imx294_mode *mode = imx294->mode;
    u64 shr, vblk, tmp;
	int ret = 0;
        u64 pixel_rate,hmax;
	/*
	 * The VBLANK control may change the limits of usable exposure, so check
	 * and adjust if necessary.
	 */
	if (ctrl->id == V4L2_CID_VBLANK){
		/* Honour the VBLANK limits when setting exposure. */
		u64 current_exposure, max_exposure, min_exposure, vmax;

        vmax = ((u64)mode->height + ctrl->val);
        do_div(vmax, mode->VMAX_scale);

		imx294 -> VMAX = vmax;
		
		calculate_min_max_v4l2_cid_exposure(imx294 -> HMAX, imx294 -> VMAX, (u64)mode->min_SHR, 0, mode->integration_offset, &min_exposure, &max_exposure);
		current_exposure = clamp_t(uint32_t, current_exposure, min_exposure, max_exposure);

		DEBUG_PRINTK("exposure_max:%lld, exposure_min:%lld, current_exposure:%lld\n",max_exposure, min_exposure, current_exposure);
		DEBUG_PRINTK("\tVMAX:%d, HMAX:%d\n",imx294->VMAX, imx294->HMAX);
		__v4l2_ctrl_modify_range(imx294->exposure, min_exposure,max_exposure, 1,current_exposure);
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	
	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		{
		DEBUG_PRINTK("V4L2_CID_EXPOSURE : %d\n",ctrl->val);
		DEBUG_PRINTK("\tvblank:%d, hblank:%d\n",imx294->vblank->val, imx294->hblank->val);
		DEBUG_PRINTK("\tVMAX:%d, HMAX:%d\n",imx294->VMAX, imx294->HMAX);
		shr = calculate_shr(ctrl->val, imx294->HMAX, imx294->VMAX, 0, mode->integration_offset);
		DEBUG_PRINTK("\tSHR:%lld\n",shr);
		ret = imx294_write_reg_2byte(imx294, IMX294_REG_SHR, shr);
		}
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		DEBUG_PRINTK("V4L2_CID_ANALOGUE_GAIN : %d\n",ctrl->val);
		ret = imx294_write_reg_2byte(imx294, IMX294_REG_ANALOG_GAIN, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		{
		DEBUG_PRINTK("V4L2_CID_VBLANK : %d\n",ctrl->val);
        tmp = ((u64)mode->height + ctrl->val);
        do_div(tmp, mode->VMAX_scale);
		imx294 -> VMAX = tmp;
		DEBUG_PRINTK("\tVMAX : %d\n",imx294 -> VMAX);
		ret = imx294_write_reg_3byte(imx294, IMX294_REG_VMAX, imx294 -> VMAX);
        vblk = imx294 -> VMAX  - mode-> min_VMAX;
        DEBUG_PRINTK("\tvblk : %lld\n",vblk);
        ret = imx294_write_reg_2byte(imx294, IMX294_REG_PSSLVS1, vblk);
        ret = imx294_write_reg_2byte(imx294, IMX294_REG_PSSLVS2, vblk);
        ret = imx294_write_reg_2byte(imx294, IMX294_REG_PSSLVS3, vblk);
        if(vblk <= 5){
            ret = imx294_write_reg_2byte(imx294, IMX294_REG_PSSLVS4, 0);
        }
        else{
            ret = imx294_write_reg_2byte(imx294, IMX294_REG_PSSLVS4, vblk - 5);
        }
        ret = imx294_write_reg_2byte(imx294, IMX294_REG_PSSLVS0, vblk);
		}
		break;
	case V4L2_CID_HBLANK:
		{
		DEBUG_PRINTK("V4L2_CID_HBLANK : %d\n",ctrl->val);
		//int hmax = (IMX294_NATIVE_WIDTH + ctrl->val) * 72000000; / IMX294_PIXEL_RATE;
		pixel_rate = (u64)mode->width * 72000000;
		do_div(pixel_rate,mode->min_HMAX);
		hmax = (u64)(mode->width + ctrl->val) * 72000000;
		do_div(hmax,pixel_rate);
		imx294 -> HMAX = hmax;
		DEBUG_PRINTK("\tHMAX : %d\n",imx294 -> HMAX);
		ret = imx294_write_reg_2byte(imx294, IMX294_REG_HMAX, hmax);
        ret = imx294_write_reg_2byte(imx294, IMX294_REG_HCOUNT1, hmax);
        ret = imx294_write_reg_2byte(imx294, IMX294_REG_HCOUNT2, hmax);
		}
		break;
	default:
		dev_err(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx294_ctrl_ops = {
	.s_ctrl = imx294_set_ctrl,
};

static int imx294_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx294 *imx294 = to_imx294(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		if (code->index >= (ARRAY_SIZE(codes) / 4))
			return -EINVAL;

		code->code = imx294_get_format_code(imx294,
						    codes[code->index * 4]);
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int imx294_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx294 *imx294 = to_imx294(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		const struct imx294_mode *mode_list;
		unsigned int num_modes;

		get_mode_table(fse->code, &mode_list, &num_modes);

		if (fse->index >= num_modes)
			return -EINVAL;

		if (fse->code != imx294_get_format_code(imx294, fse->code))
			return -EINVAL;

		fse->min_width = mode_list[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = mode_list[fse->index].height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = IMX294_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = IMX294_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void imx294_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx294_update_image_pad_format(struct imx294 *imx294,
					   const struct imx294_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx294_reset_colorspace(&fmt->format);
}

static void imx294_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = IMX294_EMBEDDED_LINE_WIDTH;
	fmt->format.height = IMX294_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int imx294_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx294 *imx294 = to_imx294(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx294->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&imx294->sd, sd_state,
						   fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = fmt->pad == IMAGE_PAD ?
				imx294_get_format_code(imx294, try_fmt->code) :
				MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			imx294_update_image_pad_format(imx294, imx294->mode,
						       fmt);
			fmt->format.code =
			       imx294_get_format_code(imx294, imx294->fmt_code);
		} else {
			imx294_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx294->mutex);
	return 0;
}

/* TODO */
static void imx294_set_framing_limits(struct imx294 *imx294)
{
	const struct imx294_mode *mode = imx294->mode;
	u64 def_hblank;
	u64 pixel_rate;


	imx294->VMAX = mode->default_VMAX;
	imx294->HMAX = mode->default_HMAX;

	pixel_rate = (u64)mode->width * 72000000 * 2;
	do_div(pixel_rate,mode->min_HMAX);
	DEBUG_PRINTK("Pixel Rate : %lld\n",pixel_rate);


	//int def_hblank = mode->default_HMAX * IMX294_PIXEL_RATE / 72000000 - IMX294_NATIVE_WIDTH;
	def_hblank = mode->default_HMAX * pixel_rate;
	do_div(def_hblank,72000000);
	def_hblank = def_hblank - mode->width;
	__v4l2_ctrl_modify_range(imx294->hblank, 0,
				 IMX294_HMAX_MAX, 1, def_hblank);


	__v4l2_ctrl_s_ctrl(imx294->hblank, def_hblank);



	/* Update limits and set FPS to default */
	__v4l2_ctrl_modify_range(imx294->vblank, mode->min_VMAX*mode->VMAX_scale - mode->height,
				 IMX294_VMAX_MAX*mode->VMAX_scale - mode->height,
				 1, mode->default_VMAX*mode->VMAX_scale - mode->height);
	__v4l2_ctrl_s_ctrl(imx294->vblank, mode->default_VMAX*mode->VMAX_scale - mode->height);

	/* Setting this will adjust the exposure limits as well. */

	__v4l2_ctrl_modify_range(imx294->pixel_rate, pixel_rate, pixel_rate, 1, pixel_rate);

	DEBUG_PRINTK("Setting default HBLANK : %lld, VBLANK : %lld with PixelRate: %lld\n",def_hblank,mode->default_VMAX*mode->VMAX_scale - mode->height, pixel_rate);

}
/* TODO */
static int imx294_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	const struct imx294_mode *mode;
	struct imx294 *imx294 = to_imx294(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx294->mutex);

	if (fmt->pad == IMAGE_PAD) {
		const struct imx294_mode *mode_list;
		unsigned int num_modes;

		/* Bayer order varies with flips */
		fmt->format.code = imx294_get_format_code(imx294,
							  fmt->format.code);

		get_mode_table(fmt->format.code, &mode_list, &num_modes);

		mode = v4l2_find_nearest_size(mode_list,
					      num_modes,
					      width, height,
					      fmt->format.width,
					      fmt->format.height);
		imx294_update_image_pad_format(imx294, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else if (imx294->mode != mode) {
			imx294->mode = mode;
			imx294->fmt_code = fmt->format.code;
			imx294_set_framing_limits(imx294);
		}
	} else {
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			imx294_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx294->mutex);

	return 0;
}
/* TODO */
static const struct v4l2_rect *
__imx294_get_pad_crop(struct imx294 *imx294,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&imx294->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx294->mode->crop;
	}

	return NULL;
}

/* Start streaming */
static int imx294_start_streaming(struct imx294 *imx294)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx294->sd);
	const struct IMX294_reg_list *reg_list;
	int ret;

	if (!imx294->common_regs_written) {
		ret = imx294_write_regs(imx294, mode_common_regs,
					ARRAY_SIZE(mode_common_regs));
		if (ret) {
			dev_err(&client->dev, "%s failed to set common settings\n",
				__func__);
			return ret;
		}
		imx294->common_regs_written = true;
	}

	/* Apply default values of current mode */
	reg_list = &imx294->mode->reg_list;
	ret = imx294_write_regs(imx294, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx294->sd.ctrl_handler);

	return ret;
}

/* Stop streaming */
static void imx294_stop_streaming(struct imx294 *imx294)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx294->sd);
	int ret;

	/* set stream off register */
	ret = imx294_write_reg_1byte(imx294, IMX294_REG_MODE_SELECT, IMX294_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);
}

static int imx294_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx294 *imx294 = to_imx294(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&imx294->mutex);
	if (imx294->streaming == enable) {
		mutex_unlock(&imx294->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx294_start_streaming(imx294);
		if (ret)
			goto err_rpm_put;
	} else {
		imx294_stop_streaming(imx294);
		pm_runtime_put(&client->dev);
	}

	imx294->streaming = enable;
	mutex_unlock(&imx294->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&imx294->mutex);

	return ret;
}

/* Power/clock management functions */
static int imx294_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx294 *imx294 = to_imx294(sd);
	int ret;

	ret = regulator_bulk_enable(imx294_NUM_SUPPLIES,
				    imx294->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx294->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx294->reset_gpio, 1);
	usleep_range(imx294_XCLR_MIN_DELAY_US,
		     imx294_XCLR_MIN_DELAY_US + imx294_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(imx294_NUM_SUPPLIES, imx294->supplies);
	return ret;
}

static int imx294_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx294 *imx294 = to_imx294(sd);

	gpiod_set_value_cansleep(imx294->reset_gpio, 0);
	regulator_bulk_disable(imx294_NUM_SUPPLIES, imx294->supplies);
	clk_disable_unprepare(imx294->xclk);

	/* Force reprogramming of the common registers when powered up again. */
	imx294->common_regs_written = false;

	return 0;
}

static int __maybe_unused imx294_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx294 *imx294 = to_imx294(sd);

	if (imx294->streaming)
		imx294_stop_streaming(imx294);

	return 0;
}

static int __maybe_unused imx294_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx294 *imx294 = to_imx294(sd);
	int ret;

	if (imx294->streaming) {
		ret = imx294_start_streaming(imx294);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx294_stop_streaming(imx294);
	imx294->streaming = 0;
	return ret;
}

static int imx294_get_regulators(struct imx294 *imx294)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx294->sd);
	unsigned int i;

	for (i = 0; i < imx294_NUM_SUPPLIES; i++)
		imx294->supplies[i].supply = imx294_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       imx294_NUM_SUPPLIES,
				       imx294->supplies);
}

/* Verify chip ID */
static int imx294_identify_module(struct imx294 *imx294, u32 expected_id)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx294->sd);
	int ret;
	u32 val;

	ret = imx294_read_reg(imx294, IMX294_REG_CHIP_ID,
			      1, &val);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id %x, with error %d\n",
			expected_id, ret);
		return ret;
	}

	dev_info(&client->dev, "Device found\n");

	return 0;
}

static int imx294_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx294 *imx294 = to_imx294(sd);

		mutex_lock(&imx294->mutex);
		sel->r = *__imx294_get_pad_crop(imx294, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&imx294->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX294_NATIVE_WIDTH;
		sel->r.height = IMX294_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = IMX294_PIXEL_ARRAY_LEFT;
		sel->r.top = IMX294_PIXEL_ARRAY_TOP;
		sel->r.width = IMX294_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX294_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}


static const struct v4l2_subdev_core_ops imx294_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx294_video_ops = {
	.s_stream = imx294_set_stream,
};

static const struct v4l2_subdev_pad_ops imx294_pad_ops = {
	.enum_mbus_code = imx294_enum_mbus_code,
	.get_fmt = imx294_get_pad_format,
	.set_fmt = imx294_set_pad_format,
	.get_selection = imx294_get_selection,
	.enum_frame_size = imx294_enum_frame_size,
};

static const struct v4l2_subdev_ops imx294_subdev_ops = {
	.core = &imx294_core_ops,
	.video = &imx294_video_ops,
	.pad = &imx294_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx294_internal_ops = {
	.open = imx294_open,
};




/* Initialize control handlers */
static int imx294_init_controls(struct imx294 *imx294)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&imx294->sd);
	struct v4l2_fwnode_device_properties props;
	int ret;

	ctrl_hdlr = &imx294->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	mutex_init(&imx294->mutex);
	ctrl_hdlr->lock = &imx294->mutex;



	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the imx294_set_framing_limits() call below.
	 */
	/* By default, PIXEL_RATE is read only */
	imx294->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx294_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       0xffff,
					       0xffff, 1,
					       0xffff);
	imx294->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx294_ctrl_ops,
					   V4L2_CID_VBLANK, 0, 0xfffff, 1, 0);
	imx294->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx294_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);

	imx294->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx294_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX294_EXPOSURE_MIN,
					     IMX294_EXPOSURE_MAX,
					     IMX294_EXPOSURE_STEP,
					     IMX294_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx294_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX294_ANA_GAIN_MIN, IMX294_ANA_GAIN_MAX,
			  IMX294_ANA_GAIN_STEP, IMX294_ANA_GAIN_DEFAULT);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx294_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	imx294->sd.ctrl_handler = ctrl_hdlr;

	/* Setup exposure and frame/line length limits. */
	imx294_set_framing_limits(imx294);

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx294->mutex);

	return ret;
}

static void imx294_free_controls(struct imx294 *imx294)
{
	v4l2_ctrl_handler_free(imx294->sd.ctrl_handler);
	mutex_destroy(&imx294->mutex);
}


static const struct imx294_compatible_data imx294_compatible = {
	.chip_id = IMX294_CHIP_ID,
	.extra_regs = {
		.num_of_regs = 0,
		.regs = NULL
	}
};

static const struct of_device_id imx294_dt_ids[] = {
	{ .compatible = "sony,imx294", .data = &imx294_compatible },
	{ /* sentinel */ }
};

static int imx294_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx294 *imx294;
	const struct of_device_id *match;
	int ret;

	imx294 = devm_kzalloc(&client->dev, sizeof(*imx294), GFP_KERNEL);
	if (!imx294)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx294->sd, client, &imx294_subdev_ops);

	match = of_match_device(imx294_dt_ids, dev);
	if (!match)
		return -ENODEV;
	imx294->compatible_data =
		(const struct imx294_compatible_data *)match->data;

	/* Get system clock (xclk) */
	imx294->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx294->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx294->xclk);
	}

	imx294->xclk_freq = clk_get_rate(imx294->xclk);
	if (imx294->xclk_freq != IMX294_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			imx294->xclk_freq);
		return -EINVAL;
	}

	ret = imx294_get_regulators(imx294);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	imx294->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	
	/*
	 * The sensor must be powered for imx294_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = imx294_power_on(dev);
	if (ret)
		return ret;

	ret = imx294_identify_module(imx294, imx294->compatible_data->chip_id);
	if (ret)
		goto error_power_off;

	/* Initialize default format */
	imx294_set_default_format(imx294);

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	/* This needs the pm runtime to be registered. */
	ret = imx294_init_controls(imx294);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	imx294->sd.internal_ops = &imx294_internal_ops;
	imx294->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	imx294->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	imx294->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	imx294->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx294->sd.entity, NUM_PADS, imx294->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx294->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	return 0;

error_media_entity:
	media_entity_cleanup(&imx294->sd.entity);

error_handler_free:
	imx294_free_controls(imx294);

error_power_off:
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	imx294_power_off(&client->dev);

	return ret;
}

static void imx294_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx294 *imx294 = to_imx294(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx294_free_controls(imx294);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx294_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

}

MODULE_DEVICE_TABLE(of, imx294_dt_ids);

static const struct dev_pm_ops imx294_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx294_suspend, imx294_resume)
	SET_RUNTIME_PM_OPS(imx294_power_off, imx294_power_on, NULL)
};

static struct i2c_driver imx294_i2c_driver = {
	.driver = {
		.name = "imx294",
		.of_match_table	= imx294_dt_ids,
		.pm = &imx294_pm_ops,
	},
	.probe = imx294_probe,
	.remove = imx294_remove,
};

module_i2c_driver(imx294_i2c_driver);

MODULE_AUTHOR("Will Whang <will@willwhang.com>");
MODULE_DESCRIPTION("Sony imx294 sensor driver");
MODULE_LICENSE("GPL v2");
