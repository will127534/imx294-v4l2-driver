// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony IMX294 camera.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>


/* --------------------------------------------------------------------------
 * Driver-local custom controls
 * --------------------------------------------------------------------------
 */

#ifndef V4L2_CID_USER_IMX585_BASE
#define V4L2_CID_USER_IMX585_BASE (V4L2_CID_USER_BASE + 0x2000)
#endif

#define V4L2_CID_IMX585_HCG_GAIN        (V4L2_CID_USER_IMX585_BASE + 6)

/* --------------------------------------------------------------------------
 * Registers / limits
 * --------------------------------------------------------------------------
 */

/* Standby or streaming mode */
#define IMX294_REG_MODE_SELECT          CCI_REG8(0x3000)
#define IMX294_MODE_STANDBY_BITS        BIT(0)
#define IMX294_MODE_STBLOGIC_BITS       BIT(1)
#define IMX294_MODE_STBMIPI_BITS        BIT(3)
#define IMX294_MODE_STBDV_BITS          BIT(4)

/* Sleep, 0x00 Normal 0x01 Circuit standby */
#define IMX294_REG_SLEEP                CCI_REG8(0x3111)

/* 
 * PLL Standby Control 
 * These registers are valid only when STANDBY = 0h
 */
#define IMX294_REG_STBPL                CCI_REG8(0x310B)
#define IMX294_MODE_STBPL_IF_BITS       BIT(0)
#define IMX294_MODE_STBPL_AD_BITS       BIT(4)

/* Clock control */
#define IMX294_REG_CLKCTL               CCI_REG8(0x355E)

#define IMX294_REG_XMSTA_MSTSLV         CCI_REG8(0x3033)
/* Master Mode Operation Control 0x00 = start 0x01 = stop */
#define IMX294_MODE_XMSTA_BITS           BIT(0)
/* Master / Slave Switching Control 0x00 = slave 0x01 = master*/
#define IMX294_MODE_MSTSLV_BITS          BIT(5)

/* Embedded Data Output Setting 0x00 = disable 0x01 = enable */
#define IMX294_REG_EBDDATAEN             CCI_REG8(0x378C)

#define IMX294_STREAM_DELAY_US          25000
#define IMX294_STREAM_DELAY_RANGE_US    1000

/* Initialisation delay between XCLR low->high and the moment sensor is ready */
#define IMX294_XCLR_MIN_DELAY_US        500000
#define IMX294_XCLR_DELAY_RANGE_US      1000

/* XVS/XHS output (0x02) or Hi-Z (0x03) */
#define IMX294_REG_SYNCDRV              CCI_REG8(0x3017)

/* Input Frequency Setting Registers */
#define IMX294_REG_PLRD1               CCI_REG16_LE(0x31E8)
#define IMX294_REG_PLRD2               CCI_REG8(0x3122)
#define IMX294_REG_PLRD3               CCI_REG8(0x3129)
#define IMX294_REG_PLRD4               CCI_REG8(0x312A)
#define IMX294_REG_PLRD10              CCI_REG8(0x311F)
#define IMX294_REG_PLRD11              CCI_REG8(0x3123)
#define IMX294_REG_PLRD12              CCI_REG8(0x3124)
#define IMX294_REG_PLRD13              CCI_REG8(0x3125)
#define IMX294_REG_PLRD14              CCI_REG8(0x3127)
#define IMX294_REG_PLRD15              CCI_REG8(0x312D)

/* Clamp Reset */
#define IMX294_REG_CLPSQRST            CCI_REG8(0x3001)

/* VMAX internal VBLANK */
#define IMX294_REG_VMAX                 CCI_REG24_LE(0x30A9)
#define IMX294_VMAX_MAX                 0xfffff

/* HMAX internal HBLANK */
#define IMX294_REG_HMAX                 CCI_REG16_LE(0x30AC)
#define IMX294_HMAX_MAX                 0xffff

#define IMX294_REG_HCOUNT1              CCI_REG16_LE(0x3084)
#define IMX294_REG_HCOUNT2              CCI_REG16_LE(0x3086)


#define IMX294_REG_PSSLVS1              CCI_REG16_LE(0x332C)
#define IMX294_REG_PSSLVS2              CCI_REG16_LE(0x334A)
#define IMX294_REG_PSSLVS3              CCI_REG16_LE(0x35B6)
#define IMX294_REG_PSSLVS4              CCI_REG16_LE(0x35B8)
#define IMX294_REG_PSSLVS0              CCI_REG16_LE(0x36BC)

/* SHR internal (coarse exposure) */
#define IMX294_REG_SHR                  CCI_REG16_LE(0x302C)
#define IMX294_SHR_MIN                  8
#define IMX294_SHR_MAX                  0xfffff
#define IMX294_REG_SVR                  CCI_REG16_LE(0x300E)

/* Exposure control (lines) */
#define IMX294_EXPOSURE_MIN             2
#define IMX294_EXPOSURE_STEP            1
#define IMX294_EXPOSURE_DEFAULT         1000
#define IMX294_EXPOSURE_MAX             49865

/* Black level control */
#define IMX294_REG_BLKLEVEL             CCI_REG16_LE(0x3042)
#define IMX294_BLKLEVEL_DEFAULT         50

/* Digital Clamp */
#define IMX294_REG_DIGITAL_CLAMP        CCI_REG8(0x3458)

/* Analog gain control */
#define IMX294_REG_ANALOG_GAIN          CCI_REG16_LE(0x300A)
#define IMX294_REG_MCOVGAIN             CCI_REG8(0x3092)
#define IMX294_ANA_GAIN_MIN             0
#define IMX294_ANA_GAIN_MAX             1957
#define IMX294_ANA_GAIN_STEP            1
#define IMX294_ANA_GAIN_DEFAULT         0

/* Vertical Flip */
#define IMX294_REG_MDVREV            CCI_REG8(0x3019)

/* Pixel rate helper (sensor line clock proxy used below) */
#define IMX294_PIXEL_RATE               72000000U
#define IMX294_LINK_FREQ               864000000U

/* Native array */
// 17:9 native
#define IMX294_NATIVE_WIDTH_17_9   4096
#define IMX294_NATIVE_HEIGHT_17_9  2160
// 4:3 native
#define IMX294_NATIVE_WIDTH_4_3    3704
#define IMX294_NATIVE_HEIGHT_4_3   2778


/* Vertical Arbitrary Cropping */
#define IMX294_REG_VWIDCUTEN        CCI_REG8(0x30DD)
/* Width of vertical arbitrary cropping  */
#define IMX294_REG_VWIDCUT          CCI_REG16_LE(0x30DE)
/* Start position of vertical arbitrary cropping  */
#define IMX294_REG_VWINPOS          CCI_REG16_LE(0x30E0)

#define IMX294_REG_VCUTMODE         CCI_REG8(0x30E2)

/* Horizontal Arbitrary Cropping */
#define IMX294_REG_HTRIMMING_EN        CCI_REG8(0x3035)
/* Horizontal cropping start position  */
#define IMX294_REG_HTRIMMING_START     CCI_REG16_LE(0x3036)
/* Horizontal cropping end position +1  */
#define IMX294_REG_HTRIMMING_END       CCI_REG16_LE(0x3038)

static const s64 imx294_link_freq_menu[] = {
    IMX294_LINK_FREQ,
};


struct imx294_input_frequency {
    unsigned int mhz;
    unsigned int reg_count;
    struct cci_reg_sequence regs[10];
};

static const struct imx294_input_frequency imx294_frequencies[] = {
    {
        .mhz = 6000000U,
        .reg_count = 10,
        .regs = {
            {IMX294_REG_PLRD1,0x0120},
            {IMX294_REG_PLRD2,0x00},
            {IMX294_REG_PLRD3,0x90},
            {IMX294_REG_PLRD4,0x00},
            {IMX294_REG_PLRD10,0x00},
            {IMX294_REG_PLRD11,0x00},
            {IMX294_REG_PLRD12,0x00},
            {IMX294_REG_PLRD13,0x01},
            {IMX294_REG_PLRD14,0x02},
            {IMX294_REG_PLRD15,0x02},
        },
    },
    {
        .mhz = 12000000U,
        .reg_count = 10,
        .regs = {
            {IMX294_REG_PLRD1,0x0120},
            {IMX294_REG_PLRD2,0x01},
            {IMX294_REG_PLRD3,0x90},
            {IMX294_REG_PLRD4,0x01},
            {IMX294_REG_PLRD10,0x00},
            {IMX294_REG_PLRD11,0x00},
            {IMX294_REG_PLRD12,0x00},
            {IMX294_REG_PLRD13,0x01},
            {IMX294_REG_PLRD14,0x02},
            {IMX294_REG_PLRD15,0x02},
        },
    },
    {
        .mhz = 18000000U,
        .reg_count = 10,
        .regs = {
            {IMX294_REG_PLRD1,0x01C0},
            {IMX294_REG_PLRD2,0x01},
            {IMX294_REG_PLRD3,0x60},
            {IMX294_REG_PLRD4,0x01},
            {IMX294_REG_PLRD10,0x00},
            {IMX294_REG_PLRD11,0x00},
            {IMX294_REG_PLRD12,0x00},
            {IMX294_REG_PLRD13,0x01},
            {IMX294_REG_PLRD14,0x02},
            {IMX294_REG_PLRD15,0x02},
        },
    },
    {
        .mhz = 24000000U,
        .reg_count = 10,
        .regs = {
            {IMX294_REG_PLRD1,0x0120},
            {IMX294_REG_PLRD2,0x02},
            {IMX294_REG_PLRD3,0x90},
            {IMX294_REG_PLRD4,0x02},
            {IMX294_REG_PLRD10,0x00},
            {IMX294_REG_PLRD11,0x00},
            {IMX294_REG_PLRD12,0x00},
            {IMX294_REG_PLRD13,0x01},
            {IMX294_REG_PLRD14,0x02},
            {IMX294_REG_PLRD15,0x02},
        },
    },
};


/* --------------------------------------------------------------------------
 * Mode Registers
 * --------------------------------------------------------------------------
 */

#define IMX294_REG_MDSEL1      CCI_REG8(0x3004)
#define IMX294_REG_MDSEL2      CCI_REG8(0x3005)
#define IMX294_REG_MDSEL3      CCI_REG8(0x3006)
#define IMX294_REG_MDSEL4      CCI_REG8(0x3007)
#define IMX294_REG_MDSEL5      CCI_REG8(0x3030)
#define IMX294_REG_MDSEL15     CCI_REG16_LE(0x3068)
#define IMX294_REG_MDSEL6      CCI_REG8(0x3080)
#define IMX294_REG_MDSEL7      CCI_REG8(0x3081)
#define IMX294_REG_MDSEL8      CCI_REG8(0x30A8)
#define IMX294_REG_MDSEL11     CCI_REG8(0x357F)
#define IMX294_REG_MDSEL12     CCI_REG8(0x3580)
#define IMX294_REG_MDSEL13     CCI_REG8(0x3581)
#define IMX294_REG_MDSEL14     CCI_REG8(0x3583)
#define IMX294_REG_MDSEL16     CCI_REG16_LE(0x3600)
#define IMX294_REG_MDSEL9      CCI_REG16_LE(0x3846)
#define IMX294_REG_MDSEL10     CCI_REG16_LE(0x384A)

#define IMX294_REG_HOPBOUT     CCI_REG8(0x3034)
#define IMX294_REG_VCUTMODE    CCI_REG8(0x30E2)
#define IMX294_REG_OPB_SIZE_V  CCI_REG8(0x312F)
#define IMX294_REG_WRITE_VSIZE CCI_REG16_LE(0x3130)
#define IMX294_REG_Y_OUT_SIZE  CCI_REG16_LE(0x3132)



static const struct cci_reg_sequence mode_common_regs_stage1[] = {

    {IMX294_REG_MODE_SELECT,0x12}, //STANDBY = 0 STBLOGIC register = 1h, STBMIPI register = 0h, STBDV register = 1h
    {IMX294_REG_STBPL,0x00}, //PLL release

    //PLSTMG Settings
    {CCI_REG8(0x3047),0x01}, //PLSTMG11
    {CCI_REG8(0x304E),0x0B}, //PLSTMG12
    {CCI_REG8(0x304F),0x24}, //PLSTMG13
    {CCI_REG8(0x3062),0x25}, //PLSTMG14
    {CCI_REG8(0x3064),0x78}, //PLSTMG15
    {CCI_REG8(0x3065),0x33}, //PLSTMG16
    {CCI_REG8(0x3067),0x71}, //PLSTMG17
    {CCI_REG8(0x3088),0x75}, //PLSTMG18
    {CCI_REG8(0x308A),0x09}, //PLSTMG19
    {CCI_REG8(0x308B),0x01}, //PLSTMG19
    {CCI_REG8(0x308C),0x61}, //PLSTMG20
    {CCI_REG8(0x3146),0x00}, //PLSTMG10
    {CCI_REG8(0x3234),0x32}, //PLSTMG21
    {CCI_REG8(0x3235),0x00}, //PLSTMG21
    {CCI_REG8(0x3248),0xBC}, //PLSTMG22
    {CCI_REG8(0x3249),0x00}, //PLSTMG22
    {CCI_REG8(0x3250),0xBC}, //PLSTMG23
    {CCI_REG8(0x3251),0x00}, //PLSTMG23
    {CCI_REG8(0x3258),0xBC}, //PLSTMG24
    {CCI_REG8(0x3259),0x00}, //PLSTMG24
    {CCI_REG8(0x3260),0xBC}, //PLSTMG25
    {CCI_REG8(0x3261),0x00}, //PLSTMG25
    {CCI_REG8(0x3274),0x13}, //PLSTMG26
    {CCI_REG8(0x3275),0x00}, //PLSTMG26
    {CCI_REG8(0x3276),0x1F}, //PLSTMG27
    {CCI_REG8(0x3277),0x00}, //PLSTMG27
    {CCI_REG8(0x3278),0x30}, //PLSTMG28
    {CCI_REG8(0x3279),0x00}, //PLSTMG28
    {CCI_REG8(0x327C),0x13}, //PLSTMG29
    {CCI_REG8(0x327D),0x00}, //PLSTMG29
    {CCI_REG8(0x327E),0x1F}, //PLSTMG30
    {CCI_REG8(0x327F),0x00}, //PLSTMG30
    {CCI_REG8(0x3280),0x30}, //PLSTMG31
    {CCI_REG8(0x3281),0x00}, //PLSTMG31
    {CCI_REG8(0x3284),0x13}, //PLSTMG32
    {CCI_REG8(0x3285),0x00}, //PLSTMG32
    {CCI_REG8(0x3286),0x1F}, //PLSTMG33
    {CCI_REG8(0x3287),0x00}, //PLSTMG33
    {CCI_REG8(0x3288),0x30}, //PLSTMG34
    {CCI_REG8(0x3289),0x00}, //PLSTMG34
    {CCI_REG8(0x328C),0x13}, //PLSTMG35
    {CCI_REG8(0x328D),0x00}, //PLSTMG35
    {CCI_REG8(0x328E),0x1F}, //PLSTMG36
    {CCI_REG8(0x328F),0x00}, //PLSTMG36
    {CCI_REG8(0x3290),0x30}, //PLSTMG37
    {CCI_REG8(0x3291),0x00}, //PLSTMG37
    {CCI_REG8(0x32AE),0x00}, //PLSTMG38
    {CCI_REG8(0x32AF),0x00}, //PLSTMG39
    {CCI_REG8(0x32CA),0x5A}, //PLSTMG40
    {CCI_REG8(0x32CB),0x00}, //PLSTMG40
    {CCI_REG8(0x332F),0x00}, //PLSTMG41
    {CCI_REG8(0x334C),0x01}, //PLSTMG09
    {CCI_REG8(0x335A),0x79}, //PLSTMG43
    {CCI_REG8(0x335B),0x00}, //PLSTMG43
    {CCI_REG8(0x335E),0x56}, //PLSTMG44
    {CCI_REG8(0x335F),0x00}, //PLSTMG44
    {CCI_REG8(0x3360),0x6A}, //PLSTMG45
    {CCI_REG8(0x3361),0x00}, //PLSTMG45
    {CCI_REG8(0x336A),0x56}, //PLSTMG46
    {CCI_REG8(0x336B),0x00}, //PLSTMG46
    {CCI_REG8(0x33D6),0x79}, //PLSTMG47
    {CCI_REG8(0x33D7),0x00}, //PLSTMG47
    {CCI_REG8(0x340C),0x6E}, //PLSTMG48
    {CCI_REG8(0x340D),0x00}, //PLSTMG48
    {CCI_REG8(0x3448),0x7E}, //PLSTMG49
    {CCI_REG8(0x3449),0x00}, //PLSTMG49
    {CCI_REG8(0x348E),0x6F}, //PLSTMG50
    {CCI_REG8(0x348F),0x00}, //PLSTMG50
    {CCI_REG8(0x3492),0x11}, //PLSTMG51
    {CCI_REG8(0x34C4),0x5A}, //PLSTMG52
    {CCI_REG8(0x34C5),0x00}, //PLSTMG52
    {CCI_REG8(0x3506),0x56}, //PLSTMG53
    {CCI_REG8(0x3507),0x00}, //PLSTMG53
    {CCI_REG8(0x350C),0x56}, //PLSTMG54
    {CCI_REG8(0x350D),0x00}, //PLSTMG54
    {CCI_REG8(0x350E),0x58}, //PLSTMG55
    {CCI_REG8(0x350F),0x00}, //PLSTMG55
    {CCI_REG8(0x3549),0x04}, //PLSTMG56
    {CCI_REG8(0x355D),0x03}, //PLSTMG57
    {CCI_REG8(0x355E),0x03}, //PLSTMG58
    {CCI_REG8(0x3574),0x56}, //PLSTMG59
    {CCI_REG8(0x3575),0x00}, //PLSTMG59
    {CCI_REG8(0x3587),0x01}, //PLSTMG60
    {CCI_REG8(0x35D0),0x5E}, //PLSTMG61
    {CCI_REG8(0x35D1),0x00}, //PLSTMG61
    {CCI_REG8(0x35D4),0x63}, //PLSTMG62
    {CCI_REG8(0x35D5),0x00}, //PLSTMG62
    {CCI_REG8(0x366A),0x1A}, //PLSTMG63
    {CCI_REG8(0x366B),0x16}, //PLSTMG64
    {CCI_REG8(0x366C),0x10}, //PLSTMG65
    {CCI_REG8(0x366D),0x09}, //PLSTMG66
    {CCI_REG8(0x366E),0x00}, //PLSTMG67
    {CCI_REG8(0x366F),0x00}, //PLSTMG68
    {CCI_REG8(0x3670),0x00}, //PLSTMG69
    {CCI_REG8(0x3671),0x00}, //PLSTMG70
    {CCI_REG8(0x3676),0x83}, //PLSTMG73
    {CCI_REG8(0x3677),0x03}, //PLSTMG73
    {CCI_REG8(0x3678),0x00}, //PLSTMG74
    {CCI_REG8(0x3679),0x04}, //PLSTMG74
    {CCI_REG8(0x367A),0x2C}, //PLSTMG75
    {CCI_REG8(0x367B),0x05}, //PLSTMG75
    {CCI_REG8(0x367C),0x00}, //PLSTMG76
    {CCI_REG8(0x367D),0x06}, //PLSTMG76
    {CCI_REG8(0x367E),0x00}, //PLSTMG77
    {CCI_REG8(0x367F),0x07}, //PLSTMG77
    {CCI_REG8(0x3680),0x4B}, //PLSTMG78
    {CCI_REG8(0x3681),0x07}, //PLSTMG78
    {CCI_REG8(0x3690),0x27}, //PLSTMG79
    {CCI_REG8(0x3691),0x00}, //PLSTMG79
    {CCI_REG8(0x3692),0x65}, //PLSTMG80
    {CCI_REG8(0x3693),0x00}, //PLSTMG80
    {CCI_REG8(0x3694),0x4F}, //PLSTMG81
    {CCI_REG8(0x3695),0x00}, //PLSTMG81
    {CCI_REG8(0x3696),0xA1}, //PLSTMG82
    {CCI_REG8(0x3697),0x00}, //PLSTMG82
    {CCI_REG8(0x382B),0x68}, //PLSTMG83
    {CCI_REG8(0x3C00),0x01}, //PLSTMG84
    {CCI_REG8(0x3C01),0x01}, //PLSTMG85
    {CCI_REG8(0x3686),0x00}, //PLSTMG101
    {CCI_REG8(0x3687),0x00}, //PLSTMG101
    {CCI_REG8(0x36BE),0x01}, //PLSTMG102
    {CCI_REG8(0x36BF),0x00}, //PLSTMG102
    {CCI_REG8(0x36C0),0x01}, //PLSTMG103
    {CCI_REG8(0x36C1),0x00}, //PLSTMG103
    {CCI_REG8(0x36C2),0x01}, //PLSTMG104
    {CCI_REG8(0x36C3),0x00}, //PLSTMG104
    {CCI_REG8(0x36C4),0x01}, //PLSTMG105
    {CCI_REG8(0x36C5),0x01}, //PLSTMG106
    {CCI_REG8(0x36C6),0x01}, //PLSTMG107


    //Global Timing Registers
    {CCI_REG16_LE(0x3134),0x00AF}, //tclkpost
    {CCI_REG16_LE(0x3136),0x00C7}, //thszero
    {CCI_REG16_LE(0x3138),0x007F}, //thsprepare
    {CCI_REG16_LE(0x313A),0x006F}, //tclktrail
    {CCI_REG16_LE(0x313C),0x006F}, //thstrail
    {CCI_REG16_LE(0x313E),0x01CF}, //tclkzero
    {CCI_REG16_LE(0x3140),0x0077}, //tclkprepare
    {CCI_REG16_LE(0x3142),0x005F}, //tlpx

};

static const struct cci_reg_sequence mode_common_regs_stage2[] = {
    {IMX294_REG_MODE_SELECT,0x02}, //STANDBY register = 0h, STBLOGIC register = 1h, STBMIPI register = 0h, STBDV register = 0h
    {CCI_REG8(0x35E5),      0x92}, //CLKDIVEN register = 2h, SYSCLKEN register = 0h
    {CCI_REG8(0x35E5),      0x9A}, //CLKDIVEN register = 2h, SYSCLKEN register = 1h
    {IMX294_REG_MODE_SELECT,0x00}, //STANDBY register = 0h, STBLOGIC register = 0h, STBMIPI register = 0h, STBDV register = 0h
};

/* 
 * 17:9 Mode
 */
static const struct cci_reg_sequence mode_1_17_9_regs[] = {
    {IMX294_REG_MDSEL1,0x1A},
    {IMX294_REG_MDSEL2,0x06},
    {IMX294_REG_MDSEL3,0x00},
    {IMX294_REG_MDSEL4,0xA0},
    {IMX294_REG_MDSEL5,0x77},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x1060},
    {IMX294_REG_MDSEL15,0x001A},
    {IMX294_REG_MDSEL6,0x00},
    {IMX294_REG_MDSEL7,0x01},
    {IMX294_REG_MDSEL8,0x02},
    {IMX294_REG_VCUTMODE,0x00},
    {IMX294_REG_OPB_SIZE_V,0x08},
    {IMX294_REG_WRITE_VSIZE,0x0888},
    {IMX294_REG_Y_OUT_SIZE,0x0880},
    {IMX294_REG_MDSEL11,0x0C},
    {IMX294_REG_MDSEL12,0x0A},
    {IMX294_REG_MDSEL13,0x08},
    {IMX294_REG_MDSEL14,0x72},
    {IMX294_REG_MDSEL16,0x0090},
    {IMX294_REG_MDSEL9,0x0000},
    {IMX294_REG_MDSEL10,0x0000},
    {IMX294_REG_SVR,0x0000},
};

static const struct cci_reg_sequence mode_1A_17_9_regs[] = {
    {IMX294_REG_MDSEL1,0x01},
    {IMX294_REG_MDSEL2,0x06},
    {IMX294_REG_MDSEL3,0x00},
    {IMX294_REG_MDSEL4,0xA0},
    {IMX294_REG_MDSEL5,0x77},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x1080},
    {IMX294_REG_MDSEL15,0x001A},
    {IMX294_REG_MDSEL6,0x01},
    {IMX294_REG_MDSEL7,0x01},
    {IMX294_REG_MDSEL8,0x02},
    {IMX294_REG_VCUTMODE,0x00},
    {IMX294_REG_OPB_SIZE_V,0x08},
    {IMX294_REG_WRITE_VSIZE,0x0888},
    {IMX294_REG_Y_OUT_SIZE,0x0880},
    {IMX294_REG_MDSEL11,0x0C},
    {IMX294_REG_MDSEL12,0x0A},
    {IMX294_REG_MDSEL13,0x08},
    {IMX294_REG_MDSEL14,0x72},
    {IMX294_REG_MDSEL16,0x007D},
    {IMX294_REG_MDSEL9,0x0000},
    {IMX294_REG_MDSEL10,0x0000},
    {IMX294_REG_SVR,0x0000},
};

static const struct cci_reg_sequence mode_1B_17_9_regs[] = {
    {IMX294_REG_MDSEL1,0x02},
    {IMX294_REG_MDSEL2,0x06},
    {IMX294_REG_MDSEL3,0x01},
    {IMX294_REG_MDSEL4,0xA0},
    {IMX294_REG_MDSEL5,0x77},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x0F50},
    {IMX294_REG_MDSEL15,0x001A},
    {IMX294_REG_MDSEL6,0x00},
    {IMX294_REG_MDSEL7,0x01},
    {IMX294_REG_MDSEL8,0x02},
    {IMX294_REG_VCUTMODE,0x00},
    {IMX294_REG_OPB_SIZE_V,0x08},
    {IMX294_REG_WRITE_VSIZE,0x0888},
    {IMX294_REG_Y_OUT_SIZE,0x0880},
    {IMX294_REG_MDSEL11,0x0C},
    {IMX294_REG_MDSEL12,0x0A},
    {IMX294_REG_MDSEL13,0x08},
    {IMX294_REG_MDSEL14,0x72},
    {IMX294_REG_MDSEL16,0x0090},
    {IMX294_REG_MDSEL9,0x0000},
    {IMX294_REG_MDSEL10,0x0000},
    {IMX294_REG_SVR,0x0000},
};

static const struct cci_reg_sequence mode_2_17_9_regs[] = {
    {IMX294_REG_MDSEL1,0x1A},
    {IMX294_REG_MDSEL2,0x01},
    {IMX294_REG_MDSEL3,0x00},
    {IMX294_REG_MDSEL4,0xA0},
    {IMX294_REG_MDSEL5,0x77},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x1060},
    {IMX294_REG_MDSEL15,0x0044},
    {IMX294_REG_MDSEL6,0x00},
    {IMX294_REG_MDSEL7,0x01},
    {IMX294_REG_MDSEL8,0x02},
    {IMX294_REG_VCUTMODE,0x00},
    {IMX294_REG_OPB_SIZE_V,0x08},
    {IMX294_REG_WRITE_VSIZE,0x0888},
    {IMX294_REG_Y_OUT_SIZE,0x0880},
    {IMX294_REG_MDSEL11,0x0C},
    {IMX294_REG_MDSEL12,0x0A},
    {IMX294_REG_MDSEL13,0x0A},
    {IMX294_REG_MDSEL14,0x75},
    {IMX294_REG_MDSEL16,0x0090},
    {IMX294_REG_MDSEL9,0x0000},
    {IMX294_REG_MDSEL10,0x0000},
    {IMX294_REG_SVR,0x0000},
};

static const struct cci_reg_sequence mode_2A_17_9_regs[] = {
    {IMX294_REG_MDSEL1,0x01},
    {IMX294_REG_MDSEL2,0x01},
    {IMX294_REG_MDSEL3,0x00},
    {IMX294_REG_MDSEL4,0xA0},
    {IMX294_REG_MDSEL5,0x77},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x1080},
    {IMX294_REG_MDSEL15,0x0044},
    {IMX294_REG_MDSEL6,0x01},
    {IMX294_REG_MDSEL7,0x01},
    {IMX294_REG_MDSEL8,0x02},
    {IMX294_REG_VCUTMODE,0x00},
    {IMX294_REG_OPB_SIZE_V,0x08},
    {IMX294_REG_WRITE_VSIZE,0x0888},
    {IMX294_REG_Y_OUT_SIZE,0x0880},
    {IMX294_REG_MDSEL11,0x0C},
    {IMX294_REG_MDSEL12,0x0A},
    {IMX294_REG_MDSEL13,0x0A},
    {IMX294_REG_MDSEL14,0x75},
    {IMX294_REG_MDSEL16,0x007D},
    {IMX294_REG_MDSEL9,0x0000},
    {IMX294_REG_MDSEL10,0x0000},
    {IMX294_REG_SVR,0x0000},
};

static const struct cci_reg_sequence mode_3_17_9_regs[] = {
    {IMX294_REG_MDSEL1,0xA8},
    {IMX294_REG_MDSEL2,0x2A},
    {IMX294_REG_MDSEL3,0x00},
    {IMX294_REG_MDSEL4,0xA0},
    {IMX294_REG_MDSEL5,0x77},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x1080},
    {IMX294_REG_MDSEL15,0x001A},
    {IMX294_REG_MDSEL6,0x00},
    {IMX294_REG_MDSEL7,0x01},
    {IMX294_REG_MDSEL8,0x02},
    {IMX294_REG_VCUTMODE,0x02},
    {IMX294_REG_OPB_SIZE_V,0x04},
    {IMX294_REG_WRITE_VSIZE,0x044C},
    {IMX294_REG_Y_OUT_SIZE,0x0448},
    {IMX294_REG_MDSEL11,0x0C},
    {IMX294_REG_MDSEL12,0x0A},
    {IMX294_REG_MDSEL13,0x08},
    {IMX294_REG_MDSEL14,0x72},
    {IMX294_REG_MDSEL16,0x0090},
    {IMX294_REG_MDSEL9,0x0000},
    {IMX294_REG_MDSEL10,0x0000},
    {IMX294_REG_SVR,0x0000},
};

static const struct cci_reg_sequence mode_4_17_9_regs[] = {
    {IMX294_REG_MDSEL1,0x0A},
    {IMX294_REG_MDSEL2,0x26},
    {IMX294_REG_MDSEL3,0x00},
    {IMX294_REG_MDSEL4,0xA1},
    {IMX294_REG_MDSEL5,0x33},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x1080},
    {IMX294_REG_MDSEL15,0x001A},
    {IMX294_REG_MDSEL6,0x00},
    {IMX294_REG_MDSEL7,0x00},
    {IMX294_REG_MDSEL8,0x02},
    {IMX294_REG_VCUTMODE,0x03},
    {IMX294_REG_OPB_SIZE_V,0x04},
    {IMX294_REG_WRITE_VSIZE,0x044C},
    {IMX294_REG_Y_OUT_SIZE,0x0448},
    {IMX294_REG_MDSEL11,0x0C},
    {IMX294_REG_MDSEL12,0x0A},
    {IMX294_REG_MDSEL13,0x08},
    {IMX294_REG_MDSEL14,0x72},
    {IMX294_REG_MDSEL16,0x0090},
    {IMX294_REG_MDSEL9,0x0000},
    {IMX294_REG_MDSEL10,0x0000},
    {IMX294_REG_SVR,0x0000},
};

static const struct cci_reg_sequence mode_5_17_9_regs[] = {
    {IMX294_REG_MDSEL1,0xA8},
    {IMX294_REG_MDSEL2,0x25},
    {IMX294_REG_MDSEL3,0x00},
    {IMX294_REG_MDSEL4,0xA0},
    {IMX294_REG_MDSEL5,0x77},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x1080},
    {IMX294_REG_MDSEL15,0x0044},
    {IMX294_REG_MDSEL6,0x00},
    {IMX294_REG_MDSEL7,0x01},
    {IMX294_REG_MDSEL8,0x02},
    {IMX294_REG_VCUTMODE,0x02},
    {IMX294_REG_OPB_SIZE_V,0x04},
    {IMX294_REG_WRITE_VSIZE,0x044C},
    {IMX294_REG_Y_OUT_SIZE,0x0448},
    {IMX294_REG_MDSEL11,0x0C},
    {IMX294_REG_MDSEL12,0x0A},
    {IMX294_REG_MDSEL13,0x0A},
    {IMX294_REG_MDSEL14,0x75},
    {IMX294_REG_MDSEL16,0x0090},
    {IMX294_REG_MDSEL9,0x0000},
    {IMX294_REG_MDSEL10,0x0000},
    {IMX294_REG_SVR,0x0000},
};

static const struct cci_reg_sequence mode_6_17_9_regs[] = {
    {IMX294_REG_MDSEL1,0x0A},
    {IMX294_REG_MDSEL2,0x41},
    {IMX294_REG_MDSEL3,0x00},
    {IMX294_REG_MDSEL4,0xA1},
    {IMX294_REG_MDSEL5,0x33},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x1080},
    {IMX294_REG_MDSEL15,0x0044},
    {IMX294_REG_MDSEL6,0x00},
    {IMX294_REG_MDSEL7,0x00},
    {IMX294_REG_MDSEL8,0x02},
    {IMX294_REG_VCUTMODE,0x03},
    {IMX294_REG_OPB_SIZE_V,0x04},
    {IMX294_REG_WRITE_VSIZE,0x044C},
    {IMX294_REG_Y_OUT_SIZE,0x0448},
    {IMX294_REG_MDSEL11,0x0C},
    {IMX294_REG_MDSEL12,0x0A},
    {IMX294_REG_MDSEL13,0x0A},
    {IMX294_REG_MDSEL14,0x75},
    {IMX294_REG_MDSEL16,0x0090},
    {IMX294_REG_MDSEL9,0x0000},
    {IMX294_REG_MDSEL10,0x0000},
    {IMX294_REG_SVR,0x0000},
};

static const struct cci_reg_sequence mode_7_17_9_regs[] = {
    {IMX294_REG_MDSEL1,0x0D},
    {IMX294_REG_MDSEL2,0x41},
    {IMX294_REG_MDSEL3,0x00},
    {IMX294_REG_MDSEL4,0xA0},
    {IMX294_REG_MDSEL5,0x77},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x1080},
    {IMX294_REG_MDSEL15,0x0044},
    {IMX294_REG_MDSEL6,0x00},
    {IMX294_REG_MDSEL7,0x01},
    {IMX294_REG_MDSEL8,0x02},
    {IMX294_REG_VCUTMODE,0x04},
    {IMX294_REG_OPB_SIZE_V,0x04},
    {IMX294_REG_WRITE_VSIZE,0x044C},
    {IMX294_REG_Y_OUT_SIZE,0x0448},
    {IMX294_REG_MDSEL11,0x0C},
    {IMX294_REG_MDSEL12,0x0A},
    {IMX294_REG_MDSEL13,0x0A},
    {IMX294_REG_MDSEL14,0x75},
    {IMX294_REG_MDSEL16,0x0090},
    {IMX294_REG_MDSEL9,0x006C},
    {IMX294_REG_MDSEL10,0x0034},
    {IMX294_REG_SVR,0x0000},
};

static const struct cci_reg_sequence mode_8_17_9_regs[] = {
    {IMX294_REG_MDSEL1,0x4F},
    {IMX294_REG_MDSEL2,0x35},
    {IMX294_REG_MDSEL3,0x00},
    {IMX294_REG_MDSEL4,0xA0},
    {IMX294_REG_MDSEL5,0x77},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x1080},
    {IMX294_REG_MDSEL15,0x0044},
    {IMX294_REG_MDSEL6,0x00},
    {IMX294_REG_MDSEL7,0x01},
    {IMX294_REG_MDSEL8,0x02},
    {IMX294_REG_VCUTMODE,0x05},
    {IMX294_REG_OPB_SIZE_V,0x04},
    {IMX294_REG_WRITE_VSIZE,0x02E4},
    {IMX294_REG_Y_OUT_SIZE,0x02E0},
    {IMX294_REG_MDSEL11,0x0C},
    {IMX294_REG_MDSEL12,0x0A},
    {IMX294_REG_MDSEL13,0x0A},
    {IMX294_REG_MDSEL14,0x75},
    {IMX294_REG_MDSEL16,0x0090},
    {IMX294_REG_MDSEL9,0x0000},
    {IMX294_REG_MDSEL10,0x0000},
    {IMX294_REG_SVR,0x0000},
};

static const struct cci_reg_sequence mode_9_17_9_regs[] = {
    {IMX294_REG_MDSEL1,0x11},
    {IMX294_REG_MDSEL2,0x35},
    {IMX294_REG_MDSEL3,0x00},
    {IMX294_REG_MDSEL4,0xA0},
    {IMX294_REG_MDSEL5,0x77},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x1080},
    {IMX294_REG_MDSEL15,0x0044},
    {IMX294_REG_MDSEL6,0x00},
    {IMX294_REG_MDSEL7,0x01},
    {IMX294_REG_MDSEL8,0x02},
    {IMX294_REG_VCUTMODE,0x06},
    {IMX294_REG_OPB_SIZE_V,0x04},
    {IMX294_REG_WRITE_VSIZE,0x02E4},
    {IMX294_REG_Y_OUT_SIZE,0x02E0},
    {IMX294_REG_MDSEL11,0x0C},
    {IMX294_REG_MDSEL12,0x0A},
    {IMX294_REG_MDSEL13,0x0A},
    {IMX294_REG_MDSEL14,0x75},
    {IMX294_REG_MDSEL16,0x0090},
    {IMX294_REG_MDSEL9,0x0028},
    {IMX294_REG_MDSEL10,0x003A},
    {IMX294_REG_SVR,0x0000},
};

static const struct cci_reg_sequence mode_10_17_9_regs[] = {
    {IMX294_REG_MDSEL1,0x33},
    {IMX294_REG_MDSEL2,0x35},
    {IMX294_REG_MDSEL3,0x00},
    {IMX294_REG_MDSEL4,0xA0},
    {IMX294_REG_MDSEL5,0x77},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x1080},
    {IMX294_REG_MDSEL15,0x0044},
    {IMX294_REG_MDSEL6,0x00},
    {IMX294_REG_MDSEL7,0x01},
    {IMX294_REG_MDSEL8,0x02},
    {IMX294_REG_VCUTMODE,0x07},
    {IMX294_REG_OPB_SIZE_V,0x04},
    {IMX294_REG_WRITE_VSIZE,0x00FC},
    {IMX294_REG_Y_OUT_SIZE,0x00F8},
    {IMX294_REG_MDSEL11,0x0C},
    {IMX294_REG_MDSEL12,0x0A},
    {IMX294_REG_MDSEL13,0x0A},
    {IMX294_REG_MDSEL14,0x75},
    {IMX294_REG_MDSEL16,0x0090},
    {IMX294_REG_MDSEL9,0x0032},
    {IMX294_REG_MDSEL10,0x0020},
    {IMX294_REG_SVR,0x0000},
};

static const struct cci_reg_sequence mode_11_17_9_regs[] = {
    {IMX294_REG_MDSEL1,0x15},
    {IMX294_REG_MDSEL2,0x31},
    {IMX294_REG_MDSEL3,0x38},
    {IMX294_REG_MDSEL4,0xA1},
    {IMX294_REG_MDSEL5,0x55},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x1080},
    {IMX294_REG_MDSEL15,0x0044},
    {IMX294_REG_MDSEL6,0x00},
    {IMX294_REG_MDSEL7,0x00},
    {IMX294_REG_MDSEL8,0x02},
    {IMX294_REG_VCUTMODE,0x08},
    {IMX294_REG_OPB_SIZE_V,0x04},
    {IMX294_REG_WRITE_VSIZE,0x00FA},
    {IMX294_REG_Y_OUT_SIZE,0x00F6},
    {IMX294_REG_MDSEL11,0x0C},
    {IMX294_REG_MDSEL12,0x0A},
    {IMX294_REG_MDSEL13,0x0A},
    {IMX294_REG_MDSEL14,0x75},
    {IMX294_REG_MDSEL16,0x0090},
    {IMX294_REG_MDSEL9,0x003C},
    {IMX294_REG_MDSEL10,0x0034},
    {IMX294_REG_SVR,0x0000},
};

/* 
 * 4:3 Mode
 */
static const struct cci_reg_sequence mode_0_4_3_regs[] = {
    {IMX294_REG_MDSEL1,0x00},
    {IMX294_REG_MDSEL2,0x0B},
    {IMX294_REG_MDSEL3,0x02},
    {IMX294_REG_MDSEL4,0xA0},
    {IMX294_REG_MDVREV,0x00},
    {IMX294_REG_MDSEL5,0x77},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x0F00},
    {IMX294_REG_MDSEL15,0x0044},
    {IMX294_REG_MDSEL6,0x00},
    {IMX294_REG_MDSEL7,0x01},
    {IMX294_REG_MDSEL8,0x03},
    {IMX294_REG_VCUTMODE,0x00},
    {IMX294_REG_OPB_SIZE_V,0x10},
    {IMX294_REG_WRITE_VSIZE,0x0B18},
    {IMX294_REG_Y_OUT_SIZE,0x0B08},
    {IMX294_REG_MDSEL11,0x0A},
    {IMX294_REG_MDSEL12,0x09},
    {IMX294_REG_MDSEL13,0x07},
    {IMX294_REG_MDSEL14,0x51},
    {IMX294_REG_MDSEL16,0x0090},
    {IMX294_REG_MDSEL9,0x0000},
    {IMX294_REG_MDSEL10,0x0000},
};

static const struct cci_reg_sequence mode_1_4_3_regs[] = {
    {IMX294_REG_MDSEL1,0x00},
    {IMX294_REG_MDSEL2,0x06},
    {IMX294_REG_MDSEL3,0x02},
    {IMX294_REG_MDSEL4,0xA0},
    {IMX294_REG_MDVREV,0x00},
    {IMX294_REG_MDSEL5,0x77},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x0F00},
    {IMX294_REG_MDSEL15,0x001A},
    {IMX294_REG_MDSEL6,0x00},
    {IMX294_REG_MDSEL7,0x01},
    {IMX294_REG_MDSEL8,0x02},
    {IMX294_REG_VCUTMODE,0x00},
    {IMX294_REG_OPB_SIZE_V,0x10},
    {IMX294_REG_WRITE_VSIZE,0x0B18},
    {IMX294_REG_Y_OUT_SIZE,0x0B08},
    {IMX294_REG_MDSEL11,0x0C},
    {IMX294_REG_MDSEL12,0x0A},
    {IMX294_REG_MDSEL13,0x08},
    {IMX294_REG_MDSEL14,0x72},
    {IMX294_REG_MDSEL16,0x0090},
    {IMX294_REG_MDSEL9,0x0000},
    {IMX294_REG_MDSEL10,0x0000},
};

static const struct cci_reg_sequence mode_1A_4_3_regs[] = {
    {IMX294_REG_MDSEL1,0x00},
    {IMX294_REG_MDSEL2,0x06},
    {IMX294_REG_MDSEL3,0x02},
    {IMX294_REG_MDSEL4,0xA0},
    {IMX294_REG_MDVREV,0x00},
    {IMX294_REG_MDSEL5,0x77},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x0F00},
    {IMX294_REG_MDSEL15,0x001A},
    {IMX294_REG_MDSEL6,0x01},
    {IMX294_REG_MDSEL7,0x01},
    {IMX294_REG_MDSEL8,0x02},
    {IMX294_REG_VCUTMODE,0x00},
    {IMX294_REG_OPB_SIZE_V,0x10},
    {IMX294_REG_WRITE_VSIZE,0x0B18},
    {IMX294_REG_Y_OUT_SIZE,0x0B08},
    {IMX294_REG_MDSEL11,0x0C},
    {IMX294_REG_MDSEL12,0x0A},
    {IMX294_REG_MDSEL13,0x08},
    {IMX294_REG_MDSEL14,0x72},
    {IMX294_REG_MDSEL16,0x007D},
    {IMX294_REG_MDSEL9,0x0000},
    {IMX294_REG_MDSEL10,0x0000},
};

static const struct cci_reg_sequence mode_7_4_3_regs[] = {
    {IMX294_REG_MDSEL1,0x0C},
    {IMX294_REG_MDSEL2,0x41},
    {IMX294_REG_MDSEL3,0x02},
    {IMX294_REG_MDSEL4,0xA0},
    {IMX294_REG_MDVREV,0x00},
    {IMX294_REG_MDSEL5,0x77},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x0F00},
    {IMX294_REG_MDSEL15,0x0044},
    {IMX294_REG_MDSEL6,0x00},
    {IMX294_REG_MDSEL7,0x01},
    {IMX294_REG_MDSEL8,0x02},
    {IMX294_REG_VCUTMODE,0x04},
    {IMX294_REG_OPB_SIZE_V,0x04},
    {IMX294_REG_WRITE_VSIZE,0x0580},
    {IMX294_REG_Y_OUT_SIZE,0x057C},
    {IMX294_REG_MDSEL11,0x0C},
    {IMX294_REG_MDSEL12,0x0A},
    {IMX294_REG_MDSEL13,0x0A},
    {IMX294_REG_MDSEL14,0x75},
    {IMX294_REG_MDSEL16,0x0090},
    {IMX294_REG_MDSEL9,0x0000},
    {IMX294_REG_MDSEL10,0x0000},
};

static const struct cci_reg_sequence mode_10_4_3_regs[] = {
    {IMX294_REG_MDSEL1,0x32},
    {IMX294_REG_MDSEL2,0x35},
    {IMX294_REG_MDSEL3,0x02},
    {IMX294_REG_MDSEL4,0xA0},
    {IMX294_REG_MDVREV,0x00},
    {IMX294_REG_MDSEL5,0x77},
    {IMX294_REG_HOPBOUT,0x00},
    {IMX294_REG_HTRIMMING_EN,0x01},
    {IMX294_REG_HTRIMMING_START,0x30},
    {IMX294_REG_HTRIMMING_END,0x0F00},
    {IMX294_REG_MDSEL15,0x0044},
    {IMX294_REG_MDSEL6,0x00},
    {IMX294_REG_MDSEL7,0x01},
    {IMX294_REG_MDSEL8,0x02},
    {IMX294_REG_VCUTMODE,0x07},
    {IMX294_REG_OPB_SIZE_V,0x04},
    {IMX294_REG_WRITE_VSIZE,0x0140},
    {IMX294_REG_Y_OUT_SIZE,0x013C},
    {IMX294_REG_MDSEL11,0x0C},
    {IMX294_REG_MDSEL12,0x0A},
    {IMX294_REG_MDSEL13,0x0A},
    {IMX294_REG_MDSEL14,0x75},
    {IMX294_REG_MDSEL16,0x0090},
    {IMX294_REG_MDSEL9,0x0000},
    {IMX294_REG_MDSEL10,0x0000},
};



/* Mode description */
struct imx294_mode {
    unsigned int width;
    unsigned int height;
    u8 scale;
    u8 min_shr;
    u16 integration_offset;
    u8  hmax_div;       /* per-mode scaling of min HMAX */
    u16 min_hmax;       /* computed at runtime */
    u32 min_vmax;       /* computed at runtime (fits 20-bit) */

    struct v4l2_rect crop;

    struct {
        unsigned int num_of_regs;
        const struct cci_reg_sequence *regs;
    } reg_list;
};


/* --------------------------------------------------------------------------
 * Mode list
 * --------------------------------------------------------------------------
 * 17:9 Modes:
 *     mode 1    All-pixel scan mode (AD 12-bit, 12-bit length output) 
 *     mode 1A   All-pixel scan mode (AD 12-bit, 12-bit length output) low noise 
 *     mode 1B   All-pixel scan mode horizontal 3840 pixels (AD 12-bit, 12-bit length output) 
 *     mode 2    All-pixel scan mode (AD 10-bit, 10-bit length output) 
 *     mode 2A   All-pixel scan mode (AD 10-bit, 10-bit length output) low noise 
 *     mode 3    Horizontal/vertical 2/2-line binning (Horizontal and vertical weighted binning) (AD 12-bit, 14-bit length output) 
 *     mode 4    Horizontal/vertical 2/2-line binning (Horizontal and vertical weighted binning) (AD 12-bit, 12-bit length output) 
 *     mode 5    Horizontal/vertical 2/2-line binning (Horizontal and vertical weighted binning) (AD 10-bit, 12-bit length output) 
 *     mode 6    Vertical 2 binning Horizontal 2/4 subsampling (Vertical weighted binning) (AD 10-bit, 10-bit length output) 
 *     mode 7    Horizontal/vertical 2/4 subsampling (AD 10-bit, 10-bit length output) 
 *     mode 8    Horizontal/vertical 3/3-line binning (AD 10-bit, 12-bit length output) 
 *     mode 9    Vertical 1/3 subsampling horizontal 3 binning (AD 10-bit, 12-bit length output) 
 *     mode 10   Vertical 2/9 subsampling binning horizontal 3 binning (AD 10-bit, 12-bit length output) 
 *     mode 11   Vertical 2/9 subsampling binning horizontal 3 binning Low power consumption (AD 10-bit, 10-bit length output) 
 * 4:3 Modes:
 *     mode 0    All-pixel scan mode (AD 14-bit, 14-bit length output) 
 *     mode 1    All-pixel scan mode (AD 12-bit, 12-bit length output) 
 *     mode 1A   All-pixel scan mode (AD 12-bit, 12-bit length output) low noise 
 *     mode 7    Horizontal/vertical 2/4 subsampling (AD 10-bit, 10-bit length output) 
 *     mode 10   Vertical 2/9 subsampling binning horizontal 3 binning (AD 10-bit, 12-bit length output) 
 */

static struct imx294_mode supported_modes_10bit[] = {
   /* 17:9 Mode 2 — All-pixel scan mode (AD 10-bit, 10-bit length output) */
    {
        .width  = 4144,
        .height = 2184,
        .min_hmax = 947,
        .min_vmax = 1116,
        .scale = 2,
        .min_shr = 5,
        .integration_offset = 217,
        .crop = {
            .left = 36,
            .top = 20,
            .width = 4096,
            .height = 2160,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_2_17_9_regs),
            .regs        = mode_2_17_9_regs,
        },
    },
   /* 17:9 Mode 2A — All-pixel scan mode (AD 10-bit, 10-bit length output) low noise */
    /*{
        .width  = 4176,
        .height = 2184,
        .min_hmax = 954,
        .min_vmax = 1116,
        .scale = 2,
        .min_shr = 5,
        .integration_offset = 322,
        .crop = {
            .left = 36,
            .top = 20,
            .width = 4096,
            .height = 2160,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_2A_17_9_regs),
            .regs        = mode_2A_17_9_regs,
        },
    },*/
   /* 17:9 Mode 6 — Vertical 2 binning Horizontal 2/4 subsampling (Vertical weighted binning) (AD 10-bit, 10-bit length output) */
    {
        .width  = 2088,
        .height = 1100,
        .min_hmax = 520,
        .min_vmax = 1148,
        .scale = 1,
        .min_shr = 5,
        .integration_offset = 217,
        .crop = {
            .left = 18,
            .top = 14,
            .width = 2048,
            .height = 1080,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_6_17_9_regs),
            .regs        = mode_6_17_9_regs,
        },
    },
   /* 17:9 Mode 7 — Horizontal/vertical 2/4 subsampling (AD 10-bit, 10-bit length output) */
    /*{
        .width  = 2088,
        .height = 1100,
        .min_hmax = 520,
        .min_vmax = 574,
        .scale = 2,
        .min_shr = 3,
        .integration_offset = 217,
        .crop = {
            .left = 18,
            .top = 14,
            .width = 2048,
            .height = 1080,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_7_17_9_regs),
            .regs        = mode_7_17_9_regs,
        },
    },*/
   /* 17:9 Mode 11 — Vertical 2/9 subsampling binning horizontal 3 binning Low power consumption (AD 10-bit, 10-bit length output) */
    {
        .width  = 1392,
        .height = 250,
        .min_hmax = 520,
        .min_vmax = 298,
        .scale = 0,
        .min_shr = 5,
        .integration_offset = 217,
        .crop = {
            .left = 12,
            .top = 8,
            .width = 1364,
            .height = 240,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_11_17_9_regs),
            .regs        = mode_11_17_9_regs,
        },
    },
   /* 4:3 Mode 7 — Horizontal/vertical 2/4 subsampling (AD 10-bit, 10-bit length output) */
    {
        .width  = 1896,
        .height = 1408,
        .min_hmax = 520,
        .min_vmax = 728,
        .scale = 2,
        .min_shr = 3,
        .integration_offset = 217,
        .crop = {
            .left = 20,
            .top = 14,
            .width = 1852,
            .height = 1388,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_7_4_3_regs),
            .regs        = mode_7_4_3_regs,
        },
    },
};

static struct imx294_mode supported_modes_12bit[] = {
   /* 17:9 Mode 1 — All-pixel scan mode (AD 12-bit, 12-bit length output) */
    {
        .width  = 4144,
        .height = 2184,
        .min_hmax = 1122,
        .min_vmax = 1111,
        .scale = 2,
        .min_shr = 5,
        .integration_offset = 256,
        .crop = {
            .left = 36,
            .top = 20,
            .width = 4096,
            .height = 2160,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_1_17_9_regs),
            .regs        = mode_1_17_9_regs,
        },
    },
   /* 17:9 Mode 1A — All-pixel scan mode (AD 12-bit, 12-bit length output) low noise */
    /*{
        .width  = 4176,
        .height = 2184,
        .min_hmax = 1192,
        .min_vmax = 1111,
        .scale = 2,
        .min_shr = 5,
        .integration_offset = 361,
        .crop = {
            .left = 36,
            .top = 20,
            .width = 4096,
            .height = 2160,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_1A_17_9_regs),
            .regs        = mode_1A_17_9_regs,
        },
    },*/
   /* 17:9 Mode 1B — All-pixel scan mode horizontal 3840 pixels (AD 12-bit, 12-bit length output) */
    {
        .width  = 3872,
        .height = 2184,
        .min_hmax = 1055,
        .min_vmax = 1111,
        .scale = 2,
        .min_shr = 5,
        .integration_offset = 256,
        .crop = {
            .left = 20,
            .top = 20,
            .width = 3840,
            .height = 2160,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_1B_17_9_regs),
            .regs        = mode_1B_17_9_regs,
        },
    },
   /* 17:9 Mode 4 — Horizontal/vertical 2/2-line binning (Horizontal and vertical weighted binning) (AD 12-bit, 12-bit length output) */
    {
        .width  = 2088,
        .height = 1100,
        .min_hmax = 706,
        .min_vmax = 1148,
        .scale = 1,
        .min_shr = 3,
        .integration_offset = 256,
        .crop = {
            .left = 18,
            .top = 14,
            .width = 2048,
            .height = 1080,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_4_17_9_regs),
            .regs        = mode_4_17_9_regs,
        },
    },
   /* 17:9 Mode 5 — Horizontal/vertical 2/2-line binning (Horizontal and vertical weighted binning) (AD 10-bit, 12-bit length output) */
    /*{
        .width  = 2088,
        .height = 1100,
        .min_hmax = 607,
        .min_vmax = 1148,
        .scale = 1,
        .min_shr = 5,
        .integration_offset = 217,
        .crop = {
            .left = 18,
            .top = 14,
            .width = 2048,
            .height = 1080,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_5_17_9_regs),
            .regs        = mode_5_17_9_regs,
        },
    },*/
   /* 17:9 Mode 8 — Horizontal/vertical 3/3-line binning (AD 10-bit, 12-bit length output) */
    {
        .width  = 1392,
        .height = 740,
        .min_hmax = 520,
        .min_vmax = 1182,
        .scale = 1,
        .min_shr = 7,
        .integration_offset = 217,
        .crop = {
            .left = 12,
            .top = 14,
            .width = 1364,
            .height = 720,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_8_17_9_regs),
            .regs        = mode_8_17_9_regs,
        },
    },
   /* 17:9 Mode 9 — Vertical 1/3 subsampling horizontal 3 binning (AD 10-bit, 12-bit length output) */
    /*{
        .width  = 1392,
        .height = 740,
        .min_hmax = 520,
        .min_vmax = 394,
        .scale = 2,
        .min_shr = 2,
        .integration_offset = 217,
        .crop = {
            .left = 12,
            .top = 14,
            .width = 1364,
            .height = 720,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_9_17_9_regs),
            .regs        = mode_9_17_9_regs,
        },
    },*/
   /* 17:9 Mode 10 — Vertical 2/9 subsampling binning horizontal 3 binning (AD 10-bit, 12-bit length output) */
    {
        .width  = 1392,
        .height = 252,
        .min_hmax = 520,
        .min_vmax = 300,
        .scale = 1,
        .min_shr = 5,
        .integration_offset = 217,
        .crop = {
            .left = 12,
            .top = 10,
            .width = 1364,
            .height = 240,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_10_17_9_regs),
            .regs        = mode_10_17_9_regs,
        },
    },
   /* 4:3 Mode 1 — All-pixel scan mode (AD 12-bit, 12-bit length output) */
    {
        .width  = 3792,
        .height = 2840,
        .min_hmax = 1034,
        .min_vmax = 1444,
        .scale = 2,
        .min_shr = 5,
        .integration_offset = 256,
        .crop = {
            .left = 40,
            .top = 42,
            .width = 3704,
            .height = 2778,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_1_4_3_regs),
            .regs        = mode_1_4_3_regs,
        },
    },
   /* 4:3 Mode 1A — All-pixel scan mode (AD 12-bit, 12-bit length output) low noise */
    /*{
        .width  = 3792,
        .height = 2840,
        .min_hmax = 1192,
        .min_vmax = 1444,
        .scale = 2,
        .min_shr = 5,
        .integration_offset = 361,
        .crop = {
            .left = 40,
            .top = 42,
            .width = 3704,
            .height = 2778,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_1A_4_3_regs),
            .regs        = mode_1A_4_3_regs,
        },
    },*/
   /* 4:3 Mode 10 — Vertical 2/9 subsampling binning horizontal 3 binning (AD 10-bit, 12-bit length output) */
    {
        .width  = 1264,
        .height = 320,
        .min_hmax = 520,
        .min_vmax = 368,
        .scale = 1,
        .min_shr = 5,
        .integration_offset = 217,
        .crop = {
            .left = 14,
            .top = 10,
            .width = 1234,
            .height = 308,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_10_4_3_regs),
            .regs        = mode_10_4_3_regs,
        },
    },
};

static struct imx294_mode supported_modes_14bit[] = {
   /* 17:9 Mode 3 — Horizontal/vertical 2/2-line binning (Horizontal and vertical weighted binning) (AD 12-bit, 14-bit length output) */
    {
        .width  = 2088,
        .height = 1100,
        .min_hmax = 706,
        .min_vmax = 1148,
        .scale = 1,
        .min_shr = 5,
        .integration_offset = 256,
        .crop = {
            .left = 18,
            .top = 14,
            .width = 2048,
            .height = 1080,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_3_17_9_regs),
            .regs        = mode_3_17_9_regs,
        },
    },
   /* 4:3 Mode 0 — All-pixel scan mode (AD 14-bit, 14-bit length output) */
    {
        .width  = 3792,
        .height = 2840,
        .min_hmax = 1730,
        .min_vmax = 1444,
        .scale = 2,
        .min_shr = 5,
        .integration_offset = 551,
        .crop = {
            .left = 40,
            .top = 42,
            .width = 3704,
            .height = 2778,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_0_4_3_regs),
            .regs        = mode_0_4_3_regs,
        },
    },
};

/* Formats exposed per mode/bit depth */
static const u32 codes[] = {
    /* 10-bit modes. */
    MEDIA_BUS_FMT_SRGGB10_1X10,
    MEDIA_BUS_FMT_SGRBG10_1X10,
    MEDIA_BUS_FMT_SGBRG10_1X10,
    MEDIA_BUS_FMT_SBGGR10_1X10,
    /* 12-bit modes. */
    MEDIA_BUS_FMT_SRGGB12_1X12,
    MEDIA_BUS_FMT_SGRBG12_1X12,
    MEDIA_BUS_FMT_SGBRG12_1X12,
    MEDIA_BUS_FMT_SBGGR12_1X12,
    /* 14-bit modes. */
    MEDIA_BUS_FMT_SRGGB14_1X14,
    MEDIA_BUS_FMT_SGRBG14_1X14,
    MEDIA_BUS_FMT_SGBRG14_1X14,
    MEDIA_BUS_FMT_SBGGR14_1X14,
};


/* Regulators */
static const char * const imx294_supply_name[] = {
    "vana", /* 3.3V analog */
    "vdig", /* 1.1V core   */
    "vddl", /* 1.8V I/O    */
};

#define IMX294_NUM_SUPPLIES ARRAY_SIZE(imx294_supply_name)

/* --------------------------------------------------------------------------
 * State
 * --------------------------------------------------------------------------
 */

struct imx294 {
    struct v4l2_subdev sd;
    struct media_pad pad;
    struct device *clientdev;
    struct regmap *regmap;

    struct clk *xclk;
    const struct imx294_input_frequency *freq;

    unsigned int lane_count;
    unsigned int link_freq_idx;

    struct gpio_desc *reset_gpio;
    struct regulator_bulk_data supplies[IMX294_NUM_SUPPLIES];

    struct v4l2_ctrl_handler ctrl_handler;

    /* Controls */
    struct v4l2_ctrl *pixel_rate;
    struct v4l2_ctrl *link_freq;
    struct v4l2_ctrl *exposure;
    struct v4l2_ctrl *gain;
    struct v4l2_ctrl *hcg_ctrl;
    struct v4l2_ctrl *vflip;
    struct v4l2_ctrl *vblank;
    struct v4l2_ctrl *hblank;
    struct v4l2_ctrl *blacklevel;

    u16  hmax;
    u32  vmax;

    bool streaming;
};

/* Helpers */

static inline struct imx294 *to_imx294(struct v4l2_subdev *sd)
{
    return container_of(sd, struct imx294, sd);
}

static inline void get_mode_table(struct imx294 *imx294, unsigned int code,
                  const struct imx294_mode **mode_list,
                  unsigned int *num_modes)
{
    switch (code) {
    /* 14-bit */
    case MEDIA_BUS_FMT_SRGGB14_1X14:
    case MEDIA_BUS_FMT_SGRBG14_1X14:
    case MEDIA_BUS_FMT_SGBRG14_1X14:
    case MEDIA_BUS_FMT_SBGGR14_1X14:
        *mode_list = supported_modes_14bit;
        *num_modes = ARRAY_SIZE(supported_modes_14bit);
        break;
    /* 12-bit */
    case MEDIA_BUS_FMT_SRGGB12_1X12:
    case MEDIA_BUS_FMT_SGRBG12_1X12:
    case MEDIA_BUS_FMT_SGBRG12_1X12:
    case MEDIA_BUS_FMT_SBGGR12_1X12:
        *mode_list = supported_modes_12bit;
        *num_modes = ARRAY_SIZE(supported_modes_12bit);
        break;
    /* 12-bit */
    case MEDIA_BUS_FMT_SRGGB10_1X10:
    case MEDIA_BUS_FMT_SGRBG10_1X10:
    case MEDIA_BUS_FMT_SGBRG10_1X10:
    case MEDIA_BUS_FMT_SBGGR10_1X10:
        *mode_list = supported_modes_10bit;
        *num_modes = ARRAY_SIZE(supported_modes_10bit);
        break;
    default:
        *mode_list = NULL;
        *num_modes = 0;
    }
}

static u32 imx294_get_format_code(struct imx294 *imx294, u32 code)
{
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(codes); i++)
        if (codes[i] == code)
            return codes[i];
    return codes[0];
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

static void imx294_set_framing_limits(struct imx294 *imx294,
                      const struct imx294_mode *mode)
{
    u64 pixel_rate;
    u64 max_hblank;

    imx294->vmax = mode->min_vmax;
    imx294->hmax = mode->min_hmax;

    /* Pixel rate proxy: width * clock / min_hmax */
    pixel_rate = (u64)mode->width * IMX294_PIXEL_RATE * mode -> scale;
    do_div(pixel_rate, mode->min_hmax);
    __v4l2_ctrl_modify_range(imx294->pixel_rate, pixel_rate, pixel_rate, 1, pixel_rate);

    max_hblank = (u64)IMX294_HMAX_MAX * pixel_rate;
    do_div(max_hblank, IMX294_PIXEL_RATE);
    max_hblank -= mode->width;

    __v4l2_ctrl_modify_range(imx294->hblank, 0, max_hblank, 1, 0);
    __v4l2_ctrl_s_ctrl(imx294->hblank, 0);

    __v4l2_ctrl_modify_range(imx294->vblank,
                 mode->min_vmax - mode->height,
                 IMX294_VMAX_MAX - mode->height,
                 1, mode->min_vmax - mode->height);
    __v4l2_ctrl_s_ctrl(imx294->vblank, mode->min_vmax - mode->height);

    __v4l2_ctrl_modify_range(imx294->exposure, IMX294_EXPOSURE_MIN,
                 imx294->vmax - mode->min_shr, 1,
                 IMX294_EXPOSURE_DEFAULT);

    dev_info(imx294->clientdev, "Framing: VMAX=%u HMAX=%u pixel_rate=%llu\n",
        imx294->vmax, imx294->hmax, pixel_rate);
}

/* --------------------------------------------------------------------------
 * Controls
 * --------------------------------------------------------------------------
 */

static int imx294_set_ctrl(struct v4l2_ctrl *ctrl)
{
    struct imx294 *imx294 = container_of(ctrl->handler, struct imx294, ctrl_handler);
    const struct imx294_mode *mode, *mode_list;
    struct v4l2_subdev_state *state;
    struct v4l2_mbus_framefmt *fmt;
    unsigned int num_modes;
    int ret = 0;

    state = v4l2_subdev_get_locked_active_state(&imx294->sd);
    fmt = v4l2_subdev_state_get_format(state, 0);

    get_mode_table(imx294, fmt->code, &mode_list, &num_modes);
    mode = v4l2_find_nearest_size(mode_list, num_modes, width, height,
                      fmt->width, fmt->height);


    if (ctrl->id == V4L2_CID_VBLANK){
        /* Honour the VBLANK limits when setting exposure. */
        u32 current_exposure = imx294->exposure->cur.val;
        u64 max_exposure, min_exposure, vmax;

        vmax = ((u64)mode->height + ctrl->val);
        do_div(vmax, mode->scale);

        imx294->vmax = vmax;
        
        calculate_min_max_v4l2_cid_exposure(imx294->hmax, imx294->vmax, (u64)mode->min_shr, 0, mode->integration_offset, &min_exposure, &max_exposure);
        current_exposure = clamp_t(uint32_t, current_exposure, min_exposure, max_exposure);

        dev_info(imx294->clientdev,"exposure_max:%lld, exposure_min:%lld, current_exposure:%d\n",max_exposure, min_exposure, current_exposure);
        dev_info(imx294->clientdev,"\tVMAX:%d, HMAX:%d\n",imx294->vmax, imx294->hmax);
        __v4l2_ctrl_modify_range(imx294->exposure, min_exposure,max_exposure, 1,current_exposure);
    }

    /* Apply control only when powered (runtime active). */
    if (!pm_runtime_get_if_active(imx294->clientdev))
        return 0;

    switch (ctrl->id) {
    case V4L2_CID_EXPOSURE: {
        u32 shr = calculate_shr(ctrl->val, imx294->hmax, imx294->vmax, 0, mode->integration_offset);

        dev_info(imx294->clientdev, "EXPOSURE=%u -> SHR=%u (VMAX=%u HMAX=%u)\n",
            ctrl->val, shr, imx294->vmax, imx294->hmax);

        ret = cci_write(imx294->regmap, IMX294_REG_SHR, shr, NULL);
        if (ret)
            dev_err_ratelimited(imx294->clientdev, "SHR write failed (%d)\n", ret);
        break;
    }
    case V4L2_CID_ANALOGUE_GAIN:
        dev_info(imx294->clientdev, "ANALOG_GAIN=%u\n", ctrl->val);
        ret = cci_write(imx294->regmap, IMX294_REG_ANALOG_GAIN, ctrl->val, NULL);
        if (ret)
            dev_err_ratelimited(imx294->clientdev, "Gain write failed (%d)\n", ret);
        break;
    case V4L2_CID_VBLANK: {
        u32 vmax = mode->height + ctrl->val;
        u64 vblk;

        do_div(vmax, mode->scale);
        imx294->vmax = vmax;

        dev_info(imx294->clientdev, "VBLANK=%u -> VMAX=%u\n", ctrl->val, imx294->vmax);

        ret = cci_write(imx294->regmap, IMX294_REG_VMAX, imx294->vmax, NULL);
        if (ret)
            dev_err_ratelimited(imx294->clientdev, "VMAX write failed (%d)\n", ret);

        vblk = imx294->vmax - mode->min_vmax;
        dev_info(imx294->clientdev,"\tvblk : %lld\n",vblk);
        ret = cci_write(imx294->regmap, IMX294_REG_PSSLVS1, vblk, NULL);
        ret = cci_write(imx294->regmap, IMX294_REG_PSSLVS2, vblk, NULL);
        ret = cci_write(imx294->regmap, IMX294_REG_PSSLVS3, vblk, NULL);
        ret = cci_write(imx294->regmap, IMX294_REG_PSSLVS0, vblk, NULL);
        if(vblk <= 5)
            ret = cci_write(imx294->regmap, IMX294_REG_PSSLVS4, 0, NULL);
        
        else
            ret = cci_write(imx294->regmap, IMX294_REG_PSSLVS4, vblk - 5, NULL);

        break;
    }
    case V4L2_CID_HBLANK: {
        u64 pixel_rate = (u64)mode->width * IMX294_PIXEL_RATE;
        u64 hmax;

        do_div(pixel_rate, mode->min_hmax);
        hmax = (u64)(mode->width + ctrl->val) * IMX294_PIXEL_RATE;
        do_div(hmax, pixel_rate);
        imx294->hmax = (u32)hmax;

        dev_info(imx294->clientdev, "HBLANK=%u -> HMAX=%u\n", ctrl->val, imx294->hmax);

        ret = cci_write(imx294->regmap, IMX294_REG_HMAX, imx294->hmax, NULL);
        if (ret)
            dev_err_ratelimited(imx294->clientdev, "HMAX write failed (%d)\n", ret);
        ret = cci_write(imx294->regmap, IMX294_REG_HCOUNT1, imx294->hmax, NULL);
        ret = cci_write(imx294->regmap, IMX294_REG_HCOUNT2, imx294->hmax, NULL);
        break;
    }
    case V4L2_CID_VFLIP:
        ret = cci_write(imx294->regmap, IMX294_REG_MDVREV, ctrl->val, NULL);
        if (ret)
            dev_err_ratelimited(imx294->clientdev, "VFLIP write failed (%d)\n", ret);
        break;
    case V4L2_CID_BRIGHTNESS: {
        u16 blacklevel = min_t(u32, ctrl->val, 4095);

        ret = cci_write(imx294->regmap, IMX294_REG_BLKLEVEL, blacklevel, NULL);
        if (ret)
            dev_err_ratelimited(imx294->clientdev, "BLKLEVEL write failed (%d)\n", ret);
        break;
    }
    case V4L2_CID_IMX585_HCG_GAIN:
        dev_info(imx294->clientdev, "HCG=%u\n", ctrl->val);
        ret = cci_write(imx294->regmap, IMX294_REG_MCOVGAIN, ctrl->val, NULL);
        if (ret)
            dev_err_ratelimited(imx294->clientdev, "MCOVGAIN write failed (%d)\n", ret);
        break;
    default:
        dev_info(imx294->clientdev, "Unhandled ctrl %s: id=0x%x, val=0x%x\n",
            ctrl->name, ctrl->id, ctrl->val);
        break;
    }

    pm_runtime_put(imx294->clientdev);
    return ret;
}

static const struct v4l2_ctrl_ops imx294_ctrl_ops = {
    .s_ctrl = imx294_set_ctrl,
};

static const struct v4l2_ctrl_config imx294_cfg_hcg = {
    .ops  = &imx294_ctrl_ops,
    .id   = V4L2_CID_IMX585_HCG_GAIN,
    .name = "HCG Enable",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min  = 0,
    .max  = 1,
    .step = 1,
    .def  = 0,
};


static int imx294_init_controls(struct imx294 *imx294)
{
    struct v4l2_ctrl_handler *hdl = &imx294->ctrl_handler;
    struct v4l2_fwnode_device_properties props;
    int ret;

    ret = v4l2_ctrl_handler_init(hdl, 16);

    /* Read-only, updated per mode */
    imx294->pixel_rate = v4l2_ctrl_new_std(hdl, &imx294_ctrl_ops,
                           V4L2_CID_PIXEL_RATE,
                           1, UINT_MAX, 1, 1);
    imx294->link_freq =
        v4l2_ctrl_new_int_menu(hdl, &imx294_ctrl_ops, V4L2_CID_LINK_FREQ,
                       0, 0, imx294_link_freq_menu);
    if (imx294->link_freq)
        imx294->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

    imx294->vblank = v4l2_ctrl_new_std(hdl, &imx294_ctrl_ops,
                       V4L2_CID_VBLANK, 0, 0xFFFFF, 1, 0);
    imx294->hblank = v4l2_ctrl_new_std(hdl, &imx294_ctrl_ops,
                       V4L2_CID_HBLANK, 0, 0xFFFF, 1, 0);
    imx294->blacklevel = v4l2_ctrl_new_std(hdl, &imx294_ctrl_ops,
                           V4L2_CID_BRIGHTNESS, 0, 0xFFFF, 1,
                           IMX294_BLKLEVEL_DEFAULT);

    imx294->exposure = v4l2_ctrl_new_std(hdl, &imx294_ctrl_ops,
                         V4L2_CID_EXPOSURE,
                         IMX294_EXPOSURE_MIN, IMX294_EXPOSURE_MAX,
                         IMX294_EXPOSURE_STEP, IMX294_EXPOSURE_DEFAULT);

    imx294->gain = v4l2_ctrl_new_std(hdl, &imx294_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
                     IMX294_ANA_GAIN_MIN, IMX294_ANA_GAIN_MAX,
                     IMX294_ANA_GAIN_STEP, IMX294_ANA_GAIN_DEFAULT);

    imx294->vflip = v4l2_ctrl_new_std(hdl, &imx294_ctrl_ops,
                      V4L2_CID_VFLIP, 0, 1, 1, 0);

    imx294->hcg_ctrl = v4l2_ctrl_new_custom(hdl, &imx294_cfg_hcg, NULL);

    if (hdl->error) {
        ret = hdl->error;
        dev_err(imx294->clientdev, "control init failed (%d)\n", ret);
        goto err_free;
    }

    ret = v4l2_fwnode_device_parse(imx294->clientdev, &props);
    if (ret)
        goto err_free;

    ret = v4l2_ctrl_new_fwnode_properties(hdl, &imx294_ctrl_ops, &props);
    if (ret)
        goto err_free;

    imx294->sd.ctrl_handler = hdl;
    return 0;

err_free:
    v4l2_ctrl_handler_free(hdl);
    return ret;
}

static void imx294_free_controls(struct imx294 *imx294)
{
    v4l2_ctrl_handler_free(imx294->sd.ctrl_handler);
}

/* --------------------------------------------------------------------------
 * Pad ops / formats
 * --------------------------------------------------------------------------
 */

static int imx294_enum_mbus_code(struct v4l2_subdev *sd,
                 struct v4l2_subdev_state *sd_state,
                 struct v4l2_subdev_mbus_code_enum *code)
{
    struct imx294 *imx294 = to_imx294(sd);
    unsigned int entries;
    const u32 *tbl;

    tbl = codes;
    entries = ARRAY_SIZE(codes) / 4;

    if (code->index >= entries)
        return -EINVAL;

    code->code = imx294_get_format_code(imx294, tbl[code->index * 4]);
    return 0;
}

static int imx294_enum_frame_size(struct v4l2_subdev *sd,
                  struct v4l2_subdev_state *sd_state,
                  struct v4l2_subdev_frame_size_enum *fse)
{
    struct imx294 *imx294 = to_imx294(sd);
    const struct imx294_mode *mode_list;
    unsigned int num_modes;

    get_mode_table(imx294, fse->code, &mode_list, &num_modes);
    if (fse->index >= num_modes)
        return -EINVAL;
    if (fse->code != imx294_get_format_code(imx294, fse->code))
        return -EINVAL;

    fse->min_width  = mode_list[fse->index].width;
    fse->max_width  = fse->min_width;
    fse->min_height = mode_list[fse->index].height;
    fse->max_height = fse->min_height;

    return 0;
}

static int imx294_set_pad_format(struct v4l2_subdev *sd,
                 struct v4l2_subdev_state *sd_state,
                 struct v4l2_subdev_format *fmt)
{
    struct imx294 *imx294 = to_imx294(sd);
    const struct imx294_mode *mode_list, *mode;
    unsigned int num_modes;
    struct v4l2_mbus_framefmt *format;
    struct v4l2_rect *crop;

    /* Normalize requested code to what we really support */
    fmt->format.code = imx294_get_format_code(imx294, fmt->format.code);

    get_mode_table(imx294, fmt->format.code, &mode_list, &num_modes);
    mode = v4l2_find_nearest_size(mode_list, num_modes, width, height,
                                  fmt->format.width, fmt->format.height);

    fmt->format.width        = mode->width;
    fmt->format.height       = mode->height;
    fmt->format.field        = V4L2_FIELD_NONE;
    fmt->format.colorspace   = V4L2_COLORSPACE_RAW;
    fmt->format.ycbcr_enc    = V4L2_YCBCR_ENC_601;
    fmt->format.quantization = V4L2_QUANTIZATION_FULL_RANGE;
    fmt->format.xfer_func    = V4L2_XFER_FUNC_NONE;

    /* Update TRY/ACTIVE format kept by the framework */
    format = v4l2_subdev_state_get_format(sd_state, 0);
    *format = fmt->format;

    /* Keep the crop in sync with the selected mode */
    crop = v4l2_subdev_state_get_crop(sd_state, 0);
    *crop = mode->crop;

    /* Update control ranges only for ACTIVE config */
    if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
        imx294_set_framing_limits(imx294, mode);

    return 0;
}



/* --------------------------------------------------------------------------
 * Stream on/off
 * --------------------------------------------------------------------------
 */

static int imx294_enable_streams(struct v4l2_subdev *sd,
                 struct v4l2_subdev_state *state, u32 pad,
                 u64 streams_mask)
{
    struct imx294 *imx294 = to_imx294(sd);
    const struct imx294_mode *mode_list, *mode;
    struct v4l2_mbus_framefmt *fmt;
    unsigned int n_modes;
    int ret;

    ret = pm_runtime_get_sync(imx294->clientdev);
    if (ret < 0) {
        pm_runtime_put_noidle(imx294->clientdev);
        return ret;
    }
    /* (XMSTA register = 1h, MSTSLV register = 1h) */
    cci_write(imx294->regmap, IMX294_REG_XMSTA_MSTSLV, 0x30, &ret);
    /* (SYS_MODE register = 1h) */
    cci_write(imx294->regmap, CCI_REG8(0x303C), 0x01, &ret);

    /* Configure PLL clocks based on the xclk */
    cci_multi_reg_write(imx294->regmap, imx294->freq->regs,
                imx294->freq->reg_count, &ret);

    dev_info(imx294->clientdev, "Using clk freq %d Hz",
        imx294->freq->mhz);

    ret = cci_multi_reg_write(imx294->regmap, mode_common_regs_stage1,
                  ARRAY_SIZE(mode_common_regs_stage1), NULL);
    if (ret) {
        dev_err(imx294->clientdev, "Failed to write common settings stage 1\n");
        goto err_rpm_put;
    }


    /* Select mode */
    fmt = v4l2_subdev_state_get_format(state, 0);

    get_mode_table(imx294, fmt->code, &mode_list, &n_modes);
    mode = v4l2_find_nearest_size(mode_list, n_modes, width, height,
                      fmt->width, fmt->height);

    dev_info(imx294->clientdev,"Set mode: %d x %d\n",mode->width,mode->height);

    ret = cci_multi_reg_write(imx294->regmap, mode->reg_list.regs,
                  mode->reg_list.num_of_regs, NULL);
    if (ret) {
        dev_err(imx294->clientdev, "Failed to write mode registers\n");
        goto err_rpm_put;
    }
    imx294_set_framing_limits(imx294, mode);
    /* Apply user controls after writing the base tables */
    ret = __v4l2_ctrl_handler_setup(imx294->sd.ctrl_handler);
    if (ret) {
        dev_err(imx294->clientdev, "Control handler setup failed\n");
        goto err_rpm_put;
    }

    usleep_range(10000,12000);
    ret = cci_multi_reg_write(imx294->regmap, mode_common_regs_stage2,
                  ARRAY_SIZE(mode_common_regs_stage2), NULL);
    if (ret) {
        dev_err(imx294->clientdev, "Failed to write common settings stage 2\n");
        goto err_rpm_put;
    }

    usleep_range(10000,12000);

    cci_write(imx294->regmap, IMX294_REG_XMSTA_MSTSLV, 0x20, &ret);
    cci_write(imx294->regmap, IMX294_REG_SYNCDRV, 0xA8, &ret);

    if (ret) {
        dev_err(imx294->clientdev, "Failed to write common settings stage 3\n");
        goto err_rpm_put;
    }

    dev_info(imx294->clientdev, "Streaming started\n");
    usleep_range(IMX294_STREAM_DELAY_US,
             IMX294_STREAM_DELAY_US + IMX294_STREAM_DELAY_RANGE_US);

    /* vflip cannot change during streaming */
    __v4l2_ctrl_grab(imx294->vflip, true);

    return 0;

err_rpm_put:
    pm_runtime_put_autosuspend(imx294->clientdev);
    return ret;
}

static int imx294_disable_streams(struct v4l2_subdev *sd,
                  struct v4l2_subdev_state *state, u32 pad,
                  u64 streams_mask)
{
    struct imx294 *imx294 = to_imx294(sd);
    int ret;

    ret = cci_write(imx294->regmap, IMX294_REG_MODE_SELECT, 0x01, NULL);
    if (ret)
        dev_err(imx294->clientdev, "Failed to stop streaming\n");

    __v4l2_ctrl_grab(imx294->vflip, false);

    pm_runtime_put_autosuspend(imx294->clientdev);

    return ret;
}

/* --------------------------------------------------------------------------
 * Power / runtime PM
 * --------------------------------------------------------------------------
 */

static int imx294_power_on(struct device *dev)
{
    struct v4l2_subdev *sd = dev_get_drvdata(dev);
    struct imx294 *imx294 = to_imx294(sd);
    int ret;

    dev_info(imx294->clientdev, "power_on\n");

    ret = regulator_bulk_enable(IMX294_NUM_SUPPLIES, imx294->supplies);
    if (ret) {
        dev_err(imx294->clientdev, "Failed to enable regulators\n");
        return ret;
    }

    ret = clk_prepare_enable(imx294->xclk);
    if (ret) {
        dev_err(imx294->clientdev, "Failed to enable clock\n");
        goto reg_off;
    }

    gpiod_set_value_cansleep(imx294->reset_gpio, 1);
    usleep_range(IMX294_XCLR_MIN_DELAY_US,
             IMX294_XCLR_MIN_DELAY_US + IMX294_XCLR_DELAY_RANGE_US);
    return 0;

reg_off:
    regulator_bulk_disable(IMX294_NUM_SUPPLIES, imx294->supplies);
    return ret;
}

static int imx294_power_off(struct device *dev)
{
    struct v4l2_subdev *sd = dev_get_drvdata(dev);
    struct imx294 *imx294 = to_imx294(sd);

    dev_info(imx294->clientdev, "power_off\n");

    gpiod_set_value_cansleep(imx294->reset_gpio, 0);
    regulator_bulk_disable(IMX294_NUM_SUPPLIES, imx294->supplies);
    clk_disable_unprepare(imx294->xclk);

    return 0;
}

/* --------------------------------------------------------------------------
 * Selection / state
 * --------------------------------------------------------------------------
 */

static int imx294_get_selection(struct v4l2_subdev *sd,
                struct v4l2_subdev_state *sd_state,
                struct v4l2_subdev_selection *sel)
{
    struct imx294 *imx294 = to_imx294(sd);
    const struct imx294_mode *mode_list, *mode;
    const struct v4l2_mbus_framefmt *fmt;
    unsigned int n_modes;

    fmt = v4l2_subdev_state_get_format(sd_state, 0);
    get_mode_table(imx294, fmt->code, &mode_list, &n_modes);
    mode = v4l2_find_nearest_size(mode_list, n_modes, width, height,
                                  fmt->width, fmt->height);

    switch (sel->target) {
    case V4L2_SEL_TGT_NATIVE_SIZE:
        sel->r.left   = 0;
        sel->r.top    = 0;
        sel->r.width  = mode->width;   /* pixel array (no blanking) */
        sel->r.height = mode->height;
        return 0;

    case V4L2_SEL_TGT_CROP_BOUNDS:
        sel->r.left   = 0;
        sel->r.top    = 0;
        sel->r.width  = mode->width;   /* full array bounds */
        sel->r.height = mode->height;
        return 0;

    case V4L2_SEL_TGT_CROP_DEFAULT:
        sel->r = mode->crop;                /* recommended default */
        return 0;

    case V4L2_SEL_TGT_CROP:
        sel->r = *v4l2_subdev_state_get_crop(sd_state, 0);
        return 0;

    default:
        return -EINVAL;
    }
}

static int imx294_init_state(struct v4l2_subdev *sd,
                 struct v4l2_subdev_state *state)
{
    struct v4l2_rect *crop;
    struct v4l2_subdev_format fmt = {
        .which  = V4L2_SUBDEV_FORMAT_TRY,
        .pad    = 0,
        .format = {
            .code   = MEDIA_BUS_FMT_SRGGB12_1X12,
            .width  = supported_modes_12bit[0].width,
            .height = supported_modes_12bit[0].height,
        },
    };

    imx294_set_pad_format(sd, state, &fmt);

    crop = v4l2_subdev_state_get_crop(state, 0);
    *crop = supported_modes_12bit[0].crop;

    return 0;
}

/* --------------------------------------------------------------------------
 * Subdev ops
 * --------------------------------------------------------------------------
 */

static const struct v4l2_subdev_video_ops imx294_video_ops = {
    .s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx294_pad_ops = {
    .enum_mbus_code = imx294_enum_mbus_code,
    .get_fmt        = v4l2_subdev_get_fmt,
    .set_fmt        = imx294_set_pad_format,
    .get_selection  = imx294_get_selection,
    .enum_frame_size = imx294_enum_frame_size,
    .enable_streams  = imx294_enable_streams,
    .disable_streams = imx294_disable_streams,
};

static const struct v4l2_subdev_internal_ops imx294_internal_ops = {
    .init_state = imx294_init_state,
};

static const struct v4l2_subdev_ops imx294_subdev_ops = {
    .video = &imx294_video_ops,
    .pad   = &imx294_pad_ops,
};

/* --------------------------------------------------------------------------
 * Probe / remove
 * --------------------------------------------------------------------------
 */

static int imx294_check_hwcfg(struct device *dev, struct imx294 *imx294)
{
    struct fwnode_handle *endpoint;
    struct v4l2_fwnode_endpoint ep = {
        .bus_type = V4L2_MBUS_CSI2_DPHY,
    };
    int ret = -EINVAL;

    endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
    if (!endpoint) {
        dev_err(dev, "endpoint node not found\n");
        return -EINVAL;
    }

    if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep)) {
        dev_err(dev, "could not parse endpoint\n");
        goto out_put;
    }

    if (ep.bus.mipi_csi2.num_data_lanes != 4) {
        dev_err(dev, "only 4 data lanes supported\n");
        goto out_free;
    }
    imx294->lane_count = ep.bus.mipi_csi2.num_data_lanes;
    dev_info(dev, "Data lanes: %u\n", imx294->lane_count);

    ret = 0;

out_free:
    v4l2_fwnode_endpoint_free(&ep);
out_put:
    fwnode_handle_put(endpoint);
    return ret;
}

static int imx294_get_regulators(struct imx294 *imx294)
{
    unsigned int i;

    for (i = 0; i < IMX294_NUM_SUPPLIES; i++)
        imx294->supplies[i].supply = imx294_supply_name[i];

    return devm_regulator_bulk_get(imx294->clientdev,
                       IMX294_NUM_SUPPLIES, imx294->supplies);
}

static int imx294_check_module_exists(struct imx294 *imx294)
{
    int ret;
    u64 val;

    /* No chip-id register; read a known register as a presence test */
    ret = cci_read(imx294->regmap, IMX294_REG_BLKLEVEL, &val, NULL);
    if (ret) {
        dev_err(imx294->clientdev, "register read failed (%d)\n", ret);
        return ret;
    }

    dev_info(imx294->clientdev, "Sensor detected\n");
    return 0;
}

static int imx294_probe(struct i2c_client *client)
{
    struct device *dev = &client->dev;
    struct imx294 *imx294;
    unsigned int xclk_freq;
    int ret, i;

    imx294 = devm_kzalloc(dev, sizeof(*imx294), GFP_KERNEL);
    if (!imx294)
        return -ENOMEM;

    v4l2_i2c_subdev_init(&imx294->sd, client, &imx294_subdev_ops);
    imx294->clientdev = dev;

    ret = imx294_check_hwcfg(dev, imx294);
    if (ret)
        return ret;

    imx294->regmap = devm_cci_regmap_init_i2c(client, 16);
    if (IS_ERR(imx294->regmap))
        return dev_err_probe(dev, PTR_ERR(imx294->regmap), "CCI init failed\n");

    imx294->xclk = devm_clk_get(dev, NULL);
    if (IS_ERR(imx294->xclk))
        return dev_err_probe(dev, PTR_ERR(imx294->xclk), "xclk missing\n");

    xclk_freq = clk_get_rate(imx294->xclk);
    for (i = 0; i < ARRAY_SIZE(imx294_frequencies); i++) {
        if (xclk_freq == imx294_frequencies[i].mhz) {
            imx294->freq = &imx294_frequencies[i];
            break;
        }
    }
    if (!imx294->freq) {
        dev_err(imx294->clientdev, "xclk frequency unsupported: %d Hz\n", xclk_freq);
        return -EINVAL;
    }

    ret = imx294_get_regulators(imx294);
    if (ret)
        return dev_err_probe(dev, ret, "regulators\n");

    imx294->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);

    /* Power on to probe the device */
    ret = imx294_power_on(dev);
    if (ret)
        return ret;

    ret = imx294_check_module_exists(imx294);
    if (ret)
        goto err_power_off;

    pm_runtime_set_active(dev);
    pm_runtime_get_noresume(dev);
    pm_runtime_enable(dev);
    pm_runtime_set_autosuspend_delay(dev, 1000);
    pm_runtime_use_autosuspend(dev);

    ret = imx294_init_controls(imx294);
    if (ret)
        goto err_pm;

    imx294->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    imx294->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
    imx294->sd.internal_ops = &imx294_internal_ops;

    imx294->pad.flags = MEDIA_PAD_FL_SOURCE;

    ret = media_entity_pads_init(&imx294->sd.entity, 1, &imx294->pad);
    if (ret) {
        dev_err(dev, "entity pads init failed: %d\n", ret);
        goto err_ctrls;
    }

    imx294->sd.state_lock = imx294->ctrl_handler.lock;
    ret = v4l2_subdev_init_finalize(&imx294->sd);
    if (ret) {
        dev_err_probe(dev, ret, "subdev init\n");
        goto err_entity;
    }

    ret = v4l2_async_register_subdev_sensor(&imx294->sd);
    if (ret) {
        dev_err(dev, "sensor subdev register failed: %d\n", ret);
        goto err_entity;
    }

    pm_runtime_mark_last_busy(dev);
    pm_runtime_put_autosuspend(dev);
    return 0;

err_entity:
    media_entity_cleanup(&imx294->sd.entity);
err_ctrls:
    imx294_free_controls(imx294);
err_pm:
    pm_runtime_disable(dev);
    pm_runtime_set_suspended(dev);
err_power_off:
    imx294_power_off(dev);
    return ret;
}

static void imx294_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct imx294 *imx294 = to_imx294(sd);

    v4l2_async_unregister_subdev(sd);
    v4l2_subdev_cleanup(sd);
    media_entity_cleanup(&sd->entity);
    imx294_free_controls(imx294);

    pm_runtime_disable(imx294->clientdev);
    if (!pm_runtime_status_suspended(imx294->clientdev))
        imx294_power_off(imx294->clientdev);
    pm_runtime_set_suspended(imx294->clientdev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx294_pm_ops, imx294_power_off,
                 imx294_power_on, NULL);

static const struct of_device_id imx294_of_match[] = {
    { .compatible = "sony,imx294" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx294_of_match);

static struct i2c_driver imx294_i2c_driver = {
    .driver = {
        .name  = "imx294",
        .pm    = pm_ptr(&imx294_pm_ops),
        .of_match_table = imx294_of_match,
    },
    .probe  = imx294_probe,
    .remove = imx294_remove,
};
module_i2c_driver(imx294_i2c_driver);

MODULE_AUTHOR("Will Whang <will@willwhang.com>");
MODULE_DESCRIPTION("Sony IMX294 sensor driver");
MODULE_LICENSE("GPL");