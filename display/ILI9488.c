/**
 * @file ILI9488.c
 *
 * ILI9488.pdf [ILI9488_DS_V1.13_20110805]
 *
 * [references]
 * - https://www.newhavendisplay.com/app_notes/ILI9488.pdf
 * - Linux Source [v5.9-rc4] "drivers/staging/fbtft/fb_ILI9488.c"
 * - https://github.com/adafruit/Adafruit_ILI9488/blob/master/Adafruit_ILI9488.cpp
 * - https://os.mbed.com/users/dreschpe/code/SPI_TFT_ILI9488
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "ILI9488.h"
#if USE_ILI9488 != 0

#include <stdio.h>
#include <stdbool.h>
#include LV_DRV_DISP_INCLUDE
#include LV_DRV_DELAY_INCLUDE

/*********************
 *      DEFINES
 *********************/
#define ILI9488_CMD_MODE    0
#define ILI9488_DATA_MODE   1

#define ILI9488_TFTWIDTH    320
#define ILI9488_TFTHEIGHT   480

/* Level 1 Commands -------------- [section] Description */

#define ILI9488_NOP         0x00 /* [8.2.1 ] No Operation / Terminate Frame Memory Write */
#define ILI9488_SWRESET     0x01 /* [8.2.2 ] Software Reset */
#define ILI9488_RDDIDIF     0x04 /* [8.2.3 ] Read Display Identification Information */
#define ILI9488_RDDST       0x09 /* [8.2.4 ] Read Display Status */
#define ILI9488_RDDPM       0x0A /* [8.2.5 ] Read Display Power Mode */
#define ILI9488_RDDMADCTL   0x0B /* [8.2.6 ] Read Display MADCTL */
#define ILI9488_RDDCOLMOD   0x0C /* [8.2.7 ] Read Display Pixel Format */
#define ILI9488_RDDIM       0x0D /* [8.2.8 ] Read Display Image Mode */
#define ILI9488_RDDSM       0x0E /* [8.2.9 ] Read Display Signal Mode */
#define ILI9488_RDDSDR      0x0F /* [8.2.10] Read Display Self-Diagnostic Result */
#define ILI9488_SLPIN       0x10 /* [8.2.11] Enter Sleep Mode */
#define ILI9488_SLPOUT      0x11 /* [8.2.12] Leave Sleep Mode */
#define ILI9488_PTLON       0x12 /* [8.2.13] Partial Display Mode ON */
#define ILI9488_NORON       0x13 /* [8.2.14] Normal Display Mode ON */
#define ILI9488_DINVOFF     0x20 /* [8.2.15] Display Inversion OFF */
#define ILI9488_DINVON      0x21 /* [8.2.16] Display Inversion ON */
#define ILI9488_PIXELOFF    0x22 /* [8.2.17] todo! */
#define ILI9488_PIXELON     0x23 /* [8.2.18] todo!*/
#define ILI9488_DISPOFF     0x28 /* [8.2.18] Display OFF*/
#define ILI9488_DISPON      0x29 /* [8.2.19] Display ON*/
#define ILI9488_CASET       0x2A /* [8.2.20] Column Address Set */
#define ILI9488_PASET       0x2B /* [8.2.21] Page Address Set */
#define ILI9488_RAMWR       0x2C /* [8.2.22] Memory Write */
#define ILI9488_RAMRD       0x2E /* [8.2.24] Memory Read */
#define ILI9488_PTLAR       0x30 /* [8.2.25] Partial Area */
#define ILI9488_VSCRDEF     0x33 /* [8.2.26] Veritcal Scrolling Definition */
#define ILI9488_TEOFF       0x34 /* [8.2.27] Tearing Effect Line OFF */
#define ILI9488_TEON        0x35 /* [8.2.28] Tearing Effect Line ON */
#define ILI9488_MADCTL      0x36 /* [8.2.29] Memory Access Control */
#define ILI9488_VSCRSADD    0x37 /* [8.2.30] Vertical Scrolling Start Address */
#define ILI9488_IDMOFF      0x38 /* [8.2.31] Idle Mode OFF */
#define ILI9488_IDMON       0x39 /* [8.2.32] Idle Mode ON */
#define ILI9488_PIXSET      0x3A /* [8.2.33] Pixel Format Set */
#define ILI9488_WRMEMCONT   0x3C /* [8.2.34] Write Memory Continue */
#define ILI9488_RDMEMCONT   0x3E /* [8.2.35] Read Memory Continue */
#define ILI9488_SETSCANTE   0x44 /* [8.2.36] Set Tear Scanline */
#define ILI9488_GETSCAN     0x45 /* [8.2.37] Get Scanline */
#define ILI9488_WRDISBV     0x51 /* [8.2.38] Write Display Brightness Value */
#define ILI9488_RDDISBV     0x52 /* [8.2.39] Read Display Brightness Value */
#define ILI9488_WRCTRLD     0x53 /* [8.2.40] Write Control Display */
#define ILI9488_RDCTRLD     0x54 /* [8.2.41] Read Control Display */
#define ILI9488_WRCABC      0x55 /* [8.2.42] Write Content Adaptive Brightness Control Value */
#define ILI9488_RDCABC      0x56 /* [8.2.43] Read Content Adaptive Brightness Control Value */
#define ILI9488_WRCABCMIN   0x5E /* [8.2.44] Write CABC Minimum Brightness */
#define ILI9488_RDCABCMIN   0x5F /* [8.2.45] Read CABC Minimum Brightness */
#define ILI9488_RDID1       0xDA /* [8.2.46] Read ID1 - Manufacturer ID (user) */
#define ILI9488_RDID2       0xDB /* [8.2.47] Read ID2 - Module/Driver version (supplier) */
#define ILI9488_RDID3       0xDC /* [8.2.48] Read ID3 - Module/Driver version (user) */

/* Level 2 Commands -------------- [section] Description */

#define ILI9488_IFMODE      0xB0 /* [8.3.1 ] Interface Mode Control */
#define ILI9488_FRMCTR1     0xB1 /* [8.3.2 ] Frame Rate Control (In Normal Mode/Full Colors) */
#define ILI9488_FRMCTR2     0xB2 /* [8.3.3 ] Frame Rate Control (In Idle Mode/8 colors) */
#define ILI9488_FRMCTR3     0xB3 /* [8.3.4 ] Frame Rate control (In Partial Mode/Full Colors) */
#define ILI9488_INVTR       0xB4 /* [8.3.5 ] Display Inversion Control */
#define ILI9488_PRCTR       0xB5 /* [8.3.6 ] Blanking Porch Control */
#define ILI9488_DISCTRL     0xB6 /* [8.3.7 ] Display Function Control */
#define ILI9488_ETMOD       0xB7 /* [8.3.8 ] Entry Mode Set */
#define ILI9488_BLCTRL2     0xB9 /* [8.3.10] Backlight Control 1 - Grayscale Histogram still picture mode */
#define ILI9488_BLCTRL3     0xBA /* [8.3.11] Backlight Control 2 - Grayscale Thresholds UI mode */
#define ILI9488_HSLANECTL   0xBE /*  [] todo */
#define ILI9488_PWCTRL1     0xC0 /* [8.3.16] Power Control 1 - GVDD */
#define ILI9488_PWCTRL2     0xC1 /* [8.3.17] Power Control 2 - step-up factor for operating voltage */
#define ILI9488_PWCTRL3     0xC2 /* [8.3.16] todo */
#define ILI9488_PWCTRL4     0xC3 /* [8.3.17] todo  */
#define ILI9488_PWCTRL5     0xC4 /* [8.3.17] todo */
#define ILI9488_VMCTRL1     0xC5 /* [8.3.18] VCOM Control 1 - Set VCOMH and VCOML */
#define ILI9488_CABCTRL1    0xC6 /* [8.3.19] todo */
#define ILI9488_CABCTRL2    0xC8 /* [8.3.19] todo */
#define ILI9488_CABCTRL3    0xC9 /* [8.3.19] todo */
#define ILI9488_CABCTRL4    0xCA /* [8.3.19] todo */
#define ILI9488_CABCTRL5    0xCB /* [8.3.19] todo */
#define ILI9488_CABCTRL6    0xCC /* [8.3.19] todo */
#define ILI9488_CABCTRL7    0xCD /* [8.3.19] todo */
#define ILI9488_CABCTRL8    0xCE /* [8.3.19] todo */
#define ILI9488_CABCTRL9    0xCF /* [8.3.19] todo */
#define ILI9488_NVMWR       0xD0 /* [8.3.20] NV Memory Write */
#define ILI9488_NVMPKEY     0xD1 /* [8.3.21] NV Memory Protection Key */
#define ILI9488_RDNVM       0xD2 /* [8.3.22] NV Memory Status Read */
#define ILI9488_RDID4       0xD3 /* [8.3.23] Read ID4 - IC Device Code */
#define ILI9488_ADJCTRL     0xD7
#define ILI9488_RDIDV       0xD8
#define ILI9488_PGAMCTRL    0xE0 /* [8.3.24] Positive Gamma Control */
#define ILI9488_NGAMCTRL    0xE1 /* [8.3.25] Negative Gamma Correction */
#define ILI9488_DGAMCTRL1   0xE2 /* [8.3.26] Digital Gamma Control 1 */
#define ILI9488_DGAMCTRL2   0xE3 /* [8.3.27] Digital Gamma Control 2 */
#define ILI9488_SETIMGFUNC  0xE9 /* todo */
#define ILI9488_ADJCTRL2    0xF2 /* todo */
#define ILI9488_ADJCTRL3    0xF7 /* todo */
#define ILI9488_ADJCTRL4    0xF8 /* todo */
#define ILI9488_ADJCTRL5    0xF9 /* todo */
#define ILI9488_SPIRDSET    0xFB /* todo */
#define ILI9488_ADJCTRL6    0xFC /* todo */
#define ILI9488_ADJCTRL7    0xFF /* todo */

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static inline void ILI9488_write(int mode, uint8_t data);
static inline void ILI9488_write_array(int mode, uint8_t *data, uint16_t len);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Initialize the ILI9488 display controller
 */
void ILI9488_init(void)
{
    uint8_t data[15];

    /* hardware reset */
    LV_DRV_DISP_SPI_CS(1);
    LV_DRV_DISP_CMD_DATA(ILI9488_DATA_MODE);
    LV_DRV_DISP_RST(0);
    LV_DRV_DELAY_US(50);
    LV_DRV_DISP_RST(1);
    LV_DRV_DELAY_MS(5);

    /* software reset */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_SWRESET);
    LV_DRV_DELAY_MS(5);
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_DISPOFF);

    /* Positive Gamma control  */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_PGAMCTRL);
    data[0] = 0x00;
    data[1] = 0x03;
    data[2] = 0x09;
    data[3] = 0x08;
    data[4] = 0x16;
    data[5] = 0x0A;
    data[6] = 0x3F;
    data[7] = 0x78;
    data[8] = 0x4C;
    data[9] = 0x09;
    data[10] = 0x0A;
    data[11] = 0x08;
    data[12] = 0x16;
    data[13] = 0x1A;
    data[14] = 0x0F;
    ILI9488_write_array(ILI9488_DATA_MODE, data, 15);

    /* Negative Gamma control*/
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_NGAMCTRL);
    data[0] = 0x00;
    data[1] = 0x16;
    data[2] = 0x19;
    data[3] = 0x03;
    data[4] = 0x0F;
    data[5] = 0x05;
    data[6] = 0x32;
    data[7] = 0x45;
    data[8] = 0x46;
    data[9] = 0x04;
    data[10] = 0x0E;
    data[11] = 0x0D;
    data[12] = 0x35;
    data[13] = 0x37;
    data[14] = 0x0F;
    ILI9488_write_array(ILI9488_DATA_MODE, data, 15);

    /* Power control 1 */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_PWCTRL1);
    data[0] = 0x17;
    data[1] = 0x15;
    ILI9488_write_array(ILI9488_DATA_MODE, data, 2);

    /* Power control 2 */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_PWCTRL2);
    ILI9488_write(ILI9488_DATA_MODE, 0x41);

    /* VCOM Control 1 */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_VMCTRL1);
    data[0] = 0x00;
    data[1] = 0x12;
    data[2] = 0x80;
    ILI9488_write_array(ILI9488_DATA_MODE, data, 3);

    /* Memory Access control */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_MADCTL);
    ILI9488_write(ILI9488_DATA_MODE, (0x20 | 0x80));

    /* Pixel format set */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_PIXSET);
    ILI9488_write(ILI9488_DATA_MODE, 0x66);

    /* Interface mode control */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_IFMODE);
    ILI9488_write(ILI9488_DATA_MODE, 0x00);

    /* Frame rate control normal */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_FRMCTR1);
    ILI9488_write(ILI9488_DATA_MODE, 0xA0);

    /* Display inversion control */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_INVTR);
    ILI9488_write(ILI9488_DATA_MODE, 0x02);

#if ILI9488_TEARING
    /* Tearing effect off */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_TEOFF);

    /* Tearing effect on */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_TEON);
#endif

    /* Display function control */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_DISCTRL);
    data[0] = 0x02;
    data[1] = 0x02;
    ILI9488_write_array(ILI9488_DATA_MODE, data, 2);

    /* Set image function */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_SETIMGFUNC);
    ILI9488_write(ILI9488_DATA_MODE, 0x00);
    
    /* Write control display */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_WRCTRLD);
    ILI9488_write(ILI9488_DATA_MODE, 0x28);

    /* Write display brightness */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_WRDISBV);
    ILI9488_write(ILI9488_DATA_MODE, 0x7F);

    /* Adjust control 3 */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_ADJCTRL3);
    data[0] = 0xA9;
    data[1] = 0x51;
    data[2] = 0x2C;
    data[3] = 0x02;
    ILI9488_write_array(ILI9488_DATA_MODE, data, 4);

    /* exit sleep mode */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_SLPOUT);

    LV_DRV_DELAY_MS(100);

    /* display on */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_DISPON);

    LV_DRV_DELAY_MS(20);
}

void ILI9488_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
    if(area->x2 < 0 || area->y2 < 0 || area->x1 > (ILI9488_HOR_RES - 1) || area->y1 > (ILI9488_VER_RES - 1)) {
        lv_disp_flush_ready(drv);
        return;
    }

    /* Truncate the area to the screen */
    int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
    int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
    int32_t act_x2 = area->x2 > ILI9488_HOR_RES - 1 ? ILI9488_HOR_RES - 1 : area->x2;
    int32_t act_y2 = area->y2 > ILI9488_VER_RES - 1 ? ILI9488_VER_RES - 1 : area->y2;

    int32_t y;
    uint8_t data[4];
    int32_t len = len = (act_x2 - act_x1 + 1) * 2;
    lv_coord_t w = (area->x2 - area->x1) + 1;

    /* window horizontal */
    ILI9488_write(ILI9488_CMD_MODE, ILI9488_CASET);
    data[0] = (act_x1 >> 8) & 0xFF;
    data[1] = act_x1 & 0xFF;
    data[2] = (act_x2 >> 8) & 0xFF;
    data[3] = act_x2 & 0xFF;
    ILI9488_write_array(ILI9488_DATA_MODE, data, 4);

    /* window vertical */
    ILI9488_write(ILI9488_CMD_MODE,  ILI9488_PASET);
    data[0] = (act_y1 >> 8) & 0xFF;
    data[1] = act_y1 & 0xFF;
    data[2] = (act_y2 >> 8) & 0xFF;
    data[3] = act_y2 * 0xFF;
    ILI9488_write_array(ILI9488_DATA_MODE, data, 4);

    ILI9488_write(ILI9488_CMD_MODE, ILI9488_RAMWR);

    for(y = act_y1; y <= act_y2; y++) {
        ILI9488_write_array(ILI9488_DATA_MODE, (uint8_t *)color_p, len);
        color_p += w;
    }

    lv_disp_flush_ready(drv);
}

void ILI9488_rotate(int degrees, bool bgr)
{
    uint8_t color_order = MADCTL_RGB;

    if(bgr)
        color_order = MADCTL_BGR;

    ILI9488_write(ILI9488_CMD_MODE, ILI9488_MADCTL);

    switch(degrees) {
    case 270:
        ILI9488_write(ILI9488_DATA_MODE, MADCTL_MV | color_order);
        break;
    case 180:
        ILI9488_write(ILI9488_DATA_MODE, MADCTL_MY | color_order);
        break;
    case 90:
        ILI9488_write(ILI9488_DATA_MODE, MADCTL_MX | MADCTL_MY | MADCTL_MV | color_order);
        break;
    case 0:
        /* fall-through */
    default:
        ILI9488_write(ILI9488_DATA_MODE, MADCTL_MX | color_order);
        break;
    }
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * Write byte
 * @param mode sets command or data mode for write
 * @param byte the byte to write
 */
static inline void ILI9488_write(int mode, uint8_t data)
{
    LV_DRV_DISP_CMD_DATA(mode);
    LV_DRV_DISP_SPI_WR_BYTE(data);
}

/**
 * Write byte array
 * @param mode sets command or data mode for write
 * @param data the byte array to write
 * @param len the length of the byte array
 */
static inline void ILI9488_write_array(int mode, uint8_t *data, uint16_t len)
{
    LV_DRV_DISP_CMD_DATA(mode);
    LV_DRV_DISP_SPI_WR_ARRAY(data, len);
}

#endif
